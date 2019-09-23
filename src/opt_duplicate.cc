// Copyright 2018 Markus Hoehnerbach

#include "opt.h"

#include <algorithm>
#include <map>
#include <set>
#include <vector>

#include "ir.h"
#include "ir_build.h"
#include "ir_visitor_empty.h"

namespace potc {

namespace opt {

namespace {

struct AtomizeVisitor : public TraverseIRVisitor {
  IRIdentifierContext* ir_ctx;
  std::vector<std::vector<IRExpr*>> expr_stack;
  std::vector<std::vector<IRStmt*>> before_stack;
  std::vector<IRStmt*> stmt_stack;
  void AcceptStmt(std::unique_ptr<IRStmt>* stmt) {
    stmt_stack.push_back(stmt->get());
    (*stmt)->Accept(this);
    stmt_stack.pop_back();
  }
  void AcceptExpr(std::unique_ptr<IRExpr>* expr) override {
    (*expr)->Accept(this);
    expr_stack.back().push_back(expr->get());
    before_stack.back().push_back(stmt_stack.back());
  }
  void HandleCompoundStmt(CompoundIRStmt* stmt) {
    using namespace potc::ir::build::stmt;
    expr_stack.push_back({});
    before_stack.push_back({});
    stmt->Accept(this);
    assert(expr_stack.back().size() == before_stack.back().size());
    auto& body = stmt->body;
    for (int i = 0; i < expr_stack.back().size(); i++) {
      auto it = std::find_if(body.begin(), body.end(), [&](auto& e) {
        return e.get() == before_stack.back()[i];
      });
      assert(it != body.end());
      body.insert(it, Let(ir_ctx->Next("atom"), expr_stack.back()[i]->Clone()));
    }
    expr_stack.pop_back();
    before_stack.pop_back();
  }
  void AcceptCompoundStmt(std::unique_ptr<CompoundIRStmt>* stmt) override {
    HandleCompoundStmt(stmt->get());
  }
  void Visit(ForIRStmt* stmt) {
    // Do not atomize ForIRStmt boundaries
    // Some further optimizations rely on them not being RefIRExprs
    AcceptCompoundStmt(&stmt->body);
  }
  void Visit(ContinueIRStmt* stmt) {
    // Do not atomize ContinueIRStmts
    // Some further optimizations rely on them being BinIRStmts
  }
};

class DetectDuplicatesVisitor : public TraverseIRVisitor {
  bool remove_stmt_;
  std::vector<std::map<std::string, IRIdentifier>> available;
  std::vector<std::map<std::string, LetComplexFunCallIRStmt*>>
      available_complex;
  std::map<IRIdentifier, IRIdentifier> renames;
  template <class T>
  bool ShouldRemove(const T& stmt) {
    remove_stmt_ = false;
    stmt->Accept(this);
    bool ret = remove_stmt_;
    remove_stmt_ = false;
    return ret;
  }
  void RemoveThisStmt() { remove_stmt_ = true; }

  void AcceptExpr(std::unique_ptr<IRExpr>* expr) override {
    using namespace potc::ir::build;

    auto repr = (*expr)->ToString();
    for (auto& scope : available) {
      if (scope.count(repr) > 0) {
        (*expr).reset(Ref(scope[repr]).release());
        return;
      }
    }
    (*expr)->Accept(this);
    repr = (*expr)->ToString();
    for (auto& scope : available) {
      if (scope.count(repr) > 0) {
        (*expr).reset(Ref(scope[repr]).release());
        return;
      }
    }
  }
  void PushScope() {
    available.push_back({});
    available_complex.push_back({});
  }
  void PopScope() {
    available.pop_back();
    available_complex.pop_back();
  }

 public:
  DetectDuplicatesVisitor() : remove_stmt_(false) {}
  void Visit(CompoundIRStmt* stmt) override {
    PushScope();
    for (int i = 0; i < stmt->body.size(); i++) {
      if (ShouldRemove(stmt->body[i])) {
        assert(stmt->body[i]->isa<LetIRStmt>() ||
               stmt->body[i]->isa<ForIRStmt>() ||
               stmt->body[i]->isa<IfIRStmt>() ||
               stmt->body[i]->isa<LetComplexFunCallIRStmt>());
        stmt->body.erase(stmt->body.begin() + i);
        i -= 1;
      }
    }
    if (stmt->body.empty()) RemoveThisStmt();
    PopScope();
  }
  void Visit(LetIRStmt* stmt) override {
    using namespace potc::ir::build;

    AcceptExpr(&stmt->value);

    // In each scope, check if an equivalent value already exists
    auto repr = (stmt->value)->ToString();
    bool found = false;
    for (auto& scope : available) {
      if (scope.count(repr) > 0) {
        found = true;
        // TODO(markus) Y not stmt->value = Ref(scope[repr])
        (stmt->value).reset(Ref(scope[repr]).release());
        break;
      }
    }

    // TODO(markus) under which circumstances is found false, but IsRef true?
    // if (found && IsRefExpr(stmt->value.get())) {
    if (stmt->value->isa<RefIRExpr>()) {
      auto id = stmt->value->asa<RefIRExpr>()->ref;
      if (0 && !found) {
        printf("%s would rename to %s\n", stmt->name.ToString().c_str(),
               id.ToString().c_str());
      } else {
        renames[stmt->name] = id;
        RemoveThisStmt();
        return;
      }
    }
    available.back()[repr] = stmt->name;
  }
  void Visit(IfIRStmt* stmt) override {
    stmt->cond->Accept(this);
    PushScope();
    bool remove_then = ShouldRemove(stmt->then);
    PopScope();
    PushScope();
    bool remove_otherwise = ShouldRemove(stmt->otherwise);
    PopScope();
    if (remove_then && remove_otherwise) {
      RemoveThisStmt();
    }
  }
  void Visit(ForIRStmt* stmt) override {
    PushScope();
    stmt->initial->Accept(this);
    stmt->top_value->Accept(this);
    if (ShouldRemove(stmt->body)) {
      RemoveThisStmt();
    }
    PopScope();
  }
  void Visit(LetComplexFunCallIRStmt* stmt) override {
    TraverseIRVisitor::Visit(stmt);

    // check if there already exists an expression for the value, i.e.
    // ToString(0)
    // TODO(markus) rename this to something better
    auto repr = stmt->ToString(0);
    LetComplexFunCallIRStmt* found = nullptr;
    for (auto& scope : available_complex) {
      if (scope.count(repr) > 0) {
        found = scope[repr];
        break;
      }
    }

    if (found != nullptr) {
      renames[stmt->name] = found->name;
      assert(stmt->other_names.size() == found->other_names.size());
      for (int i = 0; i < stmt->other_names.size(); i++) {
        renames[stmt->other_names[i]] = found->other_names[i];
      }
      RemoveThisStmt();
      return;
    } else {
      available_complex.back()[repr] = stmt;
      return;
    }
  }
  void Visit(BinIRExpr* expr) {
    TraverseIRVisitor::Visit(expr);
    // Make sure that a / b can be transformed to a * (1 / b)
    // if 1 / b exists as an expression.
    // if (expr->op == "/" && !expr->left->isa<LitIRExpr>()) {
    //  using namespace potc::ir::build;

    //  auto new_expr = Div(Lit("1"), expr->right->Clone());
    //  AcceptExpr(&new_expr);

    //  if (new_expr->isa<RefIRExpr>()) {
    //    expr->op = "*";
    //    expr->right = std::move(new_expr);
    //  }
    //}
  }
  void Visit(RefIRExpr* expr) override {
    if (renames.count(expr->ref) > 0) {
      expr->ref = renames[expr->ref];
    }
  }
};
}  // namespace

void DetectDuplicates(CompoundIRStmt* program, IRIdentifierContext* ctx) {
  // AtomizeVisitor avis;
  // avis.ir_ctx = ctx;
  // avis.HandleCompoundStmt(program);

  DetectDuplicatesVisitor vis;
  vis.Visit(program);
  // use local value numberin to identify candidates
  // compare candidates for equality modulo associativity
  // replace usages using translation table
  // traverse code forwards
}
}  // namespace opt
}  // namespace potc
