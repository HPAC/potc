// Copyright 2018 Markus Hoehnerbach

#include "opt.h"

#include <map>
#include <set>

#include "ir.h"                // NOLINT
#include "ir_visitor_empty.h"  // NOLINT

namespace potc {

namespace opt {

namespace {

struct EliminateUnusedVisitor : public TraverseIRVisitor {
  bool remove_stmt_;
  std::set<IRIdentifier> used;
  std::map<IRIdentifier, int> used_count;
  template <class T>
  bool ShouldRemove(const T& stmt) {
    remove_stmt_ = false;
    stmt->Accept(this);
    bool ret = remove_stmt_;
    remove_stmt_ = false;
    return ret;
  }
  void RemoveThisStmt() { remove_stmt_ = true; }

 public:
  EliminateUnusedVisitor() : remove_stmt_(false) {}
  void Visit(CompoundIRStmt* stmt) override {
    bool has_invalid = false;
    for (int i = stmt->body.size() - 1; i >= 0; i--) {
      if (ShouldRemove(stmt->body[i])) {
        stmt->body.erase(stmt->body.begin() + i);
      } else if (stmt->body[i]->isa<InvalidIRStmt>()) {
        has_invalid = true;
      }
    }
    if (stmt->body.empty() || has_invalid ||
        stmt->body.back()->isa<ContinueIRStmt>())
      RemoveThisStmt();
  }
  void Visit(LetIRStmt* stmt) override {
    if (used.count(stmt->name)) {
      stmt->value->Accept(this);
    } else {
      RemoveThisStmt();
    }
  }
  void Visit(DeclAssignIRStmt* stmt) override {
    if (!used.count(stmt->name)) RemoveThisStmt();
  }
  void Visit(AssignIRStmt* stmt) override {
    if (used.count(stmt->name)) {
      stmt->value->Accept(this);
    } else {
      RemoveThisStmt();
    }
  }
  void Visit(DeclAccIRStmt* stmt) override {
    if (!used.count(stmt->name)) RemoveThisStmt();
  }
  void Visit(AccIRStmt* stmt) override {
    if (used.count(stmt->name)) {
      stmt->value->Accept(this);
    } else {
      RemoveThisStmt();
    }
  }
  void Visit(IfIRStmt* stmt) override {
    bool remove_then = ShouldRemove(stmt->then);
    bool remove_otherwise = ShouldRemove(stmt->otherwise);
    if (remove_then && remove_otherwise) {
      RemoveThisStmt();
    } else {
      stmt->cond->Accept(this);
    }
  }
  void Visit(ForIRStmt* stmt) override {
    if (ShouldRemove(stmt->body)) {
      RemoveThisStmt();
    } else {
      stmt->initial->Accept(this);
      stmt->top_value->Accept(this);
    }
  }
  void Visit(ContinueIRStmt* stmt) override {
    // TODO(markus) should remove if alone in loop
    stmt->cond->Accept(this);
  }
  void Visit(InvalidIRStmt* stmt) override {
    // TODO(markus) should only remove if allow removal higher up
  }
  void Visit(LetComplexFunCallIRStmt* stmt) override {
    int uses = 0;
    uses += used.count(stmt->name);
    for (auto& n : stmt->other_names) {
      uses += used.count(n);
    }
    if (uses == 0) {
      RemoveThisStmt();
    } else {
      for (auto& arg : stmt->args) {
        arg->Accept(this);
      }
    }
  }
  void Visit(RefIRExpr* expr) {
    used.insert(expr->ref);
    used_count[expr->ref] += 1;
  }
};

struct InlineLetsVisitor : public TraverseIRVisitor {
  bool remove_stmt_;
  std::map<IRIdentifier, int> used_count;
  std::vector<std::map<IRIdentifier, std::unique_ptr<IRExpr>>> used_value;
  template <class T>
  bool ShouldRemove(const T& stmt) {
    remove_stmt_ = false;
    stmt->Accept(this);
    bool ret = remove_stmt_;
    remove_stmt_ = false;
    return ret;
  }
  void RemoveThisStmt() { remove_stmt_ = true; }
  std::unique_ptr<IRExpr> result;
  void AcceptExpr(std::unique_ptr<IRExpr>* expr) override {
    result.reset();
    (*expr)->Accept(this);
    if (result.get() != NULL) {
      // printf("Rewriting %s with %s.\n", (*expr)->ToString().c_str(),
      // result->ToString().c_str());
      *expr = std::move(result);
    }
  }

 public:
  InlineLetsVisitor() : remove_stmt_(false) { used_value.push_back({}); }
  void Visit(CompoundIRStmt* stmt) override {
    for (int i = 0; i < stmt->body.size(); i++) {
      if (ShouldRemove(stmt->body[i])) {
        stmt->body.erase(stmt->body.begin() + i);
        i -= 1;
      }
    }
    if (stmt->body.empty()) RemoveThisStmt();
  }
  void Visit(LetIRStmt* stmt) override {
    AcceptExpr(&stmt->value);
    if (used_count[stmt->name] == 1) {
      used_value.back()[stmt->name] = stmt->value->Clone();
      // RemoveThisStmt();
    }
  }
  void Visit(IfIRStmt* stmt) override {
    bool remove_then = ShouldRemove(stmt->then);
    bool remove_otherwise = ShouldRemove(stmt->otherwise);
    if (remove_then && remove_otherwise) {
      RemoveThisStmt();
    } else {
      AcceptExpr(&stmt->cond);
    }
  }
  void Visit(ForIRStmt* stmt) override {
    used_value.push_back({});
    if (ShouldRemove(stmt->body)) {
      RemoveThisStmt();
    } else {
      AcceptExpr(&stmt->initial);
      AcceptExpr(&stmt->top_value);
    }
    used_value.pop_back();
  }
  void Visit(ContinueIRStmt* stmt) override {
    // removed if alone in loop (see CompoundIRStmt)
    AcceptExpr(&stmt->cond);
  }
  void Visit(InvalidIRStmt* stmt) override {
    // only removed if allow removal higher up (see CompundIRStmt)
  }
  void Visit(RefIRExpr* expr) {
    if (used_count[expr->ref] == 1) {
      auto& tmp = used_value.back()[expr->ref];
      // printf("Want rewrite %s: ", expr->ref.ToString().c_str());
      if (tmp.get()) {
        // printf("YES %s\n", tmp->ToString().c_str());
        // result = std::move(tmp);
        result = tmp->Clone();
        assert(result.get() != NULL);
        return;
      }
      // printf("\n");
    }
  }
};

}  // namespace

void EliminateUnused(CompoundIRStmt* program) {
  EliminateUnusedVisitor vis;
  vis.Visit(program);
}

void EliminateUnusedIntegrate(CompoundIRStmt* program) {
  EliminateUnusedVisitor vis;
  vis.Visit(program);
  InlineLetsVisitor vis_let;
  vis_let.used_count = vis.used_count;
  vis_let.Visit(program);
  EliminateUnusedVisitor vis_clean;
  vis_clean.Visit(program);
}
}  // namespace opt
}  // namespace potc
