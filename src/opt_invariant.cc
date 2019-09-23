#include "opt.h"

#include <map>
#include <set>

#include "ir.h"
#include "ir_visitor_empty.h"

// annotate every expr, stmt with the loops it depends on
// keep that list ordered by level
// in the end, move each statement to the level it belongs
// Q: Should we also move stuff out of `if` conditions?
// A: Loops: Probably yes, everything else: probably no
// Problem: E.g. sum_omega needs to stay guarded

//

namespace potc {

namespace opt {

namespace {

struct InvariantCodeVisitor : TraverseIRVisitor {
  bool DEBUG = false;
  std::map<IRIdentifier, std::set<IRIdentifier>> dependencies;
  std::map<IRStmt*, std::set<IRIdentifier>> stmt_dependencies;
  std::vector<IRIdentifier> current_scope;
  std::vector<std::set<IRIdentifier>> current_dependencies;
  std::vector<std::unique_ptr<IRStmt>> move_to_front;
  void Combine(const std::set<IRIdentifier>& with) {
    for (auto& elem : with) {
      current_dependencies.back().insert(elem);
    }
  }
  void AcceptStmt(std::unique_ptr<IRStmt>* stmt) override {
    current_dependencies.push_back({});
    (*stmt)->Accept(this);
    stmt_dependencies[(*stmt).get()] = current_dependencies.back();
    current_dependencies.pop_back();
  }
  void AcceptCompoundStmt(std::unique_ptr<CompoundIRStmt>* stmt) override {
    (*stmt)->Accept(this);
    for (auto& sub_st : (*stmt)->body) {
      Combine(stmt_dependencies[sub_st.get()]);
    }
  }
  void Visit(CompoundIRStmt* stmt) {
    for (int i = 0; i < stmt->body.size(); i++) {
      auto& st = stmt->body[i];
      AcceptStmt(&st);
      for (auto& sub_st : move_to_front) {
        stmt->body.insert(stmt->body.begin() + i, std::move(sub_st));
        i += 1;
      }
      move_to_front.clear();
    }
  }
  void Visit(ForIRStmt* stmt) override {
    current_scope.push_back(stmt->index);
    AcceptExpr(&stmt->initial);
    AcceptExpr(&stmt->top_value);
    dependencies[stmt->index] = current_dependencies.back();
    dependencies[stmt->index].insert(stmt->index);
    AcceptCompoundStmt(&stmt->body);

    // check all stmts in ``body'' whether they include ``stmt->index'' as a
    // dependency
    // if not, extract and move out
    for (int i = 0; i < stmt->body->body.size(); i++) {
      auto& sub_st = stmt->body->body[i];
      if (stmt_dependencies[sub_st.get()].count(stmt->index) == 0) {
        if (DEBUG) printf("%p invariant candidate: %p\n", stmt, sub_st.get());
        move_to_front.push_back(std::move(sub_st));
        stmt->body->body.erase(stmt->body->body.begin() + i);
        i -= 1;
      }
    }
    current_scope.pop_back();
  }
  void Visit(DeclAssignIRStmt* stmt) override {
    TraverseIRVisitor::Visit(stmt);
    if (!current_scope.empty())
      current_dependencies.back().insert(current_scope.back());
    dependencies[stmt->name] = current_dependencies.back();
  }
  void Visit(AssignIRStmt* stmt) override {
    TraverseIRVisitor::Visit(stmt);
    Combine(dependencies[stmt->name]);
  }
  void Visit(DeclAccIRStmt* stmt) override {
    TraverseIRVisitor::Visit(stmt);
    if (!current_scope.empty())
      current_dependencies.back().insert(current_scope.back());
    dependencies[stmt->name] = current_dependencies.back();
  }
  void Visit(AccIRStmt* stmt) override {
    TraverseIRVisitor::Visit(stmt);
    Combine(dependencies[stmt->name]);
  }
  void Visit(DeclListIRStmt* stmt) override {
    TraverseIRVisitor::Visit(stmt);
    if (!current_scope.empty())
      current_dependencies.back().insert(current_scope.back());
    dependencies[stmt->name] = current_dependencies.back();
  }
  void Visit(AddListIRStmt* stmt) override {
    TraverseIRVisitor::Visit(stmt);
    Combine(dependencies[stmt->name]);
  }
  void Visit(LetIRStmt* stmt) override {
    TraverseIRVisitor::Visit(stmt);
    dependencies[stmt->name] = current_dependencies.back();
  }
  void Visit(LetComplexFunCallIRStmt* stmt) override {
    TraverseIRVisitor::Visit(stmt);
    dependencies[stmt->name] = current_dependencies.back();
    for (auto& name : stmt->other_names) {
      dependencies[name] = current_dependencies.back();
    }
  }
  // how to handle declacc/declass?
  // no need to, are dependency free
  void Visit(RefIRExpr* expr) override { Combine(dependencies[expr->ref]); }
};
}  // namespace

void InvariantCodeMotion(CompoundIRStmt* program) {
  InvariantCodeVisitor vis;
  program->Accept(&vis);
}

}  // namespace opt

}  // namespace potc
