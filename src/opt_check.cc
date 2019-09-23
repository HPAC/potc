// Copyright 2018 Markus Hoehnerbach

#include "opt.h"

#include <set>
#include <vector>

#include "ir.h"
#include "ir_visitor_empty.h"

namespace potc {

namespace opt {

namespace {

class CheckLegalVisitor : public TraverseIRVisitor {
  std::vector<std::set<IRIdentifier>> declared;

 public:
  CheckLegalVisitor() {}
  void Visit(CompoundIRStmt* stmt) override {
    declared.push_back({});
    for (auto& elem : stmt->body) {
      elem->Accept(this);
    }
    declared.pop_back();
  }
  void Visit(LetIRStmt* stmt) override {
    stmt->value->Accept(this);
    declared.back().insert(stmt->name);
  }
  void Visit(DeclAssignIRStmt* stmt) override {
    declared.back().insert(stmt->name);
  }
  void Visit(DeclAccIRStmt* stmt) override {
    declared.back().insert(stmt->name);
  }
  void Visit(ForIRStmt* stmt) override {
    stmt->initial->Accept(this);
    declared.push_back({});
    declared.back().insert(stmt->index);
    stmt->top_value->Accept(this);
    stmt->body->Accept(this);
    declared.pop_back();
  }
  void Visit(LetComplexFunCallIRStmt* stmt) override {
    for (auto& a : stmt->args) {
      a->Accept(this);
    }
    declared.back().insert(stmt->name);
    for (auto& n : stmt->other_names) {
      declared.back().insert(n);
    }
  }
  void Visit(RefIRExpr* expr) override {
    bool found = false;
    for (auto& scope : declared)
      if (scope.count(expr->ref) > 0) found = true;
    assert(found);
  }

  void Visit(DeclListIRStmt* stmt) override {
    declared.back().insert(stmt->name);
  }
};
}  // namespace

void CheckLegal(CompoundIRStmt* program) {
  CheckLegalVisitor vis;
  vis.Visit(program);
  // use local value numberin to identify candidates
  // compare candidates for equality modulo associativity
  // replace usages using translation table
  // traverse code forwards
}
}  // namespace opt
}  // namespace potc
