#ifndef IR_VISITOR_EMPTY_H_
#define IR_VISITOR_EMPTY_H_ IR_VISITOR_EMPTY_H_

#include "ir_visitor.h"

class EmptyIRStmtVisitor : public IRStmtVisitor {
 public:
  void Visit(CompoundIRStmt *) override {}
  void Visit(LetIRStmt *) override {}
  void Visit(DeclAssignIRStmt *) override {}
  void Visit(AssignIRStmt *) override {}
  void Visit(DeclAccIRStmt *) override {}
  void Visit(AccIRStmt *) override {}
  void Visit(AccForceIRStmt *) override {}
  void Visit(IfIRStmt *) override {}
  void Visit(ForIRStmt *) override {}
  void Visit(ContinueIRStmt *) override {}
  void Visit(AccEnergyIRStmt *) override {}
  void Visit(InvalidIRStmt *) override {}
  void Visit(LetComplexFunCallIRStmt *) override {}
  void Visit(DeclListIRStmt *) override {}
  void Visit(AddListIRStmt *) override {}
  void Visit(PerAtomActionIRStmt *) override {}
};

class TraverseIRVisitor : public IRStmtVisitor, public IRExprVisitor {
 public:
  virtual void AcceptStmt(std::unique_ptr<IRStmt> *stmt) {
    (*stmt)->Accept(this);
  }
  virtual void AcceptCompoundStmt(std::unique_ptr<CompoundIRStmt> *stmt) {
    (*stmt)->Accept(this);
  }
  virtual void AcceptExpr(std::unique_ptr<IRExpr> *expr) {
    (*expr)->Accept(this);
  }
  void Visit(CompoundIRStmt *stmt) override {
    for (auto &st : stmt->body) {
      AcceptStmt(&st);
    }
  }
  void Visit(LetIRStmt *stmt) override { AcceptExpr(&stmt->value); }
  void Visit(DeclAssignIRStmt *) override {}
  void Visit(AssignIRStmt *stmt) override { AcceptExpr(&stmt->value); }
  void Visit(DeclAccIRStmt *) override {}
  void Visit(AccIRStmt *stmt) override { AcceptExpr(&stmt->value); }
  void Visit(AccForceIRStmt *stmt) override {
    AcceptExpr(&stmt->index);
    if (stmt->value_x) AcceptExpr(&stmt->value_x);
    if (stmt->value_y) AcceptExpr(&stmt->value_y);
    if (stmt->value_z) AcceptExpr(&stmt->value_z);
  }
  void Visit(IfIRStmt *stmt) override {
    AcceptExpr(&stmt->cond);
    AcceptCompoundStmt(&stmt->then);
    AcceptCompoundStmt(&stmt->otherwise);
  }
  void Visit(ForIRStmt *stmt) override {
    AcceptExpr(&stmt->initial);
    AcceptExpr(&stmt->top_value);
    AcceptCompoundStmt(&stmt->body);
  }
  void Visit(ContinueIRStmt *stmt) override { AcceptExpr(&stmt->cond); }
  void Visit(AccEnergyIRStmt *stmt) override { AcceptExpr(&stmt->value); }
  void Visit(InvalidIRStmt *) override {}
  void Visit(DeclListIRStmt *) override {}
  void Visit(AddListIRStmt *stmt) override {
    AcceptExpr(&stmt->value);
    if (stmt->half_check) AcceptExpr(&stmt->half_check);
  }
  void Visit(PerAtomActionIRStmt *stmt) override {
    for (auto &arg : stmt->args) {
      AcceptExpr(&arg);
    }
  }
  void Visit(LetComplexFunCallIRStmt *stmt) override {
    for (auto &arg : stmt->args) {
      AcceptExpr(&arg);
    }
  }
  void Visit(BinIRExpr *expr) override {
    AcceptExpr(&expr->left);
    AcceptExpr(&expr->right);
  }
  void Visit(UnIRExpr *expr) override { AcceptExpr(&expr->inner); }
  void Visit(RefIRExpr *) override {}
  void Visit(LitIRExpr *) override {}
  void Visit(FunCallIRExpr *expr) override {
    for (auto &arg : expr->args) {
      AcceptExpr(&arg);
    }
  }
  void Visit(InvalidIRExpr *) override {}
  void Visit(LookupIRExpr *expr) override {
    for (auto &arg : expr->args) {
      AcceptExpr(&arg);
    }
  }
};

class EmptyIRExprVisitor : public IRExprVisitor {
 public:
  void Visit(BinIRExpr *) override {}
  void Visit(UnIRExpr *) override {}
  void Visit(RefIRExpr *) override {}
  void Visit(LitIRExpr *) override {}
  void Visit(FunCallIRExpr *) override {}
  void Visit(InvalidIRExpr *) override {}
  void Visit(LookupIRExpr *) override {}
};

#endif  // IR_VISITOR_EMPTY_H_
