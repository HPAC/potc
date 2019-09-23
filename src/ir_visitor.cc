#include "ir_visitor.h"
#include "ir.h"

void CompoundIRStmt::Accept(IRStmtVisitor* v) { v->Visit(this); }
void LetIRStmt::Accept(IRStmtVisitor* v) { v->Visit(this); }
void DeclAssignIRStmt::Accept(IRStmtVisitor* v) { v->Visit(this); }
void AssignIRStmt::Accept(IRStmtVisitor* v) { v->Visit(this); }
void DeclAccIRStmt::Accept(IRStmtVisitor* v) { v->Visit(this); }
void AccIRStmt::Accept(IRStmtVisitor* v) { v->Visit(this); }
void AccForceIRStmt::Accept(IRStmtVisitor* v) { v->Visit(this); }
void IfIRStmt::Accept(IRStmtVisitor* v) { v->Visit(this); }
void ForIRStmt::Accept(IRStmtVisitor* v) { v->Visit(this); }
void ContinueIRStmt::Accept(IRStmtVisitor* v) { v->Visit(this); }
void AccEnergyIRStmt::Accept(IRStmtVisitor* v) { v->Visit(this); }
void InvalidIRStmt::Accept(IRStmtVisitor* v) { v->Visit(this); }
void LetComplexFunCallIRStmt::Accept(IRStmtVisitor* v) { v->Visit(this); }
void DeclListIRStmt::Accept(IRStmtVisitor* v) { v->Visit(this); }
void AddListIRStmt::Accept(IRStmtVisitor* v) { v->Visit(this); }
void PerAtomActionIRStmt::Accept(IRStmtVisitor* v) { v->Visit(this); }
// -----------------------------------------------------------------------------
void BinIRExpr::Accept(IRExprVisitor* v) { v->Visit(this); }
void UnIRExpr::Accept(IRExprVisitor* v) { v->Visit(this); }
void RefIRExpr::Accept(IRExprVisitor* v) { v->Visit(this); }
void LitIRExpr::Accept(IRExprVisitor* v) { v->Visit(this); }
void FunCallIRExpr::Accept(IRExprVisitor* v) { v->Visit(this); }
void LookupIRExpr::Accept(IRExprVisitor* v) { v->Visit(this); }
void InvalidIRExpr::Accept(IRExprVisitor* v) { v->Visit(this); }
