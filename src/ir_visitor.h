#ifndef IR_VISITOR_H_
#define IR_VISITOR_H_ IR_VISITOR_H_

class CompoundIRStmt;
class LetIRStmt;
class DeclAssignIRStmt;
class AssignIRStmt;
class DeclAccIRStmt;
class AccIRStmt;
class AccForceIRStmt;
class IfIRStmt;
class ForIRStmt;
class ContinueIRStmt;
class AccEnergyIRStmt;
class InvalidIRStmt;
class LetComplexFunCallIRStmt;
class DeclListIRStmt;
class AddListIRStmt;
class PerAtomActionIRStmt;

class IRStmtVisitor {
 public:
  virtual ~IRStmtVisitor() {}
  virtual void Visit(CompoundIRStmt *) = 0;
  virtual void Visit(LetIRStmt *) = 0;
  virtual void Visit(DeclAssignIRStmt *) = 0;
  virtual void Visit(AssignIRStmt *) = 0;
  virtual void Visit(DeclAccIRStmt *) = 0;
  virtual void Visit(AccIRStmt *) = 0;
  virtual void Visit(AccForceIRStmt *) = 0;
  virtual void Visit(IfIRStmt *) = 0;
  virtual void Visit(ForIRStmt *) = 0;
  virtual void Visit(ContinueIRStmt *) = 0;
  virtual void Visit(AccEnergyIRStmt *) = 0;
  virtual void Visit(InvalidIRStmt *) = 0;
  virtual void Visit(LetComplexFunCallIRStmt *) = 0;
  virtual void Visit(DeclListIRStmt *) = 0;
  virtual void Visit(AddListIRStmt *) = 0;
  virtual void Visit(PerAtomActionIRStmt *) = 0;
};

class BinIRExpr;
class UnIRExpr;
class RefIRExpr;
class LitIRExpr;
class FunCallIRExpr;
class LookupIRExpr;
class InvalidIRExpr;

class IRExprVisitor {
 public:
  virtual ~IRExprVisitor() {}
  virtual void Visit(BinIRExpr *) = 0;
  virtual void Visit(UnIRExpr *) = 0;
  virtual void Visit(RefIRExpr *) = 0;
  virtual void Visit(LitIRExpr *) = 0;
  virtual void Visit(FunCallIRExpr *) = 0;
  virtual void Visit(LookupIRExpr *) = 0;
  virtual void Visit(InvalidIRExpr *) = 0;
};

#endif  // IR_VISITOR_H_
