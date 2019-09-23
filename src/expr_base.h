#ifndef EXPR_BASE_H_
#define EXPR_BASE_H_

class AllAtomsListExpr;
class NeighborsListExpr;
class NumericalConstantExpr;
class IdentifierExpr;
class SumExpr;
class LetExpr;
class ImplicitExpr;
class PiecewiseExpr;
class TypematchExpr;
class AddExpr;
class MulExpr;
class PowExpr;

class ExprVisitor {
 public:
  virtual ~ExprVisitor() {}
  virtual void Visit(AllAtomsListExpr* expr) = 0;
  virtual void Visit(NeighborsListExpr* expr) = 0;
  virtual void Visit(NumericalConstantExpr* expr) = 0;
  virtual void Visit(IdentifierExpr* expr) = 0;
  virtual void Visit(SumExpr* expr) = 0;
  virtual void Visit(LetExpr* expr) = 0;
  virtual void Visit(ImplicitExpr* expr) = 0;
  virtual void Visit(PiecewiseExpr* expr) = 0;
  virtual void Visit(AddExpr* expr) = 0;
  virtual void Visit(MulExpr* expr) = 0;
  virtual void Visit(PowExpr* expr) = 0;
  virtual void Visit(TypematchExpr* expr) = 0;
};

class Expr {
 public:
  virtual ~Expr() = default;
  virtual void Accept(ExprVisitor* vis) = 0;
};

// template <class R>
// class ExprReturnVisitor : public ExprVisitor {
//  std::stack<R> return_stack_;
//
// public:
//  virtual R VisitReturnAllAtomsList(AllAtomsListExpr& expr) = 0;
//  virtual R VisitReturnNeighborsList(NeighborsListExpr& expr) = 0;
//  virtual R VisitReturnNumericalConstant(NumericalConstantExpr& expr) = 0;
//  virtual R VisitReturnIdentifier(IdentifierExpr& expr) = 0;
//  virtual R VisitReturnSum(SumExpr& expr) = 0;
//  virtual R VisitReturnLet(LetExpr& expr) = 0;
//  virtual R VisitReturnImplicit(ImplicitExpr& expr) = 0;
//  virtual R VisitReturnPiecewise(PiecewiseExpr& expr) = 0;
//  virtual R VisitReturnAdd(AddExpr& expr) = 0;
//  virtual R VisitReturnMul(MulExpr& expr) = 0;
//  virtual R VisitReturnPow(PowExpr& expr) = 0;
//  virtual R VisitReturnTypematch(TypematchExpr& expr) = 0;
//
//  virtual R Accept(Expr& expr) {
//    expr.Accept(*this);
//    auto result = std::move(return_stack_.top());
//    return_stack_.pop();
//    return result;
//  }
//
//  void VisitAllAtomsList(AllAtomsListExpr& expr) override {
//    return_stack_.push(VisitReturnAllAtomsList(expr));
//  }
//  void VisitNeighborsList(NeighborsListExpr& expr) override {
//    return_stack_.push(VisitReturnNeighborsList(expr));
//  }
//  void VisitNumericalConstant(NumericalConstantExpr& expr) override {
//    return_stack_.push(VisitReturnNumericalConstant(expr));
//  }
//  void VisitIdentifier(IdentifierExpr& expr) override {
//    return_stack_.push(VisitReturnIdentifier(expr));
//  }
//  void VisitSum(SumExpr& expr) override {
//    return_stack_.push(VisitReturnSum(expr));
//  }
//  void VisitLet(LetExpr& expr) override {
//    return_stack_.push(VisitReturnLet(expr));
//  }
//  void VisitImplicit(ImplicitExpr& expr) override {
//    return_stack_.push(VisitReturnImplicit(expr));
//  }
//  void VisitPiecewise(PiecewiseExpr& expr) override {
//    return_stack_.push(VisitReturnPiecewise(expr));
//  }
//  void VisitAdd(AddExpr& expr) override {
//    return_stack_.push(VisitReturnAdd(expr));
//  }
//  void VisitMul(MulExpr& expr) override {
//    return_stack_.push(VisitReturnMul(expr));
//  }
//  void VisitPow(PowExpr& expr) override {
//    return_stack_.push(VisitReturnPow(expr));
//  }
//};
//
// template <class R, class S>
// class ExprReturnStateVisitor : public ExprVisitor {
//  std::stack<R> return_stack_;
//  std::stack<S*> state_stack_;
//
// public:
//  virtual R VisitReturnAllAtomsList(AllAtomsListExpr& expr, S& state) = 0;
//  virtual R VisitReturnNeighborsList(NeighborsListExpr& expr, S& state) = 0;
//  virtual R VisitReturnNumericalConstant(NumericalConstantExpr& expr,
//                                         S& state) = 0;
//  virtual R VisitReturnIdentifier(IdentifierExpr& expr, S& state) = 0;
//  virtual R VisitReturnSum(SumExpr& expr, S& state) = 0;
//  virtual R VisitReturnLet(LetExpr& expr, S& state) = 0;
//  virtual R VisitReturnImplicit(ImplicitExpr& expr, S& state) = 0;
//  virtual R VisitReturnPiecewise(PiecewiseExpr& expr, S& state) = 0;
//  virtual R VisitReturnAdd(AddExpr& expr, S& state) = 0;
//  virtual R VisitReturnMul(MulExpr& expr, S& state) = 0;
//  virtual R VisitReturnPow(PowExpr& expr, S& state) = 0;
//
//  virtual R Accept(Expr& expr, S& state) {
//    state_stack_.push(&state);
//    expr.Accept(*this);
//    state_stack_.pop();
//    auto result = std::move(return_stack_.top());
//    return_stack_.pop();
//    return result;
//  }
//
//  void VisitAllAtomsList(AllAtomsListExpr& expr) override {
//    return_stack_.push(VisitReturnAllAtomsList(expr, *state_stack_.top()));
//  }
//  void VisitNeighborsList(NeighborsListExpr& expr) override {
//    return_stack_.push(VisitReturnNeighborsList(expr, *state_stack_.top()));
//  }
//  void VisitNumericalConstant(NumericalConstantExpr& expr) override {
//    return_stack_.push(VisitReturnNumericalConstant(expr,
//    *state_stack_.top()));
//  }
//  void VisitIdentifier(IdentifierExpr& expr) override {
//    return_stack_.push(VisitReturnIdentifier(expr, *state_stack_.top()));
//  }
//  void VisitSum(SumExpr& expr) override {
//    return_stack_.push(VisitReturnSum(expr, *state_stack_.top()));
//  }
//  void VisitLet(LetExpr& expr) override {
//    return_stack_.push(VisitReturnLet(expr, *state_stack_.top()));
//  }
//  void VisitImplicit(ImplicitExpr& expr) override {
//    return_stack_.push(VisitReturnImplicit(expr, *state_stack_.top()));
//  }
//  void VisitPiecewise(PiecewiseExpr& expr) override {
//    return_stack_.push(VisitReturnPiecewise(expr, *state_stack_.top()));
//  }
//  void VisitAdd(AddExpr& expr) override {
//    return_stack_.push(VisitReturnAdd(expr, *state_stack_.top()));
//  }
//  void VisitMul(MulExpr& expr) override {
//    return_stack_.push(VisitReturnMul(expr, *state_stack_.top()));
//  }
//  void VisitPow(PowExpr& expr) override {
//    return_stack_.push(VisitReturnPow(expr, *state_stack_.top()));
//  }
//};

#endif  // EXPR_BASE_H_
