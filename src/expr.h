#ifndef EXPR_H_
#define EXPR_H_

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <map>
#include <memory>
#include <stack>
#include <string>
#include <vector>
#include "expr_base.h"

class Decl;
class NamedDecl;

class NumericalConstantExpr : public Expr {
  std::string value_;

 public:
  void SetValueStr(const char* str, size_t len) {
    value_ = std::string(str, len);
  }
  std::string& GetValueStr() { return value_; }

  void Accept(ExprVisitor* vis) override { vis->Visit(this); }
};

class IdentifierExpr : public Expr {
  std::string name_;
  std::vector<std::unique_ptr<Expr>> arguments_;
  NamedDecl* decl_;
  SumExpr* sum_expr_;
  LetExpr* let_expr_;
  bool internal_;

 public:
  IdentifierExpr()
      : decl_(nullptr),
        sum_expr_(nullptr),
        let_expr_(nullptr),
        internal_(false) {}
  IdentifierExpr(const IdentifierExpr& old)
      : name_(old.name_),
        decl_(old.decl_),
        sum_expr_(old.sum_expr_),
        let_expr_(old.let_expr_),
        internal_(old.internal_) {}
  explicit IdentifierExpr(const std::string& name, bool internal = false)
      : name_(name),
        internal_(internal),
        decl_(nullptr),
        sum_expr_(nullptr),
        let_expr_(nullptr) {}
  void SetName(const char* str, size_t len) { name_ = std::string(str, len); }
  void SetName(std::string str) { name_ = str; }
  void SetInternal(bool internal) { internal_ = internal; }
  std::string& GetName() { return name_; }
  void AddArgument(std::unique_ptr<Expr> expr) {
    arguments_.push_back(std::move(expr));
  }
  std::vector<std::unique_ptr<Expr>>& GetArguments() { return arguments_; }
  bool GetInternal() { return internal_; }

  void Accept(ExprVisitor* vis) override { vis->Visit(this); }
};

class SumExpr : public Expr {
  std::unique_ptr<Expr> expr_;
  std::unique_ptr<Expr> iter_;
  std::string index_;

 public:
  SumExpr(const SumExpr& old) : index_(old.index_) {}
  SumExpr() {}
  void SetIndex(const char* str, size_t len) { index_ = std::string(str, len); }
  void SetIter(std::unique_ptr<Expr> lexp) { iter_ = std::move(lexp); }
  void SetExpr(std::unique_ptr<Expr> expr) { expr_ = std::move(expr); }
  Expr* GetExpr() { return expr_.get(); }
  Expr* GetIter() { return iter_.get(); }
  std::string& GetIndex() { return index_; }

  void Accept(ExprVisitor* vis) override { vis->Visit(this); }
};

class LetExpr : public Expr {
  std::unique_ptr<Expr> expr_;
  std::unique_ptr<Expr> value_;
  std::string name_;

 public:
  LetExpr(const LetExpr& old) : name_(old.name_) {}
  LetExpr() {}
  void SetNameStr(const char* str, size_t len) {
    name_ = std::string(str, len);
  }
  void SetValue(std::unique_ptr<Expr> expr) { value_ = std::move(expr); }
  void SetExpr(std::unique_ptr<Expr> expr) { expr_ = std::move(expr); }
  std::string& GetName() { return name_; }
  Expr* GetExpr() { return expr_.get(); }
  Expr* GetValue() { return value_.get(); }

  void Accept(ExprVisitor* vis) override { vis->Visit(this); }
};

class ImplicitExpr : public Expr {
  std::unique_ptr<Expr> expr_;
  std::map<std::string, std::string> implicits_;
  std::string last_implicit_;

 public:
  ImplicitExpr() {}
  ImplicitExpr(const ImplicitExpr& old) : implicits_(old.implicits_) {}
  void AddImplicitStr(const char* str, size_t len) {
    last_implicit_ = std::string(str, len);
  }
  void SetLastAliasStr(const char* str, size_t len) {
    implicits_[last_implicit_] = std::string(str, len);
  }
  void SetExpr(std::unique_ptr<Expr> expr) { expr_ = std::move(expr); }
  Expr* GetExpr() { return expr_.get(); }
  std::map<std::string, std::string>& GetImplicits() { return implicits_; }

  void Accept(ExprVisitor* vis) override { vis->Visit(this); }
};

class PiecewiseExpr : public Expr {
 public:
  struct Piece {
    std::unique_ptr<Expr> first, mid, last, expr;
    bool greater, first_equal, last_equal;
    void SetFirst(std::unique_ptr<Expr> ex) { first = std::move(ex); }
    void SetMid(std::unique_ptr<Expr> ex) { mid = std::move(ex); }
    void SetLast(std::unique_ptr<Expr> ex) { last = std::move(ex); }
    void SetExpr(std::unique_ptr<Expr> ex) { expr = std::move(ex); }
  };
  // lower_ < mid_ < upper
  std::vector<std::unique_ptr<Piece>> pieces_;

 public:
  PiecewiseExpr() {}
  void AddPiece(std::unique_ptr<Piece> p) { pieces_.push_back(std::move(p)); }
  std::vector<std::unique_ptr<Piece>>& GetPieces() { return pieces_; }

  void Accept(ExprVisitor* vis) override { vis->Visit(this); }
};

struct TypematchExpr : public Expr {
  struct Piece {
    std::vector<std::string> types;
    std::unique_ptr<Expr> expr;
    void AddType(const char* s, size_t l) {
      types.push_back(std::string(s, l));
    }
    void SetExpr(std::unique_ptr<Expr> ex) { expr = std::move(ex); }
  };
  // lower_ < mid_ < upper
  std::vector<std::unique_ptr<Piece>> pieces_;
  std::vector<std::string> args_;

  TypematchExpr() {}
  void AddPiece(std::unique_ptr<Piece> p) { pieces_.push_back(std::move(p)); }
  std::vector<std::unique_ptr<Piece>>& GetPieces() { return pieces_; }
  std::vector<std::string>& GetArgs() { return args_; }
  void AddArg(const char* s, size_t l) { args_.push_back(std::string(s, l)); }

  void Accept(ExprVisitor* vis) override { vis->Visit(this); }
};

class AddExpr : public Expr {
  std::vector<std::unique_ptr<Expr>> exprs_, exprs_neg_;

 public:
  void AddNeg(std::unique_ptr<Expr> expr) {
    exprs_neg_.push_back(std::move(expr));
  }
  void Add(std::unique_ptr<Expr> expr) { exprs_.push_back(std::move(expr)); }
  std::vector<std::unique_ptr<Expr>>& GetExprs() { return exprs_; }
  std::vector<std::unique_ptr<Expr>>& GetExprsNeg() { return exprs_neg_; }

  void Accept(ExprVisitor* vis) override { vis->Visit(this); }
};

class MulExpr : public Expr {
  std::vector<std::unique_ptr<Expr>> exprs_, exprs_inv_;

 public:
  void AddInv(std::unique_ptr<Expr> expr) {
    exprs_inv_.push_back(std::move(expr));
  }
  void Add(std::unique_ptr<Expr> expr) { exprs_.push_back(std::move(expr)); }
  std::vector<std::unique_ptr<Expr>>& GetExprs() { return exprs_; }
  std::vector<std::unique_ptr<Expr>>& GetExprsInv() { return exprs_inv_; }

  void Accept(ExprVisitor* vis) override { vis->Visit(this); }
};

class PowExpr : public Expr {
  std::unique_ptr<Expr> left_, right_;

 public:
  void SetLeft(std::unique_ptr<Expr> expr) { left_ = std::move(expr); }
  void SetRight(std::unique_ptr<Expr> expr) { right_ = std::move(expr); }
  Expr* GetLeft() { return left_.get(); }
  Expr* GetRight() { return right_.get(); }

  void Accept(ExprVisitor* vis) override { vis->Visit(this); }
};

class AllAtomsListExpr : public Expr {
 public:
  void Accept(ExprVisitor* vis) override { vis->Visit(this); }
};

struct NeighborsListExpr : public Expr {
  std::string index_;
  std::vector<std::string> exclusion_;
  std::unique_ptr<Expr> cutoff_;
  bool half_;
  std::string greater_;

 public:
  NeighborsListExpr(const NeighborsListExpr& old)
      : index_(old.index_),
        half_(old.half_),
        exclusion_(old.exclusion_),
        greater_(old.greater_) {}
  explicit NeighborsListExpr(bool half) : half_(half) {}
  void SetIndexStr(const char* str, size_t len) {
    index_ = std::string(str, len);
  }
  std::string GetIndex() { return index_; }
  void AddExclusionStr(const char* str, size_t len) {
    exclusion_.push_back(std::string(str, len));
  }
  std::vector<std::string>& GetExclusion() { return exclusion_; }
  void SetCutoff(std::unique_ptr<Expr> expr) { cutoff_ = std::move(expr); }
  Expr* GetCutoff() { return cutoff_.get(); }
  bool GetHalf() { return half_; }
  void SetGreaterStr(const char* str, size_t len) {
    assert(greater_.size() == 0);
    greater_ = std::string(str, len);
  }
  std::string GetGreater() { return greater_; }
  void Accept(ExprVisitor* vis) override { vis->Visit(this); }
};

class PrintExprVisitor : public ExprVisitor {
 public:
  void Visit(NumericalConstantExpr* expr) override { printf("num"); }
  void Visit(IdentifierExpr* expr) override { printf("id"); }
  void Visit(SumExpr* expr) override {
    printf("(sum ");
    expr->GetIter()->Accept(this);
    printf(" : ");
    expr->GetExpr()->Accept(this);
    printf(")");
  }
  void Visit(LetExpr* expr) override {
    printf("(let ");
    expr->GetValue()->Accept(this);
    printf(" : ");
    expr->GetExpr()->Accept(this);
    printf(")");
  }
  void Visit(ImplicitExpr* expr) override {
    printf("(implicit %lu ", expr->GetImplicits().size());
    expr->GetExpr()->Accept(this);
    printf(")");
  }
  void Visit(PiecewiseExpr* expr) override { printf("piecewise"); }
  void Visit(TypematchExpr* expr) override { printf("typematch"); }
  void Visit(AddExpr* expr) override {
    printf("(+");
    for (auto& ex : expr->GetExprs()) {
      printf(" ");
      ex->Accept(this);
    }
    printf(" -");
    for (auto& ex : expr->GetExprsNeg()) {
      printf(" ");
      ex->Accept(this);
    }
    printf(")");
  }
  void Visit(MulExpr* expr) override {
    printf("(*");
    for (auto& ex : expr->GetExprs()) {
      printf(" ");
      ex->Accept(this);
    }
    printf(" /");
    for (auto& ex : expr->GetExprsInv()) {
      printf(" ");
      ex->Accept(this);
    }
    printf(")");
  }
  void Visit(PowExpr* expr) override {
    if (expr->GetRight() == nullptr) {
      expr->GetLeft()->Accept(this);
    } else {
      printf("(^ ");
      expr->GetLeft()->Accept(this);
      printf(" ");
      expr->GetRight()->Accept(this);
      printf(")");
    }
  }
  void Visit(AllAtomsListExpr* expr) override { printf("all_atoms"); }
  void Visit(NeighborsListExpr* expr) override { printf("neighbors_half"); }
};

struct SimplifyExprVisitor : public ExprVisitor {
  std::unique_ptr<Expr> return_value_;
  template <typename T>
  void Return(T& val) {
    return_value_ = std::move(val);
  }
  template <typename T>
  void Return(T&& val) {
    return_value_ = std::move(val);
  }
  std::unique_ptr<Expr> Accept(Expr* expr) {
    expr->Accept(this);
    return std::move(return_value_);
  }
  std::unique_ptr<Expr> Accept(const std::unique_ptr<Expr>& expr) {
    expr->Accept(this);
    return std::move(return_value_);
  }
  void Visit(NumericalConstantExpr* expr) override {
    Return(std::make_unique<NumericalConstantExpr>(*expr));
  }
  void Visit(IdentifierExpr* expr) override {
    auto result = std::make_unique<IdentifierExpr>(*expr);
    for (auto& sub : expr->GetArguments()) {
      result->AddArgument(Accept(sub));
    }
    Return(result);
  }
  void Visit(SumExpr* expr) override {
    assert(expr->GetIndex().size() > 0);
    auto result = std::make_unique<SumExpr>(*expr);
    assert(result->GetIndex().size() > 0);
    result->SetExpr(Accept(expr->GetExpr()));
    result->SetIter(Accept(expr->GetIter()));
    Return(result);
  }
  void Visit(LetExpr* expr) override {
    auto result = std::make_unique<LetExpr>(*expr);
    result->SetExpr(Accept(expr->GetExpr()));
    result->SetValue(Accept(expr->GetValue()));
    Return(result);
  }
  void Visit(ImplicitExpr* expr) override {
    auto result = std::make_unique<ImplicitExpr>(*expr);
    result->SetExpr(Accept(expr->GetExpr()));
    Return(result);
  }
  void Visit(PiecewiseExpr* expr) override {
    auto result = std::make_unique<PiecewiseExpr>();
    for (auto& p : expr->GetPieces()) {
      auto new_p = std::make_unique<PiecewiseExpr::Piece>();
      new_p->greater = p->greater;
      new_p->first_equal = p->first_equal;
      new_p->last_equal = p->last_equal;
      new_p->SetFirst(Accept(p->first));
      new_p->SetMid(Accept(p->mid));
      if (p->last.get()) new_p->SetLast(Accept(p->last));
      new_p->SetExpr(Accept(p->expr));
      result->AddPiece(std::move(new_p));
    }
    Return(result);
  }
  void Visit(TypematchExpr* expr) override {
    auto result = std::make_unique<TypematchExpr>();
    result->args_ = expr->args_;
    for (auto& p : expr->GetPieces()) {
      auto new_p = std::make_unique<TypematchExpr::Piece>();
      new_p->types = p->types;
      new_p->expr = Accept(p->expr);
      result->AddPiece(std::move(new_p));
    }
    Return(result);
  }
  void Visit(AddExpr* expr) override {
    if (expr->GetExprs().size() == 1 && expr->GetExprsNeg().size() == 0) {
      Return(Accept(expr->GetExprs()[0]));
    } else {
      auto result = std::make_unique<AddExpr>();
      for (auto& ex : expr->GetExprs()) {
        result->Add(Accept(ex));
      }
      for (auto& ex : expr->GetExprsNeg()) {
        result->AddNeg(Accept(ex));
      }
      Return(result);
    }
  }
  void Visit(MulExpr* expr) override {
    if (expr->GetExprs().size() == 1 && expr->GetExprsInv().size() == 0) {
      Return(Accept(expr->GetExprs()[0]));
    } else {
      auto result = std::make_unique<MulExpr>();
      for (auto& ex : expr->GetExprs()) {
        result->Add(Accept(ex));
      }
      for (auto& ex : expr->GetExprsInv()) {
        result->AddInv(Accept(ex));
      }
      Return(result);
    }
  }
  void Visit(PowExpr* expr) override {
    if (expr->GetRight() == nullptr) {
      Return(Accept(expr->GetLeft()));
    } else {
      auto result = std::make_unique<PowExpr>();
      result->SetLeft(Accept(expr->GetLeft()));
      result->SetRight(Accept(expr->GetRight()));
      Return(result);
    }
  }
  void Visit(AllAtomsListExpr* expr) override {
    Return(std::make_unique<AllAtomsListExpr>(*expr));
  }
  void Visit(NeighborsListExpr* expr) override {
    auto result = std::make_unique<NeighborsListExpr>(*expr);
    result->SetCutoff(Accept(expr->GetCutoff()));
    Return(result);
  }
};

class Decl {
 public:
  enum Kind { kParameter, kFunction, kEnergy, kPerAtom };
  virtual ~Decl() = default;
  virtual Kind GetKind() = 0;
  virtual void Print() = 0;
};

enum class Type { kAtomType, kReal, kDistance, kAngle, kAtom };

class NamedDecl : public Decl {
 protected:
  std::string name_;
  std::vector<std::string> args_;
  std::vector<Type> arg_types_;
  int num_continuous_args_ = 0;
  int num_discrete_args_ = 0;

 public:
  void SetName(const char* str, size_t len) { name_ = std::string(str, len); }
  void SetName(std::string str) { name_ = str; }
  std::string GetName() { return name_; }
  const char* GetNameC() { return name_.c_str(); }
  void AddArgument(const char* str, size_t len) {
    args_.push_back(std::string(str, len));
  }
  void AddArgumentType(Type t) {
    arg_types_.push_back(t);
    if (t == Type::kDistance || t == Type::kReal) {
      num_continuous_args_ += 1;
    } else if (t == Type::kAtomType) {
      num_discrete_args_ += 1;
    }
  }
  void SetLastArgType(std::unique_ptr<Type> t) {
    assert(t.get());
    arg_types_.push_back(*t);
    assert(arg_types_.size() == args_.size());
  }
  std::vector<std::string>& GetArguments() { return args_; }
  std::vector<Type>& GetArgumentTypes() { return arg_types_; }
  int GetNumContinuousArgs() { return num_continuous_args_; }
  int GetNumDiscreteArgs() { return num_discrete_args_; }
  Type GetArgumentType(int pos) { return arg_types_[pos]; }
  int GetNumArgs() { return args_.size(); }
};

struct ParameterSpec {
  enum Kind { kFile, kSpline, kCoeff, kSetting, kTabulated };
  virtual ~ParameterSpec() {}
  virtual Kind GetKind() = 0;
};

struct FileParameterSpec : public ParameterSpec {
  int index;
  Kind GetKind() override { return ParameterSpec::kFile; }
  void SetFileIndex(const char* c, size_t len) {
    index = atoi(std::string(c, len).c_str());
    assert(index && "Error parsing file index in parameter decl");
  }
};

struct SplineParameterSpec : public ParameterSpec {
  int order;
  std::string name;
  std::vector<int> nodes_along;
  std::vector<std::vector<int>> derivatives;
  std::unique_ptr<Expr> low, step;
  bool is_grid = false;
  bool is_integer = false;
  void SetName(const char* c, size_t len) { name = std::string(c, len); }
  void SetOrder(const char* c, size_t len) {
    order = atoi(std::string(c, len).c_str());
  }
  void SetLow(std::unique_ptr<Expr> expr) { low = std::move(expr); }
  void SetStep(std::unique_ptr<Expr> expr) { step = std::move(expr); }
  void AddNodesAlong(const char* c, size_t len) {
    nodes_along.push_back(atoi(std::string(c, len).c_str()));
  }
  void AddDerivTerm(const char* c, size_t len) { derivatives.push_back({}); }
  void AddDerivElem(const char* c, size_t len) {
    derivatives.back().push_back(atoi(std::string(c, len).c_str()));
  }
  Kind GetKind() override { return ParameterSpec::kSpline; }
};

struct TabulatedParameterSpec : public ParameterSpec {
  enum Interpolation { kLinear, kClosest };
  Kind GetKind() override { return ParameterSpec::kTabulated; }
};

struct CoeffParameterSpec : public ParameterSpec {
  int index;
  Kind GetKind() override { return ParameterSpec::kCoeff; }
};

struct SettingParameterSpec : public ParameterSpec {
  int index;
  Kind GetKind() override { return ParameterSpec::kSetting; }
};

class ParameterDecl : public NamedDecl {
 public:
  std::unique_ptr<ParameterSpec> spec;

  void SetParameterSpec(std::unique_ptr<ParameterSpec> the_spec) {
    spec = std::move(the_spec);
  }
  Kind GetKind() override { return Decl::kParameter; }
  void Print() override { printf("param %s\n", name_.c_str()); }
};

class FunctionDecl : public NamedDecl {
 public:
  // enum Type { kAtom, kAtomType, kDistance, kAngle, kReal };

 private:
  // std::vector<Type> arg_types_;
  std::unique_ptr<Expr> expr_;

 public:
  Kind GetKind() override { return Decl::kFunction; }
  void Print() override {
    printf("function %s: ", name_.c_str());
    PrintExprVisitor printer;
    if (expr_.get() != nullptr) expr_->Accept(&printer);
    printf("\n");
  }
  void SetExpr(std::unique_ptr<Expr> expr) {
    SimplifyExprVisitor simpl;
    expr_ = simpl.Accept(expr);
  }
  Expr* GetExpr() { return expr_.get(); }
};

struct PerAtomDecl : public FunctionDecl {
  Kind GetKind() override { return Decl::kPerAtom; }
};

class EnergyDecl : public Decl {
  std::unique_ptr<Expr> expr_;

 public:
  void SetExpr(std::unique_ptr<Expr> expr) {
    SimplifyExprVisitor simpl;
    expr_ = simpl.Accept(expr);
  }
  Expr& GetExpr() { return *expr_; }
  Kind GetKind() { return Decl::kEnergy; }
  void Print() {
    printf("energy ");
    PrintExprVisitor printer;
    if (expr_.get() != nullptr) expr_->Accept(&printer);
    printf("\n");
  }
};

class PotentialDef {
  std::map<std::string, std::unique_ptr<NamedDecl>> decls_;
  std::vector<NamedDecl*> named_decls_;
  std::vector<ParameterDecl*> parameter_decls_;
  std::vector<std::unique_ptr<EnergyDecl>> energy_decls_;

 public:
  std::vector<std::string> types;

  void Add(std::unique_ptr<Decl> decl) {
    if (decl->GetKind() == Decl::kEnergy) {
      energy_decls_.emplace_back(static_cast<EnergyDecl*>(decl.release()));
    } else {
      auto named_decl =
          std::unique_ptr<NamedDecl>(static_cast<NamedDecl*>(decl.release()));
      std::string key = named_decl->GetName();
      if (decls_.count(key) != 0) {
        printf("Warning: %s is already a parameter. Overwriting...\n",
               key.c_str());
        // remove from named_decls and parameter_decls
        named_decls_.erase(std::find(named_decls_.begin(), named_decls_.end(),
                                     decls_[key].get()));
        if (named_decl->GetKind() == Decl::kParameter) {
          parameter_decls_.erase(std::find(parameter_decls_.begin(),
                                           parameter_decls_.end(),
                                           decls_[key].get()));
        }
      }
      named_decls_.push_back(named_decl.get());
      if (named_decl->GetKind() == Decl::kParameter) {
        parameter_decls_.push_back(
            static_cast<ParameterDecl*>(named_decl.get()));
      }
      decls_[key] = std::move(named_decl);
    }
  }
  std::vector<std::unique_ptr<EnergyDecl>>& GetEnergyDecls() {
    return energy_decls_;
  }
  std::vector<NamedDecl*>& GetDecls() { return named_decls_; }
  std::vector<ParameterDecl*>& GetParameterDecls() { return parameter_decls_; }
  bool IsValid() { return !energy_decls_.empty(); }
  void Print() {
    for (const auto& decl : decls_) {
      decl.second->Print();
    }
    for (const auto& decl : energy_decls_) {
      decl->Print();
    }
  }
  NamedDecl* Lookup(const std::string& s, bool& found) {
    auto it = decls_.find(s);
    found = it != decls_.end();
    if (found) return it->second.get();
    return nullptr;
  }
};

#endif  // EXPR_H_
