#ifndef IR_BUILD_H_
#define IR_BUILD_H_ IR_BUILD_H_

#include <memory>
#include <string>
#include <vector>
#include "ir.h"

namespace potc {

namespace ir {

namespace build {

namespace stmt {

inline std::unique_ptr<IRStmt> Let(IRIdentifier id,
                                   const std::unique_ptr<IRExpr>& t,
                                   IRDataType type = kIRDataTypeDouble) {
  return std::make_unique<LetIRStmt>(id, t->Clone(), type);
}
// inline std::unique_ptr<IRStmt> Acc(IRIdentifier id,
//                                   const std::unique_ptr<IRExpr>& t) {
//  return std::make_unique<AccIRStmt>(id, t->Clone());
//}
// inline std::unique_ptr<IRExpr> EmptyExpr() { return {nullptr}; }
// inline std::unique_ptr<IRStmt> AccForceX(const std::unique_ptr<IRExpr>& i,
//                                         const std::unique_ptr<IRExpr>& t) {
//  return std::make_unique<AccForceIRStmt>(i->Clone(), t->Clone(), EmptyExpr(),
//                                          EmptyExpr());
//}
// inline std::unique_ptr<IRStmt> AccForceY(const std::unique_ptr<IRExpr>& i,
//                                         const std::unique_ptr<IRExpr>& t) {
//  return std::make_unique<AccForceIRStmt>(i->Clone(), EmptyExpr(), t->Clone(),
//                                          EmptyExpr());
//}
// inline std::unique_ptr<IRStmt> AccForceZ(const std::unique_ptr<IRExpr>& i,
//                                         const std::unique_ptr<IRExpr>& t) {
//  return std::make_unique<AccForceIRStmt>(i->Clone(), EmptyExpr(),
//  EmptyExpr(),
//                                          t->Clone());
//}

}  // namespace stmt

inline std::unique_ptr<IRExpr> Acos(const std::unique_ptr<IRExpr>& t) {
  return std::make_unique<FunCallIRExpr>("acos", t->Clone());
}
inline std::unique_ptr<IRExpr> Sin(const std::unique_ptr<IRExpr>& t) {
  return std::make_unique<FunCallIRExpr>("sin", t->Clone());
}
inline std::unique_ptr<IRExpr> Cos(const std::unique_ptr<IRExpr>& t) {
  return std::make_unique<FunCallIRExpr>("cos", t->Clone());
}
inline std::unique_ptr<IRExpr> Sqrt(const std::unique_ptr<IRExpr>& t) {
  return std::make_unique<FunCallIRExpr>("sqrt", t->Clone());
}
inline std::unique_ptr<IRExpr> Exp(const std::unique_ptr<IRExpr>& t) {
  return std::make_unique<FunCallIRExpr>("exp", t->Clone());
}
inline std::unique_ptr<IRExpr> Log(const std::unique_ptr<IRExpr>& t) {
  return std::make_unique<FunCallIRExpr>("log", t->Clone());
}
inline std::unique_ptr<IRExpr> Pow(const std::unique_ptr<IRExpr>& l,
                                   const std::unique_ptr<IRExpr>& r) {
  return std::make_unique<FunCallIRExpr>("pow", l->Clone(), r->Clone());
}
inline std::unique_ptr<IRExpr> BinOp(std::string op,
                                     const std::unique_ptr<IRExpr>& a,
                                     const std::unique_ptr<IRExpr>& b,
                                     IRDataType sub_type = kIRDataTypeDouble) {
  return std::make_unique<BinIRExpr>(op, a->Clone(), b->Clone(), sub_type);
}
inline std::unique_ptr<IRExpr> Mul(const std::unique_ptr<IRExpr>& a,
                                   const std::unique_ptr<IRExpr>& b) {
  return std::make_unique<BinIRExpr>("*", a->Clone(), b->Clone());
}
inline std::unique_ptr<IRExpr> Add(const std::unique_ptr<IRExpr>& a,
                                   const std::unique_ptr<IRExpr>& b) {
  return std::make_unique<BinIRExpr>("+", a->Clone(), b->Clone());
}
inline std::unique_ptr<IRExpr> Sub(const std::unique_ptr<IRExpr>& a,
                                   const std::unique_ptr<IRExpr>& b) {
  return std::make_unique<BinIRExpr>("-", a->Clone(), b->Clone());
}
inline std::unique_ptr<IRExpr> Div(const std::unique_ptr<IRExpr>& a,
                                   const std::unique_ptr<IRExpr>& b) {
  return std::make_unique<BinIRExpr>("/", a->Clone(), b->Clone());
}
inline std::unique_ptr<IRExpr> Neg(const std::unique_ptr<IRExpr>& a) {
  return std::make_unique<UnIRExpr>("-", a->Clone());
}
inline std::unique_ptr<IRExpr> Ref(IRIdentifier id) {
  return std::make_unique<RefIRExpr>(id);
}
inline std::unique_ptr<IRExpr> Lit(std::string val) {
  return std::make_unique<LitIRExpr>(val);
}
inline std::unique_ptr<IRExpr> AtomX(const std::unique_ptr<IRExpr>& t) {
  // return std::make_unique<AtomPosIRExpr>(t->Clone(), kAtomDimX);
  return std::make_unique<LookupIRExpr>(LookupIRExpr::kPosX, t->Clone());
}
inline std::unique_ptr<IRExpr> AtomY(const std::unique_ptr<IRExpr>& t) {
  return std::make_unique<LookupIRExpr>(LookupIRExpr::kPosY, t->Clone());
  // return std::make_unique<AtomPosIRExpr>(t->Clone(), kAtomDimY);
}
inline std::unique_ptr<IRExpr> AtomZ(const std::unique_ptr<IRExpr>& t) {
  return std::make_unique<LookupIRExpr>(LookupIRExpr::kPosZ, t->Clone());
  // return std::make_unique<AtomPosIRExpr>(t->Clone(), kAtomDimZ);
}
inline std::unique_ptr<IRExpr> Invalid(std::string hint) {
  return std::make_unique<InvalidIRExpr>(hint);
}
inline std::unique_ptr<IRExpr> Param(
    std::string name, std::vector<std::unique_ptr<IRExpr>> args) {
  return std::make_unique<LookupIRExpr>(LookupIRExpr::kParam, name,
                                        std::move(args));
}
inline std::unique_ptr<IRExpr> PerAtom(
    std::string name, std::vector<std::unique_ptr<IRExpr>> args) {
  return std::make_unique<LookupIRExpr>(LookupIRExpr::kPerAtom, name,
                                        std::move(args));
}

template <typename T>
struct IRBuildMixin {
  typedef const std::unique_ptr<IRExpr>& IRExprPtr;

 private:
  template <typename U>
  U* AddStmt_(std::unique_ptr<U> stmt) {
    U* ret = stmt.get();
    static_cast<T*>(this)->AddStmt(std::move(stmt));
    return ret;
  }
  std::unique_ptr<IRExpr> E() { return {nullptr}; }
  std::unique_ptr<IRExpr> C(IRExprPtr p) { return p->Clone(); }

 public:
  void Let(IRIdentifier id, IRExprPtr t, IRDataType type = kIRDataTypeDouble) {
    AddStmt_(std::make_unique<LetIRStmt>(id, C(t), type));
  }
  void Acc(IRIdentifier id, IRExprPtr t) {
    AddStmt_(std::make_unique<AccIRStmt>(id, C(t)));
  }
  void AccForceX(IRExprPtr i, IRExprPtr t) {
    AddStmt_(std::make_unique<AccForceIRStmt>(C(i), C(t), E(), E()));
  }
  void AccForceY(IRExprPtr& i, IRExprPtr& t) {
    AddStmt_(std::make_unique<AccForceIRStmt>(C(i), E(), C(t), E()));
  }
  void AccForceZ(IRExprPtr i, IRExprPtr t) {
    AddStmt_(std::make_unique<AccForceIRStmt>(C(i), E(), E(), C(t)));
  }
  void AccForceXYZ(IRExprPtr i, IRExprPtr x, IRExprPtr y, IRExprPtr z) {
    AddStmt_(std::make_unique<AccForceIRStmt>(C(i), C(x), C(y), C(z)));
  }
  void DeclAcc(IRIdentifier id) {
    AddStmt_(std::make_unique<DeclAccIRStmt>(id));
  }
  void DeclAssign(IRIdentifier id) {
    AddStmt_(std::make_unique<DeclAssignIRStmt>(id));
  }
  void InvalidStmt(const std::string& s) {
    AddStmt_(std::make_unique<InvalidIRStmt>(s));
  }
  void LetComplexFunCall(const std::string& name, IRIdentifier res,
                         std::vector<IRIdentifier> others,
                         std::vector<std::unique_ptr<IRExpr>> args) {
    AddStmt_(std::make_unique<LetComplexFunCallIRStmt>(name, res, others,
                                                       std::move(args)));
  }
  void PerAtomAdjointAcc(const std::string& name,
                         std::vector<std::unique_ptr<IRExpr>> args,
                         IRExprPtr val) {
    args.push_back(val->Clone());
    AddStmt_(std::make_unique<PerAtomActionIRStmt>(
        PerAtomActionIRStmt::kPerAtomAdjointAcc, name, std::move(args)));
  }
  void PerAtomAcc(const std::string& name,
                  std::vector<std::unique_ptr<IRExpr>> args, IRExprPtr val) {
    args.push_back(val->Clone());
    AddStmt_(std::make_unique<PerAtomActionIRStmt>(
        PerAtomActionIRStmt::kPerAtomAcc, name, std::move(args)));
  }
  void PerAtomComm(std::string name, bool need_forward = false) {
    AddStmt_(std::make_unique<PerAtomActionIRStmt>(
        PerAtomActionIRStmt::kPerAtomComm, name,
        std::vector<std::unique_ptr<IRExpr>>()));
  }
  void PerAtomAdjointComm(std::string name, bool need_reverse = false) {
    AddStmt_(std::make_unique<PerAtomActionIRStmt>(
        PerAtomActionIRStmt::kPerAtomAdjointComm, name,
        std::vector<std::unique_ptr<IRExpr>>()));
  }
  void PerAtomZero(const std::string& name,
                   std::vector<std::unique_ptr<IRExpr>> args) {
    AddStmt_(std::make_unique<PerAtomActionIRStmt>(
        PerAtomActionIRStmt::kPerAtomZero, name, std::move(args)));
  }
  void PerAtomAdjointZero(const std::string& name,
                          std::vector<std::unique_ptr<IRExpr>> args) {
    AddStmt_(std::make_unique<PerAtomActionIRStmt>(
        PerAtomActionIRStmt::kPerAtomAdjointZero, name, std::move(args)));
  }
};

template <typename T>
class IRScopeMixin {
 private:
  struct Scope_ {
    T data;
    CompoundIRStmt code;
  } root_scope_, *current_scope_;

 public:
  explicit IRScopeMixin(T* scope)
      : root_scope_{scope}, current_scope_(&root_scope_) {}
  void AddStmt(std::unique_ptr<IRStmt> stmt) {
    current_scope_->code.Add(std::move(stmt));
  }
  T* Scope() { return &current_scope_->data; }
  template <typename F, typename G>
  void WithNestedScope(F f, G g) {
    Scope_ new_scope{Scope()};
    auto old_scope = current_scope_;
    current_scope_ = &new_scope;
    f();
    current_scope_ = old_scope;
    G(new_scope.code);
  }
  template <typename F>
  void WithNestedScope(F f) {
    WithNestedScope(f, [&](auto code) { AddStmt(code); });
  }
  template <typename F, typename G>
  void WithFreshScope(F f, G g) {
    Scope_ new_scope;
    auto old_scope = current_scope_;
    current_scope_ = &new_scope;
    f();
    current_scope_ = old_scope;
    G(new_scope.code);
  }
  template <typename F>
  void WithFreshScope(F f) {
    WithFreshScope(f, [&](auto code) { AddStmt(code); });
  }
};

}  // namespace build

}  // namespace ir

}  // namespace potc

#endif  // IR_BUILD_H_
