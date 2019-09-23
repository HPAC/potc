#include "der.h"

#include <map>
#include <set>
#include <string>
#include <vector>
#include "expr.h"

#include "ir.h"
#include "ir_build.h"

using namespace potc::ir::build;

namespace potc {

namespace der {

namespace {

struct CodeEmitterState {
  CodeEmitterState* parent;
  std::unique_ptr<IRExpr> adjoint;
  std::string iterator_name;
  CompoundIRStmt code;
  std::map<std::string, std::unique_ptr<IRExpr>> lookup;
  std::map<std::string, std::unique_ptr<IRExpr>> lookup_index;
  std::map<std::string, std::string> implicits;
  std::map<std::string, IRIdentifier> adjoint_lookup;

  // Find the identifier called `s` in the surrounding scope
  std::unique_ptr<IRExpr> Lookup(const std::string& s, bool& found) {
    auto it = lookup.find(s);
    found = it != lookup.end();
    if (found) return it->second->Clone();
    if (parent != nullptr) return parent->Lookup(s, found);
    return std::make_unique<InvalidIRExpr>(
        "CodeEmitterState::Lookup - not found: " + s);
  }

  // Find the adjoint for the identifier `s` in the surrounding scope
  IRIdentifier LookupAdjoint(const std::string& s, bool& found) {
    auto it = adjoint_lookup.find(s);
    found = it != adjoint_lookup.end();
    if (found) return it->second;
    if (parent != nullptr) return parent->LookupAdjoint(s, found);
    return IRIdentifier::Invalid(
        "CodeEmitterState::LookupAdjoint - not found: " + s);
  }

  // Find the `loop index` corresponding to a loop (as opposed to its actual
  // value)
  // These two are different when iterating through a neighbor list
  std::unique_ptr<IRExpr> LookupIndex(const std::string& s, bool& found) {
    auto it = lookup_index.find(s);
    found = it != lookup_index.end();
    if (found) return it->second->Clone();
    if (parent != nullptr) return parent->LookupIndex(s, found);
    return std::make_unique<InvalidIRExpr>(
        "CodeEmitterState::LookupIndex - not found: " + s);
  }

  // Find the name that has been set to stand in for `s` implicitly
  std::string LookupImplicitRaw(const std::string& s, bool& found,
                                CodeEmitterState*& at) {
    auto it = implicits.find(s);
    found = it != implicits.end();
    at = this;
    if (found) return it->second;
    if (parent != nullptr) return parent->LookupImplicitRaw(s, found, at);
    return "__NOT_FOUND_IMPLICIT_ERROR_RAW__" + s;
  }

  // Find the expr that stands for `s` implicitly
  std::unique_ptr<IRExpr> LookupImplicit(const std::string& s, bool& found) {
    CodeEmitterState* at;
    std::string result = LookupImplicitRaw(s, found, at);
    if (found) return at->Lookup(result, found);
    return std::make_unique<InvalidIRExpr>(
        "CodeEmitterState::LookupImplicit - not found: " + s);
  }
};

struct CodeEmitterExprVisitor
    : public ExprVisitor,
      public potc::ir::build::IRBuildMixin<CodeEmitterExprVisitor> {
  IRIdentifierContext idc;
  std::unique_ptr<IRExpr> return_value_;
  CodeEmitterState* state;
  PotentialDef* def;
  std::map<IRIdentifier, int> loop_id;
  std::map<IRIdentifier, bool> is_potentially_nonlocal_neighbor;
  std::set<std::string> peratom_need_both_directions;
  enum Mode { kForward, kReverse, kBoth } mode;
  CodeEmitterState root;
  bool peratom_mode = false;
  std::string peratom_name;

  void AddStmt(std::unique_ptr<IRStmt> stmt) {
    state->code.Add(std::move(stmt));
  }

  template <typename F>
  using enable_if_void =
      typename std::enable_if<std::is_void<F>::value, F>::type;

  template <typename F>
  using enable_if_not_void =
      typename std::enable_if<!std::is_void<F>::value, F>::type;

  template <typename F>
  auto WithState(CodeEmitterState* new_state, F f)
      -> enable_if_not_void<decltype(f())> {
    auto old_state = state;
    state = new_state;
    auto ret = f();
    state = old_state;
    return ret;
  }

  template <typename F>
  auto WithState(CodeEmitterState* new_state, F f)
      -> enable_if_void<decltype(f())> {
    auto old_state = state;
    state = new_state;
    f();
    state = old_state;
  }

  template <typename F>
  auto WithMode(Mode new_mode, F f) -> enable_if_not_void<decltype(f())> {
    Mode old_mode = mode;
    mode = new_mode;
    auto ret = f();
    mode = old_mode;
    return ret;
  }

  template <typename F>
  auto WithMode(Mode new_mode, F f) -> enable_if_void<decltype(f())> {
    Mode old_mode = mode;
    mode = new_mode;
    f();
    mode = old_mode;
  }

  void LoopVarsBegin(const std::unique_ptr<IRExpr>& iter_val) {
    auto ref_expr = iter_val->asa<RefIRExpr>();
    assert(ref_expr);
    idc.Advance();
    loop_id[ref_expr->ref] = idc.next_identifier;
    Let(idc.Get("px"), AtomX(iter_val));
    Let(idc.Get("py"), AtomY(iter_val));
    Let(idc.Get("pz"), AtomZ(iter_val));
    Let(idc.Get("ty"),
        std::make_unique<LookupIRExpr>(LookupIRExpr::kType, iter_val->Clone()),
        kIRDataTypeInt);
  }
  void LoopVarsStart(const std::unique_ptr<IRExpr>& iter_val) {
    auto ref_expr = iter_val->asa<RefIRExpr>();
    assert(ref_expr);
    int id = loop_id[ref_expr->ref];
    if (mode == kReverse || mode == kBoth) {
      DeclAcc(IRIdentifier(id, "fx"));
      DeclAcc(IRIdentifier(id, "fy"));
      DeclAcc(IRIdentifier(id, "fz"));
    }
  }
  void LoopVarsEnd(const std::unique_ptr<IRExpr>& iter_val) {
    auto ref_expr = iter_val->asa<RefIRExpr>();
    assert(ref_expr);
    int id = loop_id[ref_expr->ref];
    if (mode == kReverse || mode == kBoth) {
      AccForceXYZ(iter_val->Clone(), Ref(IRIdentifier(id, "fx")),
                  Ref(IRIdentifier(id, "fy")), Ref(IRIdentifier(id, "fz")));
    }
  }

  std::unique_ptr<IRExpr> LoopAtomX(const std::unique_ptr<IRExpr>& i) {
    auto ref_expr = i->asa<RefIRExpr>();
    if (ref_expr) {
      if (loop_id.count(ref_expr->ref)) {
        int id = loop_id[ref_expr->ref];
        return Ref(IRIdentifier(id, "px"));
      }
    }
    return AtomX(i);
  }

  std::unique_ptr<IRExpr> LoopAtomY(const std::unique_ptr<IRExpr>& i) {
    auto ref_expr = i->asa<RefIRExpr>();
    if (ref_expr) {
      if (loop_id.count(ref_expr->ref)) {
        int id = loop_id[ref_expr->ref];
        return Ref(IRIdentifier(id, "py"));
      }
    }
    return AtomY(i);
  }

  std::unique_ptr<IRExpr> LoopAtomZ(const std::unique_ptr<IRExpr>& i) {
    auto ref_expr = i->asa<RefIRExpr>();
    if (ref_expr) {
      if (loop_id.count(ref_expr->ref)) {
        int id = loop_id[ref_expr->ref];
        return Ref(IRIdentifier(id, "pz"));
      }
    }
    return AtomZ(i);
  }

  std::unique_ptr<IRExpr> LoopAtomType(const std::unique_ptr<IRExpr>& i) {
    auto ref_expr = i->asa<RefIRExpr>();
    if (ref_expr) {
      if (loop_id.count(ref_expr->ref)) {
        int id = loop_id[ref_expr->ref];
        return Ref(IRIdentifier(id, "ty"));
      }
    }
    return std::make_unique<LookupIRExpr>(LookupIRExpr::kType, i->Clone());
  }

  void LoopAccForceX(const std::unique_ptr<IRExpr>& i,
                     const std::unique_ptr<IRExpr>& t) {
    auto ref_expr = i->asa<RefIRExpr>();
    if (ref_expr) {
      if (loop_id.count(ref_expr->ref)) {
        int id = loop_id[ref_expr->ref];
        Acc(IRIdentifier(id, "fx"), t);
        return;
      }
    }
    AccForceX(i, t);
  }

  void LoopAccForceY(const std::unique_ptr<IRExpr>& i,
                     const std::unique_ptr<IRExpr>& t) {
    auto ref_expr = i->asa<RefIRExpr>();
    if (ref_expr) {
      if (loop_id.count(ref_expr->ref)) {
        int id = loop_id[ref_expr->ref];
        Acc(IRIdentifier(id, "fy"), t);
        return;
      }
    }
    AccForceY(i, t);
  }

  void LoopAccForceZ(const std::unique_ptr<IRExpr>& i,
                     const std::unique_ptr<IRExpr>& t) {
    auto ref_expr = i->asa<RefIRExpr>();
    if (ref_expr) {
      if (loop_id.count(ref_expr->ref)) {
        int id = loop_id[ref_expr->ref];
        Acc(IRIdentifier(id, "fz"), t);
        return;
      }
    }
    AccForceZ(i, t);
  }

 public:
  explicit CodeEmitterExprVisitor(PotentialDef* c_def, Mode c_mode,
                                  std::unique_ptr<IRExpr> adj)
      : def(c_def), mode(c_mode), root{nullptr, adj->Clone()}, state(&root) {}
  IRIdentifier NextId(std::string hint) { return idc.Next(hint); }

  template <typename T>
  void Return(T& val) {
    return_value_ = std::move(val);
  }

  template <typename T>
  void Return(T&& val) {
    return_value_ = std::move(val);
  }

  std::unique_ptr<IRExpr> Accept(Expr* expr) {
    expr->Accept(this);
    return std::move(return_value_);
  }

  template <typename T>
  std::unique_ptr<IRExpr> Accept(const std::unique_ptr<T>& expr) {
    return Accept(expr.get());
  }

  std::unique_ptr<IRExpr> AcceptForward(Expr* expr) {
    WithMode(kForward, [&]() { expr->Accept(this); });
    return std::move(return_value_);
  }

  template <typename T>
  std::unique_ptr<IRExpr> AcceptForward(const std::unique_ptr<T>& expr) {
    return AcceptForward(expr.get());
  }

  void AcceptReverse(Expr* expr, std::unique_ptr<IRExpr> adjoint) {
    WithMode(kReverse, [&]() {
      auto old_adjoint = std::move(state->adjoint);
      state->adjoint = std::move(adjoint);
      expr->Accept(this);
      state->adjoint = std::move(old_adjoint);
    });
  }

  template <typename T>
  void AcceptReverse(const std::unique_ptr<T>& expr,
                     std::unique_ptr<IRExpr> adjoint) {
    AcceptReverse(expr.get(), std::move(adjoint));
  }

  void Visit(NumericalConstantExpr* expr) override {
    if (mode == kForward || mode == kBoth) {
      return Return(Lit(expr->GetValueStr()));
    } else {
      return Return(
          std::make_unique<InvalidIRExpr>("__REVERSE_NO_VALUE_ERROR__"));
    }
  }

  /*
   * decl_args: arg names as declared
   * expr_args: the args actually passed to the function
   * returns: the IRExprs for all the args
   * Does not switch to another forward/reverse mode itself
   * ONLY evaluates stuff in forward mode!
   */
  std::vector<std::unique_ptr<IRExpr>> LookupWithImplicits(
      std::vector<std::string>& decl_args,
      std::vector<std::unique_ptr<Expr>>& expr_args) {
    int imply = decl_args.size() - expr_args.size();
    assert(imply >= 0);
    std::vector<std::unique_ptr<IRExpr>> args;
    for (auto& param : decl_args) {
      bool found;
      if (imply == 0) break;
      args.push_back(state->LookupImplicit(param, found));
      // assert(found);
      imply--;
    }
    assert(imply == 0);
    for (auto& param : expr_args) {
      args.push_back(AcceptForward(param));
    }
    return args;
  }
  template <typename T>
  std::unique_ptr<IRExpr> HandleBuiltinFunction(IdentifierExpr* expr,
                                                std::string fn, T dfn) {
    assert(expr->GetArguments().size() == 1);
    assert(expr->GetName().size() > 0);
    auto a = AcceptForward(expr->GetArguments()[0]);
    auto val = NextId("bultin");
    Let(val, std::make_unique<FunCallIRExpr>(fn, a->Clone()));
    if (mode == kForward) return Ref(val);
    auto old_adj = state->adjoint->Clone();
    auto adj = NextId("builtin_adj");
    auto dval = dfn(a->Clone());
    Let(adj, Mul(old_adj, dval));
    state->adjoint = Ref(adj);
    WithMode(kReverse, [&]() { Accept(expr->GetArguments()[0]); });
    state->adjoint = old_adj->Clone();
    if (mode == kBoth)
      return Ref(val);
    else
      return Invalid("__REVERSE_NO_VALUE_ERROR__");
  }
  std::unique_ptr<IRExpr> HandleDistance(std::unique_ptr<IRExpr>& i,
                                         std::unique_ptr<IRExpr>& j, int& id) {
    idc.Advance();
    id = idc.next_identifier;
    auto dx = idc.Get("dx");
    auto dy = idc.Get("dy");
    auto dz = idc.Get("dz");
    Let(dx, Sub(LoopAtomX(i), LoopAtomX(j)));
    Let(dy, Sub(LoopAtomY(i), LoopAtomY(j)));
    Let(dz, Sub(LoopAtomZ(i), LoopAtomZ(j)));
    auto rsq = IRIdentifier(id, "rsq");
    Let(rsq, Add(Add(Mul(Ref(dx), Ref(dx)), Mul(Ref(dy), Ref(dy))),
                 Mul(Ref(dz), Ref(dz))));
    auto r = IRIdentifier(id, "r");
    Let(r, Sqrt(Ref(rsq)));
    if (mode == kForward) return Ref(r);
    auto adj = IRIdentifier(id, "adj_by_r");
    auto recip = IRIdentifier(id, "recip_r");
    Let(recip, Div(Lit("1"), Ref(r)));
    Let(adj, Mul(state->adjoint, Ref(recip)));
    LoopAccForceX(i, Neg(Mul(Ref(adj), Ref(dx))));
    LoopAccForceY(i, Neg(Mul(Ref(adj), Ref(dy))));
    LoopAccForceZ(i, Neg(Mul(Ref(adj), Ref(dz))));
    LoopAccForceX(j, Mul(Ref(adj), Ref(dx)));
    LoopAccForceY(j, Mul(Ref(adj), Ref(dy)));
    LoopAccForceZ(j, Mul(Ref(adj), Ref(dz)));
    if (mode == kBoth)
      return Ref(r);
    else
      return Invalid("__REVERSE_NO_VALUE_ERROR__");
  }
  void HandleFunctionCall(IdentifierExpr* expr, FunctionDecl* decl) {
    std::vector<std::unique_ptr<IRExpr>> args;
    auto& decl_args = decl->GetArguments();
    auto& expr_args = expr->GetArguments();
    const int imply = decl_args.size() - expr_args.size();
    CodeEmitterState nested{nullptr, state->adjoint->Clone()};
    assert(imply >= 0);
    int idx = 0;
    for (auto& param : decl_args) {
      bool found;
      if (idx == imply) break;
      CodeEmitterState* at;
      std::string name = state->LookupImplicitRaw(param, found, at);
      if (!found) {
        printf("Not found in %s: %s\n", decl->GetName().c_str(), param.c_str());
        assert(found);
      }
      nested.lookup[param] = state->Lookup(name, found);
      assert(found);
      if (mode != kForward) {
        auto acc = at->LookupAdjoint(name, found);
        nested.adjoint_lookup[param] = acc;
      }
      idx += 1;
    }
    assert(decl->GetArguments().size() == decl->GetArgumentTypes().size());
    bool is_pure = true;
    for (auto t : decl->GetArgumentTypes()) {
      if (t == Type::kAtom) {
        is_pure = false;
      }
    }
    is_pure = false;
    assert(imply == idx);
    for (auto& param : expr_args) {
      auto& name = decl_args[idx];
      nested.lookup[name] = AcceptForward(param);
      if (mode != kForward) {
        auto acc = NextId("fun_acc_adj");
        DeclAcc(acc);
        nested.adjoint_lookup[name] = acc;
      }
      idx += 1;
    }
    std::unique_ptr<IRExpr> old_adj = state->adjoint->Clone();
    auto result = WithState(&nested, [&]() {
      if (is_pure) state->adjoint = Lit("1");
      return Accept(static_cast<FunctionDecl*>(decl)->GetExpr());
    });
    state->code.AddAndConsume(nested.code);
    if (mode == kForward) return Return(result);
    WithMode(kReverse, [&]() {
      idx = imply;
      for (auto& param : expr_args) {
        auto& name = decl_args[idx];
        auto adj = NextId("fun_adj");
        assert(nested.adjoint_lookup.count(name));
        // Let(adj, Mul(old_adj, Ref(nested.adjoint_lookup[name]))));
        // the latter is correct since the adjoint multiplication is
        // already incorporated
        if (is_pure)
          Let(adj, Mul(old_adj, Ref(nested.adjoint_lookup[name])));
        else
          Let(adj, Ref(nested.adjoint_lookup[name]));
        state->adjoint = Ref(adj);
        Accept(param);
        idx += 1;
      }
    });
    state->adjoint = std::move(old_adj);
    if (mode == kBoth)
      return Return(result);
    else
      return Return(Invalid("__REVERSE_NO_VALUE_ERROR__"));
  }
  void HandleSplineCall(IdentifierExpr* expr, ParameterDecl* decl,
                        SplineParameterSpec* spec) {
    auto& decl_args = decl->GetArguments();
    auto& expr_args = expr->GetArguments();
    const int imply = decl_args.size() - expr_args.size();
    std::vector<IRIdentifier> implicit_adjoints;
    std::vector<std::unique_ptr<IRExpr>> args;
    assert(imply >= 0);
    int idx = 0;
    for (auto& param : decl_args) {
      bool found;
      if (idx == imply) break;
      CodeEmitterState* at;
      std::string name = state->LookupImplicitRaw(param, found, at);
      if (!found) {
        printf("Not found in %s: %s\n", decl->GetName().c_str(), param.c_str());
        assert(found);
      }
      args.push_back(state->Lookup(name, found));
      assert(found);
      if (mode != kForward) {
        auto acc = at->LookupAdjoint(name, found);
        implicit_adjoints.push_back(acc);
      }
      idx += 1;
    }
    assert(imply == idx);
    WithMode(kForward, [&]() {
      for (auto& param : expr_args) {
        args.push_back(Accept(param));
        idx += 1;
      }
      for (int i = 0; i < decl_args.size(); i++) {
        if (decl->GetArgumentType(i) != Type::kAtomType) continue;
        args[i] = LoopAtomType(args[i]);
      }
    });
    auto eval_fn_name = "spline_eval_" + decl->GetName();
    IRIdentifier name = NextId("spv");
    std::vector<IRIdentifier> other_names;
    int num_args = decl->GetArguments().size();
    for (int i = 0; i < num_args; i++) {
      if (decl->GetArgumentType(i) == Type::kAtomType) continue;
      other_names.push_back(NextId("spd"));
    }
    LetComplexFunCall(eval_fn_name, name, other_names, std::move(args));
    if (mode == kForward) return Return(Ref(name));
    auto old_adj = state->adjoint->Clone();
    WithMode(kReverse, [&]() {
      idx = 0;
      int arg_idx = 0;
      for (auto& impl : implicit_adjoints) {
        if (decl->GetArgumentType(arg_idx++) == Type::kAtomType) continue;
        Acc(impl, Mul(old_adj, Ref(other_names[idx])));
        idx += 1;
      }
      for (auto& arg : expr_args) {
        if (decl->GetArgumentType(arg_idx++) == Type::kAtomType) continue;
        state->adjoint = Mul(old_adj, Ref(other_names[idx]));
        Accept(arg);
        idx += 1;
      }
    });
    state->adjoint = std::move(old_adj);
    if (mode == kBoth)
      return Return(Ref(name));
    else
      return Return(Invalid("__REVERSE_NO_VALUE_ERROR__"));
  }
  // Lookup can return: A symbol defined in an outer scope, a symbol in the
  // defintion (function/parameter), an internal reference.
  // Ideally, this would be bound already in a previous pass.
  void Visit(IdentifierExpr* expr) override {
    bool found;
    auto val = state->Lookup(expr->GetName(), found);
    if (found) {
      // This case is tricky! Solve in conjunction with Visitreturn ReturnLet.
      if (mode == kReverse || mode == kBoth) {
        auto acc = state->LookupAdjoint(expr->GetName(), found);
        if (found) {
          Acc(acc, state->adjoint);
        }
      }
      if (mode == kForward || mode == kBoth)
        return Return(val);
      else
        return Return(
            std::make_unique<InvalidIRExpr>("__REVERSE_NO_VALUE_ERROR__"));
    }
    NamedDecl* decl = def->Lookup(expr->GetName(), found);
    if (found) {
      if (decl->GetKind() == Decl::kParameter) {
        ParameterDecl* param_decl = static_cast<ParameterDecl*>(decl);
        if (param_decl->spec->GetKind() == ParameterSpec::kSpline) {
          return HandleSplineCall(
              expr, param_decl,
              static_cast<SplineParameterSpec*>(param_decl->spec.get()));
        } else if (param_decl->spec->GetKind() == ParameterSpec::kTabulated) {
          return Return(Invalid("__TABULATION_NOT_IMPLEMENTED__"));
        } else {
          if (mode == kForward || mode == kBoth) {
            auto args =
                LookupWithImplicits(decl->GetArguments(), expr->GetArguments());
            for (auto& arg : args) {
              arg = LoopAtomType(arg);
            }
            // If parameter: Convert atom id types to atom type types.
            // Need a concept of types to pull this off.
            auto v = NextId("param");
            Let(v, Param(decl->GetName(), std::move(args)));
            return Return(Ref(v));
          } else {
            return Return(Invalid("__REVERSE_NO_VALUE_ERROR__"));
          }
        }
      } else if (decl->GetKind() == Decl::kFunction) {
        return HandleFunctionCall(expr, static_cast<FunctionDecl*>(decl));
      } else if (decl->GetKind() == Decl::kPerAtom) {
        // return HandleFunctionCall(expr, static_cast<FunctionDecl*>(decl));
        // TODO(markus) do we want an explicit peratom kind?
        // if (mode != kForward) {
        //  // collect args (potentially atom types).
        //  Acc(PerAtomAdjoint(decl->GetName(), args),
        //  state->adjoint));
        //}
        // if (mode == kReverse) {
        //  return Return(Invalid("__REVERSE_NO_VALUE_ERROR__"));
        //}
        // return Return(std::make_unique<LookupIRExpr>(LookupIRExpr::kPerAtom,
        // decl->GetName(), std::vector<std::unique_ptr<IRExpr>>()));
        //
        // A peratom may depend on one atom id (and potentially multiple atom
        // types?)
        // As such, at this stage we only need to insert accordingly.
        // Some other component will make sure that the data is later used
        // properly.
        // As such we can use lookupWithImplicits.
        //
        //
        //
        auto args =
            LookupWithImplicits(decl->GetArguments(), expr->GetArguments());
        assert(args.size() == 1);
        bool is_nonlocal =
            is_potentially_nonlocal_neighbor[args[0]->asa<RefIRExpr>()->ref];
        if (is_nonlocal) {
          assert(decl->GetName() != peratom_name);
          peratom_need_both_directions.insert(decl->GetName());
        }
        if (mode == kReverse || mode == kBoth) {
          decltype(args) args_copy;
          for (auto& arg : args) args_copy.push_back(arg->Clone());
          PerAtomAdjointAcc(decl->GetName(), std::move(args_copy),
                            state->adjoint);
        }
        if (mode == kForward || mode == kBoth) {
          auto v = NextId("peratom");
          Let(v, PerAtom(decl->GetName(), std::move(args)));
          return Return(Ref(v));
        } else {
          return Return(Invalid("__REVERSE_NO_VALUE_ERROR__"));
        }
      } else {
        assert(0);
      }
    } else if (expr->GetName() == "r") {
      std::vector<std::string> decl_args;
      decl_args.push_back("i");
      decl_args.push_back("j");
      auto args = LookupWithImplicits(decl_args, expr->GetArguments());
      int id;
      return Return(HandleDistance(args[0], args[1], id));
    } else if (expr->GetName() == "theta") {
      std::vector<std::string> decl_args;
      decl_args.push_back("j");
      decl_args.push_back("i");
      decl_args.push_back("k");
      auto args = LookupWithImplicits(decl_args, expr->GetArguments());
      int id_a, id_b, id_c;
      auto ra = WithMode(
          kForward, [&]() { return HandleDistance(args[1], args[0], id_a); });
      auto rb = WithMode(
          kForward, [&]() { return HandleDistance(args[1], args[2], id_b); });
      auto rc = WithMode(
          kForward, [&]() { return HandleDistance(args[0], args[2], id_c); });
      // int id_num = next_identifier_++;
      idc.Advance();
      // std::string id = std::to_string();
      auto dot = idc.Get("dot");
      Let(dot, Add(Add(Mul(Ref(IRIdentifier(id_a, "dx")),
                           Ref(IRIdentifier(id_b, "dx"))),
                       Mul(Ref(IRIdentifier(id_a, "dy")),
                           Ref(IRIdentifier(id_b, "dy")))),
                   Mul(Ref(IRIdentifier(id_a, "dz")),
                       Ref(IRIdentifier(id_b, "dz")))));
      auto cos = idc.Get("cos");
      auto recip_a = idc.Get("recip_a");
      auto recip_b = idc.Get("recip_b");
      Let(recip_a, Div(Lit("1"), ra));
      Let(recip_b, Div(Lit("1"), rb));
      Let(cos, Mul(Ref(dot), Mul(Ref(recip_a), Ref(recip_b))));
      auto thet = idc.Get("thet");
      Let(thet, Acos(Ref(cos)));
      if (mode == kForward) return Return(Ref(thet));
      auto old_adj = state->adjoint->Clone();
      auto adj_acos = idc.Get("adj_acos");
      Let(adj_acos, Neg(Div(state->adjoint,
                            Sqrt(Sub(Lit("1"), Mul(Ref(cos), Ref(cos)))))));
      auto adj_a = idc.Get("adj_a");
      auto adj_b = idc.Get("adj_b");
      auto adj_c = idc.Get("adj_c");
      // TODO(markus) recheck this, esp also signs
      Let(adj_a,
          Mul(Ref(adj_acos), Sub(Ref(recip_b), Mul(Ref(cos), Ref(recip_a)))));
      Let(adj_b,
          Mul(Ref(adj_acos), Sub(Ref(recip_a), Mul(Ref(cos), Ref(recip_b)))));
      Let(adj_c,
          Neg(Mul(Ref(adj_acos), Mul(rc, Mul(Ref(recip_a), Ref(recip_b))))));
      WithMode(kReverse, [&]() {
        state->adjoint = Ref(adj_a);
        HandleDistance(args[1], args[0], id_a);
        state->adjoint = Ref(adj_b);
        HandleDistance(args[1], args[2], id_b);
        state->adjoint = Ref(adj_c);
        HandleDistance(args[0], args[2], id_c);
      });
      state->adjoint = std::move(old_adj);
      if (mode == kBoth)
        return Return(Ref(thet));
      else
        return Return(Invalid("__REVERSE_NO_VALUE_ERROR__"));
    }
    if (expr->GetName() == "_px" || expr->GetName() == "_py" ||
        expr->GetName() == "_pz") {
      auto result = NextId("x");
      assert(expr->GetArguments().size() == 1);
      auto idx = AcceptForward(expr->GetArguments()[0]);
      if (expr->GetName() == "_px") {
        Let(result, LoopAtomX(idx));
      } else if (expr->GetName() == "_py") {
        Let(result, LoopAtomY(idx));
      } else {
        Let(result, LoopAtomZ(idx));
      }
      if (mode == kForward) return Return(Ref(result));
      if (expr->GetName() == "_px") {
        LoopAccForceX(idx, Neg(state->adjoint));
      } else if (expr->GetName() == "_py") {
        LoopAccForceY(idx, Neg(state->adjoint));
      } else {
        LoopAccForceZ(idx, Neg(state->adjoint));
      }
      if (mode == kBoth)
        return Return(Ref(result));
      else
        return Return(Invalid("__REVERSE_NO_VALUE_ERROR__"));
    } else if (expr->GetName() == "sqrt") {
      return Return(HandleBuiltinFunction(
          expr, "sqrt", [](const std::unique_ptr<IRExpr>& x) {
            return BinOp("/", Lit("0.5"), Sqrt(x));
          }));
    }
    if (expr->GetName() == "ffi") {
      // assert first arg is identifierexpr, take its str repr
      // add arglist, do not care about forward or reverse
      assert(expr->GetArguments().size() == 2);
      assert(typeid(*expr->GetArguments()[0].get()) == typeid(IdentifierExpr));
      IdentifierExpr* name =
          static_cast<IdentifierExpr*>(expr->GetArguments()[0].get());
      assert(name->GetArguments().empty());
      assert(mode == kForward);
      auto arg = Accept(expr->GetArguments()[1]);
      return Return(
          std::make_unique<FunCallIRExpr>(name->GetName(), std::move(arg)));
    } else if (expr->GetName() == "_ffi") {
      assert(expr->GetArguments().size() >= 2);
      assert(typeid(*expr->GetArguments()[0].get()) == typeid(IdentifierExpr));
      IdentifierExpr* name =
          static_cast<IdentifierExpr*>(expr->GetArguments()[0].get());
      assert(name->GetArguments().empty());
      IdentifierExpr* spec =
          static_cast<IdentifierExpr*>(expr->GetArguments()[1].get());
      assert(spec->GetArguments().empty());
      assert(spec->GetName().size() == expr->GetArguments().size() - 2);
      // Always kBoth, same as spline
      // Can, in principle, also pass in atom indices directly
      // I.e. need to pass in adjoint
      // Signature: Function(lammps, adjoint, arg0, arg1, arg2, arg3, *darg0,
      // *darg1, *darg2);
      auto adjoint = Lit("0");
      if (mode != kForward) adjoint = state->adjoint->Clone();
      std::vector<std::unique_ptr<IRExpr>> args;
      int num_args = expr->GetArguments().size() - 2;
      args.push_back(Lit("lammps"));
      args.push_back(adjoint->Clone());
      IRIdentifier result = NextId("sff");
      std::vector<IRIdentifier> other_names;
      for (int i = 0; i < num_args; i++) {
        args.push_back(AcceptForward(expr->GetArguments()[i + 2]));
      }
      for (int i = 0; i < num_args; i++) {
        if (spec->GetName()[i] == 'd') continue;
        other_names.push_back(NextId("ffd"));
      }
      LetComplexFunCall(name->GetName(), result, other_names, std::move(args));
      if (mode == kForward) return Return(Ref(result));
      int index = 0;
      for (int i = 0; i < num_args; i++) {
        if (spec->GetName()[i] == 'd') continue;
        AcceptReverse(expr->GetArguments()[i + 2],
                      Mul(state->adjoint, Ref(other_names[index++])));
      }
      if (mode == kBoth)
        return Return(Ref(result));
      else
        return Return(Invalid("__REVERSE_NO_VALUE_ERROR__"));
    } else if (expr->GetName() == "type") {
      assert(expr->GetArguments().size() == 1);
      if (mode != kReverse)
        return Return(LoopAtomType(AcceptForward(expr->GetArguments()[0])));
      else
        return Return(Invalid("__REVERSE_NO_VALUE_ERROR__"));
    } else if (expr->GetName() == "exp") {
      return Return(HandleBuiltinFunction(expr, "exp", Exp));
    }
    if (expr->GetName() == "sin") {
      return Return(HandleBuiltinFunction(expr, "sin", Cos));
    }
    if (expr->GetName() == "cos") {
      return Return(HandleBuiltinFunction(
          expr, "cos",
          [](const std::unique_ptr<IRExpr>& x) { return Neg(Sin(x)); }));
    }
    if (expr->GetName() == "pi") {
      assert(expr->GetArguments().size() == 0);
      if (mode != kReverse) {
        return Return(Lit("M_PI"));
      } else {
        return Return(Invalid("__REVERSE_NO_VALUE_ERROR__"));
      }
    } else if (expr->GetName() == "force") {
      assert(expr->GetArguments().size() == 2);
      if (mode == kForward) return Return(Lit("0"));
      auto fpair = AcceptForward(expr->GetArguments()[0]);
      AcceptReverse(expr->GetArguments()[1], Mul(state->adjoint, fpair));
      if (mode == kBoth)
        return Return(Lit("0"));
      else
        return Return(Invalid("__REVERSE_NO_VALUE_ERROR__"));
    } else if (expr->GetName() == "random_gaussian") {
      assert(expr->GetArguments().size() == 0);
      assert(mode == kForward);
      return Return(std::make_unique<FunCallIRExpr>("random_gaussian"));
    } else if (expr->GetName() == "derivative") {
      assert(expr->GetArguments().size() == 3);
      auto value = AcceptForward(expr->GetArguments()[0]);
      if (mode == kForward) return Return(value);
      auto adjoint = AcceptForward(expr->GetArguments()[1]);
      AcceptReverse(expr->GetArguments()[2], Mul(state->adjoint, adjoint));
      if (mode == kReverse)
        return Return(value);
      else
        return Return(Invalid("__REVERSE_NO_VALUE_ERROR__"));
    } else if (expr->GetName() == "alternative") {
      assert(expr->GetArguments().size() == 2);
      if (mode == kForward)
        return Return(Accept(expr->GetArguments()[0]));
      else if (mode == kReverse)
        return Return(Accept(expr->GetArguments()[1]));
      else
        assert(0 && "Ambiguous alternative usage");
    }
    // Lookup the thing: In particular, there are certain BuiltinNamedDecls that
    // are handled seperately
    // by methods in this class.
    // Need to distinguish what to generate based on what the identifier is
    // referring to.
    // Ideally, this would work by having a "parent" pointer to walk up until
    // the scope becomes visible.
    // Failing that, there could be a formal lookup as well, i.e. not mapping
    // from symbol to code value,
    // but to syntax element that introduced the symbol.
    return Return(Invalid("IDENTIFIER<" + expr->GetName() + ">"));
  }
  // Inference visitor to resolve implicits, replace builtins by appropriate
  // nodes,
  // and add casts from atom id to atom type.
  void Visit(SumExpr* expr) override {
    auto acc = NextId("a");
    if (mode == kForward || mode == kBoth) DeclAcc(acc);
    CodeEmitterState nested{state, state->adjoint->Clone(), expr->GetIndex()};

    bool old_peratom_mode = peratom_mode;
    peratom_mode = false;

    //    assert(nested.lookup[expr->GetIndex()] == iter_val);
    WithState(&nested, [&]() {
      auto iter_val = Accept(expr->GetIter());
      LoopVarsStart(iter_val);

      if (old_peratom_mode) {
        assert(typeid(*expr->GetIter()) == typeid(NeighborsListExpr));
        NeighborsListExpr* neigh_expr =
            static_cast<NeighborsListExpr*>(expr->GetIter());
        auto name_outer = neigh_expr->GetIndex();
        auto name_inner = expr->GetIndex();
        bool found;
        auto ref_outer = state->Lookup(name_outer, found);
        assert(found);
        auto ref_inner = state->Lookup(name_inner, found);
        assert(found);

        std::vector<std::unique_ptr<IRExpr>> args;
        args.push_back(ref_inner->Clone());
        CodeEmitterState nested_flip{
            state, Mul(state->adjoint, std::make_unique<LookupIRExpr>(
                                           LookupIRExpr::kPerAtomAdjoint,
                                           peratom_name, std::move(args)))};
        nested_flip.lookup[name_outer] = ref_inner->Clone();
        nested_flip.lookup[name_inner] = ref_outer->Clone();
        state->LookupImplicit(name_outer, found);
        if (found) nested_flip.implicits[name_outer] = name_outer;
        state->LookupImplicit(name_inner, found);
        if (found) nested_flip.implicits[name_inner] = name_inner;
        WithState(&nested_flip, [&]() {
          auto val_flip = Accept(expr->GetExpr());
          if (mode == kForward || mode == kBoth) {
            std::vector<std::unique_ptr<IRExpr>> args;
            args.push_back(iter_val->Clone());
            PerAtomAcc(peratom_name, std::move(args), val_flip);
          }
        });
        state->code.AddAndConsume(nested_flip.code);
        args.push_back(ref_outer->Clone());
        state->adjoint =
            Mul(state->adjoint,
                std::make_unique<LookupIRExpr>(LookupIRExpr::kPerAtomAdjoint,
                                               peratom_name, std::move(args)));
      }

      auto val = Accept(expr->GetExpr());
      if (mode == kForward || mode == kBoth) {
        Acc(acc, val);
      }
      LoopVarsEnd(iter_val);
    });

    ForIRStmt* f = state->code.back()->asa<ForIRStmt>();
    assert(f);
    f->body->AddAndConsume(nested.code);

    peratom_mode = old_peratom_mode;

    if (mode == kForward || mode == kBoth)
      return Return(Ref(acc));
    else
      return Return(Invalid("__REVERSE_NO_VALUE_ERROR__"));
  }
  void Visit(LetExpr* expr) override {
    // Tricky
    // Option: Within GetExpr(), accumulate all the derivatives w.r.t. GetName()
    // Then pass into adjoint calculation of GetValue().
    // I.e. Need to record and accumulate adjoints there.
    // Maybe adjoint_lookup[...] += adjoint there?
    CodeEmitterState nested{state, state->adjoint->Clone()};
    if (mode == kForward) {
      auto ret = WithState(&nested, [&]() {
        state->lookup[expr->GetName()] = Accept(expr->GetValue());
        return Accept(expr->GetExpr());
      });
      state->code.AddAndConsume(nested.code);
      return Return(ret);
    } else if (mode == kReverse) {
      auto ret = WithState(&nested, [&]() {
        state->lookup[expr->GetName()] = AcceptForward(expr->GetValue());
        auto acc = NextId("let_acc_adj");
        state->adjoint_lookup[expr->GetName()] = acc;
        DeclAcc(acc);
        Accept(expr->GetExpr());
        auto adj = NextId("let_adj");
        Let(adj, Ref(acc));
        auto old_adj = std::move(state->adjoint);
        state->adjoint = Ref(adj);
        auto ret = Accept(expr->GetValue());
        state->adjoint = std::move(old_adj);
        return ret;
      });
      state->code.AddAndConsume(nested.code);
      return Return(ret);
    } else {
      assert(mode == kBoth);
      auto ret = WithState(&nested, [&]() {
        state->lookup[expr->GetName()] = AcceptForward(expr->GetValue());
        auto acc = NextId("let_acc_adj");
        state->adjoint_lookup[expr->GetName()] = acc;
        DeclAcc(acc);
        auto ret = Accept(expr->GetExpr());
        auto adj = NextId("let_adj");
        Let(adj, Ref(acc));
        auto old_adj = std::move(state->adjoint);
        state->adjoint = Ref(adj);
        WithMode(kReverse, [&]() { Accept(expr->GetValue()); });
        state->adjoint = std::move(old_adj);
        return ret;
      });
      state->code.AddAndConsume(nested.code);
      return Return(ret);
    }
  }
  void Visit(ImplicitExpr* expr) override {
    CodeEmitterState nested = {state, state->adjoint->Clone()};
    for (auto& a : expr->GetImplicits()) {
      nested.implicits[a.first] = a.second;
    }
    auto ret = WithState(&nested, [&]() { return Accept(expr->GetExpr()); });
    state->code.AddAndConsume(nested.code);
    return Return(ret);
  }
  void HandlePieces(std::vector<std::unique_ptr<PiecewiseExpr::Piece>>& pieces,
                    int pos, IRIdentifier var) {
    if (pos == pieces.size()) {
      InvalidStmt("else no expr");
      return;
    }
    auto& piece = *pieces[pos];
    auto first = AcceptForward(piece.first);
    auto mid = AcceptForward(piece.mid);
    std::string first_op = piece.greater ? ">" : "<";
    if (piece.first_equal) first_op += "=";
    auto cond_expr = BinOp(first_op, first, mid);
    if (piece.last.get()) {
      auto last = AcceptForward(piece.last);
      std::string last_op = piece.greater ? ">" : "<";
      if (piece.last_equal) last_op += "=";
      cond_expr = BinOp("&&", BinOp(last_op, mid, last), std::move(cond_expr));
    }
    CodeEmitterState nested_expr{state, state->adjoint->Clone()};
    auto result = WithState(&nested_expr, [&]() { return Accept(piece.expr); });
    if (mode != kReverse) {
      nested_expr.code.Add(
          std::make_unique<AssignIRStmt>(var, std::move(result)));
    }
    CodeEmitterState nested_other{state, state->adjoint->Clone()};
    WithState(&nested_other, [&]() { HandlePieces(pieces, pos + 1, var); });
    state->code.Add(std::make_unique<IfIRStmt>(std::move(cond_expr),
                                               nested_expr.code.Move(),
                                               nested_other.code.Move()));
  }
  void Visit(PiecewiseExpr* expr) override {
    assert(expr->GetPieces().size() > 0);
    if (mode != kReverse) {
      auto var = NextId("v");
      DeclAssign(var);
      HandlePieces(expr->GetPieces(), 0, var);
      return Return(Ref(var));
    } else {
      auto var = NextId("invalid");
      HandlePieces(expr->GetPieces(), 0, var);
      return Return(Invalid("__NO_REVERSE_VALUE__"));
    }
  }
  void Visit(AddExpr* expr) override {
    if (mode == kForward) {
      std::vector<std::unique_ptr<IRExpr>> val, val_neg;
      for (auto& ex : expr->GetExprs()) {
        val.push_back(Accept(ex));
      }
      for (auto& ex : expr->GetExprsNeg()) {
        val_neg.push_back(Accept(ex));
      }
      auto var = NextId("v");
      auto add_expr = Lit("0");
      for (auto& v : val) {
        add_expr = Add(add_expr, v);
      }
      for (auto& v : val_neg) {
        add_expr = Sub(add_expr, v);
      }
      Let(var, add_expr);
      return Return(Ref(var));
    } else if (mode == kReverse) {
      for (auto& ex : expr->GetExprs()) {
        Accept(ex);
      }
      auto old_adj = std::move(state->adjoint);
      auto adj = NextId("adj_neg");
      Let(adj, Neg(old_adj));
      state->adjoint = Ref(adj);
      for (auto& ex : expr->GetExprsNeg()) {
        Accept(ex);
      }
      state->adjoint = std::move(old_adj);
      return Return(Invalid("__REVERSE_NO_VALUE_ERROR__"));
    } else {
      assert(mode == kBoth);
      std::vector<std::unique_ptr<IRExpr>> val, val_neg;
      for (auto& ex : expr->GetExprs()) {
        val.push_back(Accept(ex));
      }
      auto old_adj = std::move(state->adjoint);
      auto adj = NextId("adj_neg");
      Let(adj, Neg(old_adj));
      state->adjoint = Ref(adj);
      for (auto& ex : expr->GetExprsNeg()) {
        val_neg.push_back(Accept(ex));
      }
      state->adjoint = std::move(old_adj);
      auto var = NextId("v");
      auto add_expr = Lit("0");
      for (auto& v : val) {
        add_expr = Add(add_expr, v);
      }
      for (auto& v : val_neg) {
        add_expr = Sub(add_expr, v);
      }
      Let(var, add_expr);
      return Return(Ref(var));
    }
  }
  void Visit(MulExpr* expr) override {
    if (mode == kForward) {
      std::vector<std::unique_ptr<IRExpr>> val, val_inv;
      for (auto& ex : expr->GetExprs()) {
        val.push_back(Accept(ex));
      }
      for (auto& ex : expr->GetExprsInv()) {
        val_inv.push_back(Accept(ex));
      }
      auto var = NextId("mul_val");
      auto exp = Lit("1");
      for (auto& v : val) {
        exp = Mul(std::move(exp), std::move(v));
      }
      for (auto& v : val_inv) {
        auto rec = NextId("recip");
        Let(rec, Div(Lit("1"), v));
        exp = Mul(std::move(exp), Ref(rec));
      }
      Let(var, exp);
      return Return(Ref(var));
    } else {
      bool ENABLE_SCALE_MODE =
          true && expr->GetExprsInv().empty() && (expr->GetExprs().size() > 1);
      assert(mode == kReverse || mode == kBoth);
      std::vector<std::unique_ptr<IRExpr>> val, val_inv;
      for (auto& ex : expr->GetExprs()) {
        if (ENABLE_SCALE_MODE && (&ex == &expr->GetExprs().back())) continue;
        val.push_back(AcceptForward(ex));
      }
      if (ENABLE_SCALE_MODE) {
        auto adjoint = state->adjoint->Clone();
        for (auto& elem : val) {
          adjoint = Mul(elem, adjoint);
        }
        val.push_back(WithMode(kBoth, [&]() {
          auto old_adj = state->adjoint->Clone();
          state->adjoint = adjoint->Clone();
          auto ret = Accept(expr->GetExprs().back());
          state->adjoint = old_adj->Clone();
          return ret;
        }));
      }
      for (auto& ex : expr->GetExprsInv()) {
        val_inv.push_back(AcceptForward(ex));
      }
      WithMode(kReverse, [&]() {
        auto old_adj = state->adjoint->Clone();
        for (auto& ex : expr->GetExprs()) {
          if (ENABLE_SCALE_MODE && (&ex == &expr->GetExprs().back())) continue;
          IRIdentifier adj = NextId("mul_adj");
          auto adj_expr = old_adj->Clone();
          for (auto& v : val)
            if (&ex - &expr->GetExprs()[0] != &v - &val[0])
              adj_expr = Mul(adj_expr, v);
          for (auto& v : val_inv) adj_expr = Div(adj_expr, v);
          Let(adj, adj_expr);
          state->adjoint = Ref(adj);
          Accept(ex);
        }
        for (auto& ex : expr->GetExprsInv()) {
          IRIdentifier adj = NextId("mul_adj");
          auto adj_expr = old_adj->Clone();
          for (auto& v : val) adj_expr = Mul(adj_expr, v);
          for (auto& v : val_inv) {
            auto rec = NextId("recip");
            Let(rec, Div(Lit("1"), v));
            if (&ex - &expr->GetExprsInv()[0] != &v - &val_inv[0])
              adj_expr = Mul(adj_expr, Ref(rec));
            else
              adj_expr = Neg(Mul(adj_expr, Mul(Ref(rec), Ref(rec))));
          }
          Let(adj, adj_expr);
          state->adjoint = Ref(adj);
          Accept(ex);
        }
        state->adjoint = std::move(old_adj);
      });
      if (mode == kReverse)
        return Return(Invalid("__REVERSE_NO_VALUE_ERROR__"));
      auto var = NextId("v");
      auto var_expr = Lit("1");
      for (auto& v : val) {
        var_expr = Mul(var_expr, v);
      }
      for (auto& v : val_inv) {
        auto rec = NextId("recip");
        Let(rec, Div(Lit("1"), v));
        var_expr = Mul(var_expr, Ref(rec));
      }
      Let(var, var_expr);
      return Return(Ref(var));
    }
  }
  void Visit(PowExpr* expr) override {
    if (expr->GetRight() == nullptr) return Return(Accept(expr->GetLeft()));
    auto vleft = AcceptForward(expr->GetLeft());
    auto vright = AcceptForward(expr->GetRight());
    auto var = NextId("v");
    Let(var, Pow(vleft, vright));
    if (mode == kForward) return Return(Ref(var));
    auto old_adj = std::move(state->adjoint);
    auto adj_left = NextId("adj");
    auto adj_right = NextId("adj");

    // state->code.push_back(std::make_unique<DeclAssignIRStmt>(adj_left));
    // CompoundIRStmt code_zero;
    // code_zero.push_back(std::make_unique<AssignIRStmt>(adj_left, Mul(old_adj,
    // vright)));
    // CompoundIRStmt code_nonzero;
    // code_nonzero.push_back(std::make_unique<AssignIRStmt>(adj_left,
    //          Mul(old_adj, Mul(Div(Pow(vleft, vright), vleft), vright))));
    // state->code.push_back(std::make_unique<IfIRStmt>(BinOp("==", vleft,
    // Lit("0.0")), code_zero.Move(), code_nonzero.Move()));
    // bool arg_is_strictly_positive = true; // TODO(markus) deduce this
    // if (arg_is_strictly_positive) {
    //  state->code.push_back(
    //      Let(adj_left,
    //          Mul(old_adj, Mul(Div(Pow(vleft, vright), vleft), vright))));
    //} else {
    Let(adj_left, Mul(old_adj, Mul(Pow(vleft, Sub(vright, Lit("1"))), vright)));
    //}
    Let(adj_right, Mul(old_adj, Mul(Ref(var), Log(vleft))));
    WithMode(kReverse, [&]() {
      state->adjoint = Ref(adj_left);
      Accept(expr->GetLeft());
      state->adjoint = Ref(adj_right);
      Accept(expr->GetRight());
    });
    state->adjoint = std::move(old_adj);
    if (mode == kReverse) return Return(Invalid("__REVERSE_NO_VALUE_ERROR__"));
    return Return(Ref(var));
  }
  void Visit(AllAtomsListExpr* expr) override {
    auto var = NextId("i");
    state->parent->code.Add(std::make_unique<ForIRStmt>(
        var, Lit("0"), std::make_unique<LookupIRExpr>(LookupIRExpr::kNumLocal),
        std::make_unique<CompoundIRStmt>()));
    state->lookup[state->iterator_name] = Ref(var);
    LoopVarsBegin(Ref(var));
    return Return(Ref(var));
  }
  // Needs to generate both full and half neighbor list traversals
  // The problem: The mechanism to halve a neighbor list differs based on
  // the generation backend and its neighlist generation settings:
  // - If only half lists are needed at all, then one can just use that.
  // - If both half and full lists are needed, it depends on the backend:
  //   - For USER-INTEL only the upper loop bound changes.
  //   - For regular LAMMPS one has to manually filter.
  // Options:
  // - Do other steps for the different backends.
  // - Encode both at the same time, i.e. a "continue if other half"
  //   "upper limit neigh  list"
  void Visit(NeighborsListExpr* expr) override {
    auto var = NextId("i");
    bool found;
    auto index_var = state->Lookup(expr->GetIndex(), found);
    assert(found);
    auto upper_kind = expr->GetHalf() ? LookupIRExpr::kNeighborHalfNum
                                      : LookupIRExpr::kNeighborNum;
    if (expr->GetGreater().empty()) {
      state->parent->code.Add(std::make_unique<ForIRStmt>(
          var, Lit("0"),
          std::make_unique<LookupIRExpr>(upper_kind, index_var->Clone()),
          std::make_unique<CompoundIRStmt>()));
    } else {
      state->parent->code.Add(std::make_unique<ForIRStmt>(
          var,
          BinOp("+", Lit("1"), state->LookupIndex(expr->GetGreater(), found),
                kIRDataTypeInt),
          std::make_unique<LookupIRExpr>(upper_kind, index_var->Clone()),
          std::make_unique<CompoundIRStmt>()));
      assert(found);
    }

    auto atom = NextId("a");
    Let(atom, std::make_unique<LookupIRExpr>(LookupIRExpr::kNeighborEntry,
                                             index_var->Clone(), Ref(var)),
        kIRDataTypeInt);
    is_potentially_nonlocal_neighbor[atom] = true;
    state->lookup[state->iterator_name] = Ref(atom);
    state->lookup_index[state->iterator_name] = Ref(var);

    if (expr->GetHalf()) {
      auto half_filter = std::make_unique<ContinueIRStmt>(
          BinOp("__half__", index_var, Ref(atom)));
      half_filter->half = true;
      state->code.Add(std::move(half_filter));
    }

    LoopVarsBegin(Ref(atom));

    // int id = next_identifier_++;
    idc.Advance();

    auto dx = idc.Get("dx");
    auto dy = idc.Get("dy");
    auto dz = idc.Get("dz");
    Let(dx, Sub(LoopAtomX(index_var), LoopAtomX(Ref(atom))));
    Let(dy, Sub(LoopAtomY(index_var), LoopAtomY(Ref(atom))));
    Let(dz, Sub(LoopAtomZ(index_var), LoopAtomZ(Ref(atom))));
    auto rsq = idc.Get("rsq");
    Let(rsq, Add(Add(Mul(Ref(dx), Ref(dx)), Mul(Ref(dy), Ref(dy))),
                 Mul(Ref(dz), Ref(dz))));
    auto cutoff = AcceptForward(expr->GetCutoff());
    state->code.Add(std::make_unique<ContinueIRStmt>(
        BinOp(">=", Ref(rsq), Mul(cutoff, cutoff))));
    for (auto& excl : expr->GetExclusion()) {
      state->code.Add(std::make_unique<ContinueIRStmt>(
          BinOp("==", Ref(atom), state->Lookup(excl, found), kIRDataTypeInt)));
      assert(found);
    }
    return Return(Ref(atom));
  }
  void Visit(TypematchExpr* expr) override {
    /*
      int tmp0 = type_map[type[elem0]];
      int tmp1 = type_map[type[elem1]];

    */
    std::vector<IRIdentifier> mapped_types;
    for (int i = 0; i < expr->GetArgs().size(); i++) {
      IRIdentifier id = NextId("mti");
      bool found;
      auto the_atom = state->Lookup(expr->GetArgs()[i], found);
      assert(found);
      auto the_expr = std::make_unique<LookupIRExpr>(LookupIRExpr::kTypeMap,
                                                     std::move(the_atom));
      Let(id, std::move(the_expr), kIRDataTypeInt);
      mapped_types.push_back(id);
    }
    IRIdentifier res = NextId("mtv");
    CompoundIRStmt* insert_here = &state->code;
    insert_here->Add(std::make_unique<DeclAssignIRStmt>(res));
    auto& pieces = expr->GetPieces();
    for (int i = 0; i < pieces.size(); i++) {
      CodeEmitterState nested_then{state, state->adjoint->Clone()};
      CodeEmitterState nested_other{state, state->adjoint->Clone()};
      auto val =
          WithState(&nested_then, [&]() { return Accept(pieces[i]->expr); });
      nested_then.code.Add(std::make_unique<AssignIRStmt>(res, val->Clone()));
      auto cond = Lit("1");
      for (int j = 0; j < mapped_types.size(); j++) {
        auto type = pieces[i]->types[j];
        bool found = false;
        for (int k = 0; k < def->types.size(); k++) {
          if (def->types[k] == type) found = true;
        }
        if (!found) def->types.push_back(type);
        cond = BinOp("&&", cond, BinOp("==", Ref(mapped_types[j]),
                                       Lit("type_var_" + pieces[i]->types[j]),
                                       kIRDataTypeInt), kIRDataTypeBool);
      }
      auto stmt = std::make_unique<IfIRStmt>(
          std::move(cond), nested_then.code.Move(), nested_other.code.Move());
      CompoundIRStmt* next_insert = stmt->otherwise.get();
      insert_here->Add(std::move(stmt));
      insert_here = next_insert;
    }
    insert_here->Add(std::make_unique<InvalidIRStmt>("typematch no expr"));
    return Return(Ref(res));
  }
  void AcceptPerAtom(std::string name, std::string atom_var, Expr* expr) {
    // One loop. Pattern match on the second loop.
    // Half the second loop.
    // What about cutoff checks?
    // How to see if symmetric?
    // Easiest: Don't halve, yet.
    // Zero out
    CodeEmitterState nested_zero{state, state->adjoint->Clone()};
    auto var = NextId("i");
    WithState(&nested_zero, [&]() {
      std::vector<std::unique_ptr<IRExpr>> args;
      args.push_back(Ref(var));
      PerAtomZero(name, std::move(args));
      args.push_back(Ref(var));
      PerAtomAdjointZero(name, std::move(args));
    });
    AddStmt(std::make_unique<ForIRStmt>(
        var, Lit("0"), std::make_unique<LookupIRExpr>(LookupIRExpr::kNumAll),
        std::make_unique<CompoundIRStmt>(std::move(nested_zero.code))));

    assert(typeid(*expr) == typeid(SumExpr));
    SumExpr* sum_expr = static_cast<SumExpr*>(expr);
    assert(typeid(*sum_expr->GetIter()) == typeid(NeighborsListExpr));
    NeighborsListExpr* neigh_expr =
        static_cast<NeighborsListExpr*>(sum_expr->GetIter());
    assert(neigh_expr->half_ == false);
    neigh_expr->half_ = true;
    peratom_mode = true;
    peratom_name = name;
    // Flow: Mark Sum. Acc makes the correct steps.
    // Needs to know nothing.

    CodeEmitterState nested{state, state->adjoint->Clone()};
    var = NextId("i");
    WithState(&nested, [&]() {
      state->lookup[atom_var] = Ref(var);
      LoopVarsBegin(Ref(var));
      auto val = AcceptForward(expr);
      std::vector<std::unique_ptr<IRExpr>> args;
      args.push_back(Ref(var));
      PerAtomAcc(name, std::move(args), val);
    });
    AddStmt(std::make_unique<ForIRStmt>(
        var, Lit("0"), std::make_unique<LookupIRExpr>(LookupIRExpr::kNumLocal),
        std::make_unique<CompoundIRStmt>(std::move(nested.code))));
    PerAtomComm(name);

    peratom_mode = false;
    peratom_name = "";
    neigh_expr->half_ = false;
  }
  void AcceptPerAtomReverse(std::string name, std::string atom_var,
                            Expr* expr) {
    PerAtomAdjointComm(name);

    assert(typeid(*expr) == typeid(SumExpr));
    SumExpr* sum_expr = static_cast<SumExpr*>(expr);
    assert(typeid(*sum_expr->GetIter()) == typeid(NeighborsListExpr));
    NeighborsListExpr* neigh_expr =
        static_cast<NeighborsListExpr*>(sum_expr->GetIter());
    assert(neigh_expr->half_ == false);
    neigh_expr->half_ = true;
    peratom_mode = true;
    peratom_name = name;

    CodeEmitterState nested{state, state->adjoint->Clone()};
    auto var = NextId("i");
    WithState(&nested, [&]() {
      state->lookup[atom_var] = Ref(var);
      LoopVarsBegin(Ref(var));
      LoopVarsStart(Ref(var));
      AcceptReverse(expr, state->adjoint->Clone());
      LoopVarsEnd(Ref(var));
    });
    AddStmt(std::make_unique<ForIRStmt>(
        var, Lit("0"), std::make_unique<LookupIRExpr>(LookupIRExpr::kNumLocal),
        std::make_unique<CompoundIRStmt>(std::move(nested.code))));

    peratom_mode = false;
    peratom_name = "";
    neigh_expr->half_ = false;
  }
};

}  // namespace

std::unique_ptr<CompoundIRStmt> GenerateCode(
    PotentialDef* def, IRIdentifierContext* next_identifier) {
  // Find all used peratom values
  // TODO(markus) why is this not -1???
  CodeEmitterExprVisitor vis(def, CodeEmitterExprVisitor::kBoth, Lit("1"));
  vis.idc = *next_identifier;
  int num_peratom = 0;
  for (auto& decl : def->GetDecls()) {
    if (decl->GetKind() == Decl::kPerAtom) {
      auto per_decl = static_cast<PerAtomDecl*>(decl);
      auto expr = per_decl->GetExpr();
      assert(per_decl->GetArguments().size() == 1);
      vis.AcceptPerAtom(decl->GetName(), per_decl->GetArguments()[0], expr);
      num_peratom += 1;
    }
  }
  for (auto& a : vis.peratom_need_both_directions) {
    printf("peratom both: %s\n", a.c_str());
  }
  std::vector<std::unique_ptr<IRExpr>> energies;
  for (auto& decl : def->GetEnergyDecls()) {
    energies.push_back(vis.Accept(&decl->GetExpr()));
  }
  for (auto it = def->GetDecls().rbegin(); it != def->GetDecls().rend(); it++) {
    auto& decl = *it;
    if (decl->GetKind() == Decl::kPerAtom) {
      auto per_decl = static_cast<PerAtomDecl*>(decl);
      auto expr = per_decl->GetExpr();
      assert(per_decl->GetArguments().size() == 1);
      vis.AcceptPerAtomReverse(decl->GetName(), per_decl->GetArguments()[0],
                               expr);
    }
  }
  for (auto& energy : energies) {
    vis.root.code.Add(std::make_unique<AccEnergyIRStmt>(std::move(energy)));
  }
  // patch up two-way comm
  for (auto& stmt : vis.root.code) {
    auto comm = stmt->asa<PerAtomActionIRStmt>();
    if (!comm) continue;
    if (!comm->IsComm()) continue;
    if (vis.peratom_need_both_directions.find(comm->id) ==
        vis.peratom_need_both_directions.end())
      continue;
    comm->need_both_directions = true;
  }
  // Generate loop to compute that & prepend that
  // Generate the loop to propagate derivatives & append that
  *next_identifier = vis.idc;
  return std::make_unique<CompoundIRStmt>(std::move(vis.root.code));
}

std::pair<std::unique_ptr<CompoundIRStmt>, std::unique_ptr<IRExpr>>
GenerateCode(PotentialDef* def, Expr* expr) {
  CodeEmitterExprVisitor vis(def, CodeEmitterExprVisitor::kForward, Lit("1"));
  auto val = vis.Accept(expr);
  return std::pair<std::unique_ptr<CompoundIRStmt>, std::unique_ptr<IRExpr>>(
      std::make_unique<CompoundIRStmt>(std::move(vis.root.code)),
      std::move(val));
}

}  // namespace der

}  // namespace potc
