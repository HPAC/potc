// Copyright 2018 Markus Hoehnerbach

#ifndef IR_H_
#define IR_H_ IR_H_

#include <cassert>
#include <memory>
#include <string>
#include <vector>

class IRStmt;

struct IRIdentifier {
  std::string hint;
  int seq_num;
  IRIdentifier() : seq_num(0) {}
  explicit IRIdentifier(std::string h) : seq_num(0), hint(h) {}
  IRIdentifier(int seq, std::string h) : seq_num(seq + 1), hint(h) {}
  static IRIdentifier Invalid(const std::string& hint) {
    return IRIdentifier(hint);
  }
  std::string ToString() const {
    if (seq_num == 0) {
      return "(assert(0), (void*)0 /*" + hint + "*/)";
    } else {
      return "i_" + hint + "_" + std::to_string(seq_num);
    }
  }
};

inline bool operator<(const IRIdentifier& a, const IRIdentifier& b) {
  // return a.ToString() < b.ToString();
  if (a.seq_num == b.seq_num) {
    return a.hint < b.hint;
  }
  return a.seq_num < b.seq_num;
}
inline bool operator==(const IRIdentifier& a, const IRIdentifier& b) {
  return (a.hint == b.hint) && (a.seq_num == b.seq_num);
  // return a.ToString() == b.ToString();
}
inline bool operator!=(const IRIdentifier& a, const IRIdentifier& b) {
  return !(a == b);
}

// Stolen from
// https://stackoverflow.com/questions/2590677/how-do-i-combine-hash-values-in-c0x#2595226
template <class T>
inline void hash_combine(std::size_t& seed, const T& v) {
  std::hash<T> hasher;
  seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

namespace std {
template <>
struct hash<IRIdentifier> {
  size_t operator()(const IRIdentifier& id) const noexcept {
    size_t ret = 0;
    hash_combine(ret, id.seq_num);
    hash_combine(ret, id.hint);
    return ret;
  }
};
}  // namespace std

struct IRIdentifierContext {
  size_t next_identifier = 0;
  void Advance() { next_identifier += 1; }
  IRIdentifier Get(const std::string& s) {
    return IRIdentifier(next_identifier, s);
  }
  IRIdentifier Next(const std::string& s) {
    Advance();
    return Get(s);
  }
};

enum IRDataType {
  kIRDataTypeNone,
  kIRDataTypeInt,
  kIRDataTypeDouble,
  kIRDataTypeBool,
  kIRDataTypeReal,
  kIRDataTypeAccum,
  kIRDataTypeLocalAtom,
  kIRDataTypeAnyAtom
};

enum AtomDim { kAtomDimX, kAtomDimY, kAtomDimZ };

inline std::string AtomDimToIndexStr(AtomDim d) {
  switch (d) {
    case kAtomDimX:
      return "0";
    case kAtomDimY:
      return "1";
    case kAtomDimZ:
      return "2";
    default:
      return "__AtomDimToIndexStr__ILLEGAL__";
  }
}

inline std::string AtomDimToLowStr(AtomDim d) {
  switch (d) {
    case kAtomDimX:
      return "x";
    case kAtomDimY:
      return "y";
    case kAtomDimZ:
      return "z";
    default:
      return "__AtomDimToLowStr__ILLEGAL__";
  }
}

struct IRNode {
  virtual ~IRNode() = default;
  template <typename T>
  bool isa() const {
    return typeid(*this) == typeid(T);
  }
  template <typename T>
  T* asa() {
    if (isa<T>()) {
      return static_cast<T*>(this);
    }
    return nullptr;
  }
  template <typename T>
  const T* asa() const {
    if (isa<T>()) {
      return static_cast<const T*>(this);
    }
    return nullptr;
  }
};

// --------------------------------------------

class IRExprVisitor;

struct IRExpr : public IRNode {
  bool is_vector;
  IRDataType type;
  IRExpr() : is_vector(false), type(kIRDataTypeNone) {}
  virtual std::unique_ptr<IRExpr> Clone() const = 0;
  virtual std::string ToString() const = 0;
  virtual void Accept(IRExprVisitor*) = 0;
};

struct BinIRExpr : public IRExpr {
  std::string op;
  std::unique_ptr<IRExpr> left, right;
  IRDataType sub_type;

 public:
  BinIRExpr(std::string opc, std::unique_ptr<IRExpr> leftc,
            std::unique_ptr<IRExpr> rightc, IRDataType st = kIRDataTypeDouble)
      : op(opc),
        left(std::move(leftc)),
        right(std::move(rightc)),
        sub_type(st) {}
  std::unique_ptr<IRExpr> Clone() const override {
    return std::make_unique<BinIRExpr>(op, left->Clone(), right->Clone(),
                                       sub_type);
  }
  std::string ToString() const override {
    return "(" + left->ToString() + " " + op + "<" + std::to_string(sub_type) +
           "> " + right->ToString() + ")";
  }
  void Accept(IRExprVisitor*) override;
};

struct UnIRExpr : public IRExpr {
  std::string op;
  std::unique_ptr<IRExpr> inner;
  UnIRExpr(std::string opc, std::unique_ptr<IRExpr> innerc)
      : op(opc), inner(std::move(innerc)) {}
  std::unique_ptr<IRExpr> Clone() const override {
    return std::make_unique<UnIRExpr>(op, inner->Clone());
  }
  std::string ToString() const override {
    return "(" + op + " " + inner->ToString() + ")";
  }
  void Accept(IRExprVisitor*) override;
};

struct RefIRExpr : public IRExpr {
  IRIdentifier ref;
  explicit RefIRExpr(IRIdentifier r) : ref(r) {}
  std::unique_ptr<IRExpr> Clone() const override {
    return std::make_unique<RefIRExpr>(ref);
  }
  std::string ToString() const override { return ref.ToString(); }
  void Accept(IRExprVisitor*) override;
};

struct IRLitValue {
  // Needs to represent: Integers, Fractions, Multiples of PI
  int num_pi;
  int denom_pi;
  int num;
  int denom;
  static IRLitValue Pi() { return IRLitValue{1, 1, 0, 1}; }
  static IRLitValue Zero() { return IRLitValue{0, 1, 0, 1}; }
  static IRLitValue One() { return IRLitValue{0, 1, 1, 1}; }
};

struct LitIRExpr : public IRExpr {
  std::string value;
  explicit LitIRExpr(std::string v) : value(v) {}
  std::unique_ptr<IRExpr> Clone() const override {
    return std::make_unique<LitIRExpr>(value);
  }
  std::string ToString() const override { return value; }
  void Accept(IRExprVisitor*) override;
};

struct FunCallIRExpr : public IRExpr {
  std::string name;
  std::vector<std::unique_ptr<IRExpr>> args;
  explicit FunCallIRExpr(std::string v) : name(v) {}
  FunCallIRExpr(std::string v, std::unique_ptr<IRExpr> a) : name(v) {
    args.push_back(std::move(a));
  }
  FunCallIRExpr(std::string v, std::unique_ptr<IRExpr> a,
                std::unique_ptr<IRExpr> b)
      : name(v) {
    args.push_back(std::move(a));
    args.push_back(std::move(b));
  }
  std::unique_ptr<IRExpr> Clone() const override {
    auto ret = std::make_unique<FunCallIRExpr>(name);
    for (const auto& v : args) {
      ret->args.push_back(v->Clone());
    }
    return ret;
  }
  std::string ToString() const override {
    std::string ret = name + "(";
    bool first = true;
    for (const auto& a : args) {
      if (!first) ret += ", ";
      first = false;
      ret += a->ToString();
    }
    return ret + ")";
  }
  void Accept(IRExprVisitor*) override;
};

struct InvalidIRExpr : public IRExpr {
  std::string reason;
  explicit InvalidIRExpr(std::string r) : reason(r) {}
  std::unique_ptr<IRExpr> Clone() const override {
    return std::make_unique<InvalidIRExpr>(reason);
  }
  std::string ToString() const override {
    return "(assert(0), (void*)0 /*" + reason + "*/)";
  }
  void Accept(IRExprVisitor*) override;
};

struct LookupIRExpr : public IRExpr {
  std::vector<std::unique_ptr<IRExpr>> args;
  enum LookupKind {
    kParam,
    kOutlinedParam,
    kNumLocal,
    kNumAll,
    kThreadNumLocalTo,
    kThreadNumLocalFrom,
    kThreadNumAllTo,
    kThreadNumAllFrom,
    kPosX,
    kPosY,
    kPosZ,
    kType,
    kInternal,
    kNeighborNum,
    kNeighborHalfNum,
    kNeighborEntry,
    kTypeMap,
    kListNum,
    kListHalfNum,
    kListEntry,
    kMasterCutoff,
    kPerAtom,
    kPerAtomAdjoint
  } lookup_kind;
  std::string id;

  explicit LookupIRExpr(LookupKind c_lookup_kind)
      : lookup_kind(c_lookup_kind) {}
  LookupIRExpr(LookupKind c_lookup_kind, std::string c_id,
               std::vector<std::unique_ptr<IRExpr>> c_args)
      : lookup_kind(c_lookup_kind), id(c_id), args(std::move(c_args)) {}
  LookupIRExpr(LookupKind c_lookup_kind, std::unique_ptr<IRExpr> arg)
      : lookup_kind(c_lookup_kind) {
    args.push_back(std::move(arg));
  }
  LookupIRExpr(LookupKind c_lookup_kind, std::unique_ptr<IRExpr> arg1,
               std::unique_ptr<IRExpr> arg2)
      : lookup_kind(c_lookup_kind) {
    args.push_back(std::move(arg1));
    args.push_back(std::move(arg2));
  }

  std::unique_ptr<IRExpr> Clone() const override {
    auto ret = std::make_unique<LookupIRExpr>(lookup_kind);
    ret->id = id;
    for (auto& arg : args) {
      ret->args.push_back(arg->Clone());
    }
    return ret;
  }
  std::string ToString() const override {
    std::string ret = "(lookup ";
    ret += std::to_string(lookup_kind);
    ret += " " + id;
    for (auto& arg : args) {
      ret += " ";
      ret += arg->ToString();
    }
    ret += ")";
    return ret;
  }
  void Accept(IRExprVisitor*) override;
};

// -----------------------------------------------------------------------------

class IRStmtVisitor;

struct IRStmt : public IRNode {
  template <typename T>
  [[deprecated]] const T* as() const {
    return reinterpret_cast<T*>(this);
  }
  template <typename T>
  [[deprecated]] T* as() {
    return reinterpret_cast<T*>(this);
  }
  static std::string indent(const std::string& s) {
    std::string ret = "";
    size_t pos = 0;
    while (pos < s.size()) {
      size_t line_end = s.find('\n', pos);
      assert(line_end != std::string::npos);
      ret += "  " + s.substr(pos, line_end - pos + 1);
      pos = line_end + 1;
    }
    return ret;
  }
  virtual void Accept(IRStmtVisitor*) = 0;
  virtual std::unique_ptr<IRStmt> Clone() {
    return std::unique_ptr<IRStmt>();
  }
};

struct CompoundIRStmt : public IRStmt {
  std::vector<std::unique_ptr<IRStmt>> body;
  size_t size() const { return body.size(); }
  std::vector<std::unique_ptr<IRStmt>>::iterator begin() {
    return body.begin();
  }
  std::vector<std::unique_ptr<IRStmt>>::iterator end() { return body.end(); }
  CompoundIRStmt() {}
  explicit CompoundIRStmt(std::vector<std::unique_ptr<IRStmt>> b)
      : body(std::move(b)) {}
  IRStmt* back() { return body.back().get(); }
  void Add(std::unique_ptr<IRStmt> arg) {
    if (arg->isa<CompoundIRStmt>()) {
      AddAndConsume(*arg->asa<CompoundIRStmt>());
    } else {
      body.push_back(std::move(arg));
    }
  }
  void AddAndConsume(CompoundIRStmt& stmt) {
    for (auto& v : stmt.body) {
      body.push_back(std::move(v));
    }
  }
  std::unique_ptr<CompoundIRStmt> Move() {
    return std::make_unique<CompoundIRStmt>(std::move(body));
  }
  void Accept(IRStmtVisitor* v) override;
};

// struct DeclIRStmt : public IRStmt {
//  std::vector<IRIdentifier> names;
//  enum DeclKind {
//    kLet, kAccumulate, kAssign, kList, kLetComplex
//  } kind;
//  IRDataType type;
//  std::vector<std::unique_ptr<IRExpr>> args;
//  std::string function; // for LetComplex
//  bool IsModifiable();
//  std::vector<RefIRExpr*> GetRefs();
//  std::vector<ModifyIRStmt*> GetModifications();
//};

struct DeclListIRStmt : public IRStmt {
  IRIdentifier name;
  explicit DeclListIRStmt(IRIdentifier n) : name(n) {}
  void Accept(IRStmtVisitor* v) override;
};

struct AddListIRStmt : public IRStmt {
  IRIdentifier name;
  std::unique_ptr<IRExpr> value;
  std::unique_ptr<IRExpr> half_check;
  explicit AddListIRStmt(IRIdentifier n, std::unique_ptr<IRExpr> v)
      : name(n), value(std::move(v)) {}
  explicit AddListIRStmt(IRIdentifier n, std::unique_ptr<IRExpr> v,
                         std::unique_ptr<IRExpr> h)
      : name(n), value(std::move(v)), half_check(std::move(h)) {}
  void Accept(IRStmtVisitor* v) override;
};

struct LetIRStmt : public IRStmt {
  IRIdentifier name;
  std::unique_ptr<IRExpr> value;
  IRDataType type;
  LetIRStmt(IRIdentifier n, std::unique_ptr<IRExpr> v,
            IRDataType t = kIRDataTypeDouble)
      : name(n), value(std::move(v)), type(t) {}
  LetIRStmt(const LetIRStmt &other) : LetIRStmt(other.name, other.value->Clone(), other.type) {}
  void Accept(IRStmtVisitor* v) override;
  std::unique_ptr<RefIRExpr> Ref() { return std::make_unique<RefIRExpr>(name); }
  std::unique_ptr<IRStmt> Clone() {
    return std::make_unique<LetIRStmt>(*this);
  }
};

struct AssignIRStmt : public IRStmt {
  IRIdentifier name;
  std::unique_ptr<IRExpr> value;
  AssignIRStmt(IRIdentifier n, std::unique_ptr<IRExpr> v)
      : name(n), value(std::move(v)) {}
  void Accept(IRStmtVisitor* v) override;
};

struct DeclAssignIRStmt : public IRStmt {
  IRIdentifier name;
  explicit DeclAssignIRStmt(IRIdentifier n) : name(n) {}
  void Accept(IRStmtVisitor* v) override;
  std::unique_ptr<RefIRExpr> Ref() { return std::make_unique<RefIRExpr>(name); }
  std::unique_ptr<AssignIRStmt> Assign(std::unique_ptr<IRExpr> v) {
    return std::make_unique<AssignIRStmt>(name, std::move(v));
  }
};

struct AccIRStmt : public IRStmt {
  IRIdentifier name;
  std::unique_ptr<IRExpr> value;
  AccIRStmt(IRIdentifier n, std::unique_ptr<IRExpr> v)
      : name(n), value(std::move(v)) {}
  void Accept(IRStmtVisitor* v) override;
};

struct DeclAccIRStmt : public IRStmt {
  IRIdentifier name;
  explicit DeclAccIRStmt(IRIdentifier n) : name(n) {}
  DeclAccIRStmt(const DeclAccIRStmt &other) : DeclAccIRStmt(other.name) {}
  void Accept(IRStmtVisitor* v) override;
  std::unique_ptr<RefIRExpr> Ref() { return std::make_unique<RefIRExpr>(name); }
  std::unique_ptr<AccIRStmt> Acc(std::unique_ptr<IRExpr> v) {
    return std::make_unique<AccIRStmt>(name, std::move(v));
  }
  std::unique_ptr<IRStmt> Clone() {
    return std::make_unique<DeclAccIRStmt>(*this);
  }
};

struct AccForceIRStmt : public IRStmt {
  std::unique_ptr<IRExpr> index, value_x, value_y, value_z;
  AccForceIRStmt(std::unique_ptr<IRExpr> i, std::unique_ptr<IRExpr> vx,
                 std::unique_ptr<IRExpr> vy, std::unique_ptr<IRExpr> vz)
      : index(std::move(i)),
        value_x(std::move(vx)),
        value_y(std::move(vy)),
        value_z(std::move(vz)) {}
  void Accept(IRStmtVisitor* v) override;
};

struct IfIRStmt : public IRStmt {
  std::unique_ptr<IRExpr> cond;
  std::unique_ptr<CompoundIRStmt> then, otherwise;
  IfIRStmt(std::unique_ptr<IRExpr> c, std::unique_ptr<CompoundIRStmt> t,
           std::unique_ptr<CompoundIRStmt> o)
      : cond(std::move(c)), then(std::move(t)), otherwise(std::move(o)) {}
  void Accept(IRStmtVisitor* v) override;
};

enum struct IRThreadMode { kNone, kSingle, kBarrier, kFull };

struct ForIRStmt : public IRStmt {
  IRIdentifier index;
  std::unique_ptr<IRExpr> initial, top_value;
  IRThreadMode thread_mode;
  std::unique_ptr<CompoundIRStmt> body;
  IRStmt* GetBody() const { return body.get(); }
  ForIRStmt(IRIdentifier idx, std::unique_ptr<IRExpr> in,
            std::unique_ptr<IRExpr> to, std::unique_ptr<CompoundIRStmt> b)
      : index(idx),
        initial(std::move(in)),
        top_value(std::move(to)),
        body(std::move(b)) {}
  void Accept(IRStmtVisitor* v) override;
};

struct ContinueIRStmt : public IRStmt {
  std::unique_ptr<IRExpr> cond;
  bool half;
  bool is_cutoff_check;
  explicit ContinueIRStmt(std::unique_ptr<IRExpr> c)
      : cond(std::move(c)), half(false) {}
  void Accept(IRStmtVisitor* v) override;
};

struct AccEnergyIRStmt : public IRStmt {
  std::unique_ptr<IRExpr> value;
  explicit AccEnergyIRStmt(std::unique_ptr<IRExpr> v) : value(std::move(v)) {}
  AccEnergyIRStmt(const AccEnergyIRStmt &other) : AccEnergyIRStmt(other.value->Clone()) {}
  // std::string ToString() const {
  //  return "this->energy += " + value->ToString() + ";\n";
  //}
  void Accept(IRStmtVisitor* v) override;
  std::unique_ptr<IRStmt> Clone() {
    return std::make_unique<AccEnergyIRStmt>(*this);
  }
};

struct InvalidIRStmt : public IRStmt {
  std::string reason;
  explicit InvalidIRStmt(std::string r) : reason(r) {}
  std::string ToString() const { return "assert(0); /*" + reason + "*/\n"; }
  void Accept(IRStmtVisitor* v) override;
};

struct LetComplexFunCallIRStmt : public IRStmt {
  std::string function;
  IRIdentifier name;
  std::vector<IRIdentifier> other_names;
  std::vector<std::unique_ptr<IRExpr>> args;
  LetComplexFunCallIRStmt(std::string fn, IRIdentifier& n,
                          std::vector<IRIdentifier> other_n,
                          std::vector<std::unique_ptr<IRExpr>> v)
      : function(fn), name(n), other_names(other_n), args(std::move(v)) {}
  enum { REGULAR_TOSTRING = -1 };
  // idx == -1-> normal ToString
  // idx == 0 -> result
  // idx == n -> n-th argument
  std::string ToString(int idx) const {
    std::string ret;
    if (!other_names.empty() && idx == REGULAR_TOSTRING) {
      ret += "double ";
      bool first = true;
      for (auto& n : other_names) {
        ret += "";
        if (!first) {
          ret += ", ";
        }
        first = false;
        ret += n.ToString();
      }
      ret += ";\n";
    }
    if (idx == REGULAR_TOSTRING) ret += "double " + name.ToString() + " = ";
    ret += function + "(";
    bool first = true;
    for (auto& a : args) {
      if (!first) {
        ret += ", ";
      }
      first = false;
      ret += a->ToString();
    }
    int other_idx = 1;
    for (auto& n : other_names) {
      if (!first) {
        ret += ", ";
      }
      first = false;
      if (idx == REGULAR_TOSTRING)
        ret += "&" + n.ToString();
      else if (idx == other_idx)
        ret += "X";
      else
        ret += "_";
      other_idx += 1;
    }
    ret += ");\n";
    return ret;
  }
  std::string ToString() const { return ToString(-1); }
  void Accept(IRStmtVisitor* v) override;
};

struct PerAtomActionIRStmt : public IRStmt {
  enum ActionKind {
    //  kAddEnergy,
    //  kAddForce,
    //  kListReset,
    //  kListAdd,
    kPerAtomAcc,
    kPerAtomComm,
    kPerAtomZero,
    kPerAtomAdjointAcc,
    kPerAtomAdjointComm,
    kPerAtomAdjointZero
  } kind;

  enum Direction { None = 0, Forward = 1, Reverse = 2, Both = 3 };

  std::string id;  // if multiple energy kinds, force kinds etc
  std::vector<std::unique_ptr<IRExpr>> args;
  bool need_both_directions = false;
  PerAtomActionIRStmt(ActionKind ckind, std::string cid,
                      std::vector<std::unique_ptr<IRExpr>> cargs)
      : kind(ckind), id(cid), args(std::move(cargs)) {}
  void Accept(IRStmtVisitor* v) override;

  bool IsComm() { return kind == kPerAtomComm || kind == kPerAtomAdjointComm; }

  Direction GetDirection() {
    assert(IsComm());
    if (need_both_directions) return Both;
    if (kind == kPerAtomAdjointComm) return Forward;
    if (kind == kPerAtomComm) return Reverse;
    assert(0);
  }

  Direction AugmentDirection(Direction d) {
    if (d == Both) return d;
    auto m = GetDirection();
    if (d == None) return m;
    if (d == m) return d;
    return Both;
  }

  bool IsForward() { return GetDirection() & Forward; }

  bool IsReverse() { return GetDirection() & Reverse; }

  bool IsAdjoint() {
    assert(IsComm());
    return kind == kPerAtomAdjointComm;
  }
};

#endif  // IR_H_
