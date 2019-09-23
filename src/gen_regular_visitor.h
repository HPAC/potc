// Copyright 2018 Markus Hoehnerbach
#ifndef GEN_REGULAR_VISITOR_H_
#define GEN_REGULAR_VISITOR_H_ GEN_REGULAR_VISITOR_H_

#include <map>
#include <string>
#include <vector>

#include "ana_comm.h"
#include "gen_internal.h"
#include "ir.h"
#include "ir_visitor.h"

namespace potc {

namespace gen {

bool NeedBracket(std::string op_outer, std::string op_inner, bool op_left) {
  if (op_outer == "+" && op_inner == "+") return false;
  if (op_outer == "*" && op_inner == "*") return false;
  if (op_outer == "+" && op_inner == "*") return false;
  if (op_outer == "+" && op_inner == "/") return false;
  if (op_outer == "-" && op_inner == "*") return false;
  if (op_outer == "-" && op_inner == "/") return false;
  if (op_outer == " ") return false;
  if (op_outer == "-" && op_inner == "+" && op_left) return false;
  if (op_outer == "-" && op_inner == "-" && op_left) return false;
  if (op_outer == "/" && op_inner == "*" && op_left) return false;
  if (op_outer == "/" && op_inner == "/" && op_left) return false;
  if (op_outer == "*" && op_inner == "/") return false;
  if (op_outer == "+" && op_inner == "-") return false;
  return true;
}

using potc::ana::AnalyzeCommunication;

struct WriteRegularVisitor : public IRStmtVisitor, public IRExprVisitor {
  std::map<std::string, int> params;
  std::vector<IRDataType> context;
  std::vector<std::string> op_context;
  bool op_left;
  std::string indent;
  bool debug;
  int debug_focus;
  CodeBuilder* cb;
  AnalyzeCommunication* ana_comm;

  bool neigh_is_half = false;

  explicit WriteRegularVisitor(CodeBuilder* c, AnalyzeCommunication* cana_comm)
      : cb(c), indent("  "), ana_comm(cana_comm) {
    op_context.push_back(" ");  // empty case
    op_left = true;
    debug = false;
    debug_focus = 1457;
  }

  void Visit(CompoundIRStmt* stmt) override {
    for (auto& b : stmt->body) {
      b->Accept(this);
    }
  }
  void Visit(LetIRStmt* stmt) override {
    switch (stmt->type) {
      case kIRDataTypeDouble:
        cb->Raw("%sdouble %s = ", indent.c_str(),
                stmt->name.ToString().c_str());
        context.push_back(kIRDataTypeDouble);
        stmt->value->Accept(this);
        context.pop_back();
        cb->Raw(";\n");
        return;
      case kIRDataTypeInt:
        cb->Raw("%sint %s = ", indent.c_str(), stmt->name.ToString().c_str());
        context.push_back(kIRDataTypeInt);
        stmt->value->Accept(this);
        context.pop_back();
        cb->Raw(";\n");
        return;
      default:
        cb->Raw("assert(0); __ERROR_TYPE_NOT_HANDLED__;\n");
        return;
    }
  }
  void Visit(DeclAssignIRStmt* stmt) override {
    cb->Raw("%sdouble %s;\n", indent.c_str(), stmt->name.ToString().c_str());
  }
  void Visit(AssignIRStmt* stmt) {
    cb->Raw("%s%s = ", indent.c_str(), stmt->name.ToString().c_str());
    context.push_back(kIRDataTypeDouble);
    stmt->value->Accept(this);
    context.pop_back();
    cb->Raw(";\n");
  }
  void Visit(DeclAccIRStmt* stmt) override {
    cb->Raw("%sdouble %s = 0.0;  // acc\n", indent.c_str(),
            stmt->name.ToString().c_str());
  }
  void Visit(AccIRStmt* stmt) override {
    cb->Raw("%s%s += ", indent.c_str(), stmt->name.ToString().c_str());
    context.push_back(kIRDataTypeDouble);
    stmt->value->Accept(this);
    context.pop_back();
    cb->Raw(";\n");
  }
  virtual void ForceAcc(IRExpr* idx, IRExpr* val, std::string dim) {
    if (debug) {
      cb->Raw("%sif (%d == (", indent.c_str(), debug_focus);
      context.push_back(kIRDataTypeInt);
      idx->Accept(this);
      context.pop_back();
      cb->Raw(")) {\n");
      cb->Raw("%s  printf(\"%s %s %s %%d = %%f\\n\", ", indent.c_str(),
              idx->ToString().c_str(), val->ToString().c_str(), dim.c_str());
      context.push_back(kIRDataTypeInt);
      idx->Accept(this);
      context.pop_back();
      cb->Raw(", ");
      context.push_back(kIRDataTypeDouble);
      val->Accept(this);
      context.pop_back();
      cb->Raw(");\n");
      cb->Raw("%s}\n", indent.c_str());
    }
    cb->Raw("%sf[", indent.c_str());
    context.push_back(kIRDataTypeInt);
    idx->Accept(this);
    context.pop_back();
    cb->Raw("]%s += ", dim.c_str());
    context.push_back(kIRDataTypeDouble);
    val->Accept(this);
    context.pop_back();
    cb->Raw(";\n");
  }
  void Visit(AccForceIRStmt* stmt) override {
    if (stmt->value_x) ForceAcc(stmt->index.get(), stmt->value_x.get(), "[0]");
    if (stmt->value_y) ForceAcc(stmt->index.get(), stmt->value_y.get(), "[1]");
    if (stmt->value_z) ForceAcc(stmt->index.get(), stmt->value_z.get(), "[2]");
  }
  void Visit(IfIRStmt* stmt) override {
    cb->Raw("%sif (", indent.c_str());
    // TODO(markus) should be evaluated in `bool' context,
    // but for now literals can only occur s.t. they are real
    context.push_back(kIRDataTypeDouble);
    stmt->cond->Accept(this);
    context.pop_back();
    cb->Raw(") {\n");
    std::string old_indent = indent;
    indent += "  ";
    stmt->then->Accept(this);
    cb->Raw("%s} else {\n", old_indent.c_str());
    stmt->otherwise->Accept(this);
    cb->Raw("%s}\n", old_indent.c_str());
    indent = old_indent;
  }
  void Visit(ForIRStmt* stmt) override {
    cb->Raw("%sfor (int %s = ", indent.c_str(), stmt->index.ToString().c_str());
    context.push_back(kIRDataTypeInt);
    stmt->initial->Accept(this);
    cb->Raw("; %s < ", stmt->index.ToString().c_str());
    // TODO(markus) should be evaluated in `bool' context,
    // but for now literals can only occur s.t. they are real
    stmt->top_value->Accept(this);
    cb->Raw("; %s++", stmt->index.ToString().c_str());
    context.pop_back();
    cb->Raw(") {\n");
    std::string old_indent = indent;
    indent += "  ";
    stmt->body->Accept(this);
    cb->Raw("%s}\n", old_indent.c_str());
    indent = old_indent;
  }
  virtual std::string TagStr(std::string idx) { return "tag[" + idx + "]"; }
  virtual std::string PosStr(std::string idx, int dim) {
    return "x[" + idx + "][" + std::to_string(dim) + "]";
  }
  void Visit(ContinueIRStmt* stmt) override {
    if (stmt->half) {
      if (neigh_is_half) return;
      BinIRExpr* b = stmt->cond->asa<BinIRExpr>();
      assert(b);
      assert(b->op == "__half__");
      cb->Raw("%sint i = ", indent.c_str());
      context.push_back(kIRDataTypeDouble);
      b->left->Accept(this);
      cb->Raw(";\n");
      cb->Raw("%sint j = ", indent.c_str());
      b->right->Accept(this);
      context.pop_back();
      cb->Fragment(";\n");
      cb->Fragment(indent + "tagint itag = " + TagStr("i") + ";\n");
      cb->Fragment(indent + "tagint jtag = " + TagStr("j") + ";\n");
      cb->Raw("%sif (itag > jtag) {\n", indent.c_str());
      cb->Raw("%s  if ((itag+jtag) %% 2 == 0) continue;\n", indent.c_str());
      cb->Raw("%s} else if (itag < jtag) {\n", indent.c_str());
      cb->Raw("%s  if ((itag+jtag) %% 2 == 1) continue;\n", indent.c_str());
      cb->Raw("%s} else {\n", indent.c_str());
      cb->Fragment(indent + "  if (" + PosStr("j", 2) + " <  " +
                   PosStr("i", 2) + ") continue;\n");
      cb->Fragment(indent + "  if (" + PosStr("j", 2) + " == " +
                   PosStr("i", 2) + " && ");
      cb->Fragment(PosStr("j", 1) + " <  " + PosStr("i", 1) + ") continue;\n");
      cb->Fragment(indent + "  if (" + PosStr("j", 2) + " == " +
                   PosStr("i", 2) + " && " + PosStr("j", 1) + " ==  " +
                   PosStr("i", 1) + " && " + PosStr("j", 0) + " <  " +
                   PosStr("i", 0) + ") continue;\n");
      cb->Raw("%s}\n", indent.c_str());
      return;
    }
    cb->Raw("%sif (", indent.c_str());
    context.push_back(kIRDataTypeDouble);
    stmt->cond->Accept(this);
    context.pop_back();
    cb->Raw(") continue;\n");
  }
  void Visit(AccEnergyIRStmt* stmt) override {
    cb->Raw("%seng_vdwl += ", indent.c_str());
    context.push_back(kIRDataTypeDouble);
    stmt->value->Accept(this);
    context.pop_back();
    cb->Raw(";\n");
  }
  void Visit(InvalidIRStmt* stmt) override {
    cb->Raw("%serror->one(FLERR,\"%s\");\n", indent.c_str(),
            stmt->reason.c_str());
  }
  void Visit(LetComplexFunCallIRStmt* stmt) override {
    if (!stmt->other_names.empty()) {
      cb->Raw("%sdouble ", indent.c_str());
      bool first = true;
      for (auto& n : stmt->other_names) {
        if (!first) {
          cb->Raw(", ");
        }
        first = false;
        cb->Raw("%s", n.ToString().c_str());
      }
      cb->Raw(";\n");
    }
    cb->Raw("%sdouble %s = %s(", indent.c_str(), stmt->name.ToString().c_str(),
            stmt->function.c_str());
    bool first = true;
    for (auto& arg : stmt->args) {
      if (!first) {
        cb->Raw(", ");
      }
      first = false;
      context.push_back(kIRDataTypeDouble);
      arg->Accept(this);
      context.pop_back();
    }
    for (auto& n : stmt->other_names) {
      if (!first) {
        cb->Raw(", ");
      }
      first = false;
      cb->Raw("&%s", n.ToString().c_str());
    }
    cb->Raw(");\n");
  }

  void Visit(BinIRExpr* expr) override {
    assert(!op_context.empty());
    bool need_bracket = NeedBracket(op_context.back(), expr->op, op_left);
    if (need_bracket) cb->Raw("(");
    op_context.push_back(expr->op);
    op_left = true;
    expr->left->Accept(this);
    cb->Raw(" %s ", expr->op.c_str());
    if (expr->op == "/") cb->Raw("((double) ");
    op_left = false;
    expr->right->Accept(this);
    if (expr->op == "/") cb->Raw(")");
    op_context.pop_back();
    if (need_bracket) cb->Raw(")");
  }
  void Visit(UnIRExpr* expr) override {
    assert(!op_context.empty());
    if (op_context.back() == " ") {
      cb->Raw("%s ", expr->op.c_str());
      expr->inner->Accept(this);
    } else {
      cb->Raw("(%s ", expr->op.c_str());
      expr->inner->Accept(this);
      cb->Raw(")");
    }
  }
  void Visit(RefIRExpr* expr) override {
    cb->Raw("%s", expr->ref.ToString().c_str());
  }
  void Visit(LitIRExpr* expr) {
    if (context.empty()) {
      cb->Raw("/*untyped*/ ");
      cb->Raw("%s", expr->value.c_str());
    } else if (context.back() == kIRDataTypeDouble) {
      // cb->Raw("((double) %s)", expr->value.c_str());
      cb->Raw("%s", expr->value.c_str());
    } else {
      cb->Raw("%s", expr->value.c_str());
    }
  }
  void Visit(FunCallIRExpr* expr) override {
    cb->Raw("%s(", expr->name.c_str());
    bool first = true;
    for (auto& arg : expr->args) {
      if (!first) cb->Raw(", ");
      first = false;
      arg->Accept(this);
    }
    cb->Raw(")");
  }
  virtual void VisitParameter(LookupIRExpr* expr) {
    context.push_back(kIRDataTypeInt);
    auto& name = expr->id;
    if (expr->lookup_kind == LookupIRExpr::kOutlinedParam)
      cb->Raw("outlined_param_");
    if (expr->lookup_kind == LookupIRExpr::kParam) cb->Raw("param_");
    if (expr->lookup_kind == LookupIRExpr::kPerAtom) cb->Raw("peratom_");
    if (expr->lookup_kind == LookupIRExpr::kPerAtomAdjoint)
      cb->Raw("peratom_adjoint_");
    cb->Raw("%s", name.c_str());
    if (params.count(name) > 0) {
      assert(params[name] == expr->args.size());
    } else {
      params[name] = expr->args.size();
    }
    for (auto& arg : expr->args) {
      cb->Raw("[");
      arg->Accept(this);
      cb->Raw("]");
    }
    context.pop_back();
  }
  virtual std::vector<std::string> GetLookupFragments(
      LookupIRExpr::LookupKind kind) {
    if (kind == LookupIRExpr::kPosX) {
      return {"x[", "][0]"};
    } else if (kind == LookupIRExpr::kPosY) {
      return {"x[", "][1]"};
    } else if (kind == LookupIRExpr::kPosZ) {
      return {"x[", "][2]"};
    } else if (kind == LookupIRExpr::kNeighborNum) {
      return {"numneigh[", "]"};
    } else if (kind == LookupIRExpr::kNeighborHalfNum) {
      return {"numneigh[", "]"};
    } else if (kind == LookupIRExpr::kNeighborEntry) {
      return {"firstneigh[", "][", "]"};
    } else if (kind == LookupIRExpr::kNumLocal) {
      return {"nlocal"};
    } else if (kind == LookupIRExpr::kNumAll) {
      return {"(nlocal + atom->nghost)"};
    } else if (kind == LookupIRExpr::kTypeMap) {
      return {"type_map[type[", "]]"};
    } else if (kind == LookupIRExpr::kType) {
      return {"type[", "]"};
    } else if (kind == LookupIRExpr::kListNum) {
      return {"listnum_", ""};
    } else if (kind == LookupIRExpr::kListHalfNum) {
      return {"listnum_", ""};
    } else if (kind == LookupIRExpr::kListEntry) {
      return {"listentry_", "[", "]"};
    } else if (kind == LookupIRExpr::kMasterCutoff) {
      return {"cutsq[", "][", "]"};
    } else {
      assert(0);
    }
  }
  void Visit(LookupIRExpr* expr) override {
    if (expr->lookup_kind == LookupIRExpr::kParam ||
        expr->lookup_kind == LookupIRExpr::kOutlinedParam ||
        expr->lookup_kind == LookupIRExpr::kPerAtom ||
        expr->lookup_kind == LookupIRExpr::kPerAtomAdjoint) {
      VisitParameter(expr);
      return;
    }
    std::vector<std::string> fragments = GetLookupFragments(expr->lookup_kind);
    context.push_back(kIRDataTypeInt);
    assert(expr->args.size() == fragments.size() - 1);
    int idx = 0;
    for (auto& arg : expr->args) {
      cb->Raw("%s", fragments[idx].c_str());
      arg->Accept(this);
      idx += 1;
    }
    cb->Raw("%s", fragments[idx].c_str());
    context.pop_back();
  }
  void Visit(InvalidIRExpr* expr) override {
    cb->Raw("(error->one(FLERR,\"%s\"), 0)", expr->reason.c_str());
  }
  void Visit(DeclListIRStmt* stmt) override {
    auto n = stmt->name.ToString();
    cb->Raw("%sint listnum_%s = 0;\n", indent.c_str(), n.c_str());
    cb->Raw("%sint listentry_%s[neighbor->oneatom];\n", indent.c_str(),
            n.c_str());
  }
  void Visit(AddListIRStmt* stmt) override {
    auto n = stmt->name.ToString();
    cb->Raw("%slistentry_%s[listnum_%s++] = ", indent.c_str(), n.c_str(),
            n.c_str());
    stmt->value->Accept(this);
    cb->Raw(";\n");
  }
  void Visit(PerAtomActionIRStmt* stmt) override {
    if (stmt->kind == PerAtomActionIRStmt::kPerAtomAdjointAcc) {
      cb->Raw("%speratom_adjoint_%s[", indent.c_str(), stmt->id.c_str());
      stmt->args[0]->Accept(this);
      cb->Raw("] += ");
      stmt->args[1]->Accept(this);
      cb->Raw(";\n");
    } else if (stmt->kind == PerAtomActionIRStmt::kPerAtomAcc) {
      cb->Raw("%speratom_%s[", indent.c_str(), stmt->id.c_str());
      stmt->args[0]->Accept(this);
      cb->Raw("] += ");
      stmt->args[1]->Accept(this);
      cb->Raw(";\n");
    } else if (stmt->IsComm()) {
      if (ana_comm->should_generate[stmt]) {
        int stage = ana_comm->stage_assignment_reverse[stmt];
        cb->Raw("%sstage_peratom = %d;\n", indent.c_str(), stage);
        if (ana_comm->IsReverse(stage))
          cb->Raw("%scomm->reverse_comm_pair(this);  // comm: %s\n",
                  indent.c_str(), stmt->id.c_str());
        if (ana_comm->IsForward(stage))
          cb->Raw("%scomm->forward_comm_pair(this);  // comm: %s\n",
                  indent.c_str(), stmt->id.c_str());
      } else {
        cb->Raw("%s// comm: %s\n", indent.c_str(), stmt->id.c_str());
      }
    } else if (stmt->kind == PerAtomActionIRStmt::kPerAtomZero) {
      assert(stmt->args.size() == 1);
      cb->Raw("%speratom_%s[", indent.c_str(), stmt->id.c_str());
      stmt->args[0]->Accept(this);
      cb->Raw("] = 0;\n");
    } else if (stmt->kind == PerAtomActionIRStmt::kPerAtomAdjointZero) {
      assert(stmt->args.size() == 1);
      cb->Raw("%speratom_adjoint_%s[", indent.c_str(), stmt->id.c_str());
      stmt->args[0]->Accept(this);
      cb->Raw("] = 0;\n");
    } else {
      assert(0);
    }
  }
};

}  // namespace gen

}  // namespace potc

#endif  // GEN_REGULAR_VISITOR_H_
