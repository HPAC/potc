// Copyright 2018 Markus Hoehnerbach
#ifndef GEN_KOKKOS_VISITOR_H_
#define GEN_KOKKOS_VISITOR_H_ GEN_KOKKOS_VISITOR_H_

#include <string>
#include <vector>

#include "gen_regular_visitor.h"

namespace potc {

namespace gen {

struct WriteKokkosVisitor : public WriteRegularVisitor {
  bool is_first_for = true;
  bool is_in_first_for = false;

  WriteKokkosVisitor(CodeBuilder* out, AnalyzeCommunication* ana_comm)
      : WriteRegularVisitor(out, ana_comm) {
    indent = "  ";
  }

  void ForceAcc(IRExpr* idx, IRExpr* val, std::string dim) override {
    cb->Fragment(indent + "a_f(");
    context.push_back(kIRDataTypeInt);
    idx->Accept(this);
    context.pop_back();
    cb->Fragment(", " + dim + ") += ");
    context.push_back(kIRDataTypeDouble);
    val->Accept(this);
    context.pop_back();
    cb->Fragment(";\n");
  }

  void Visit(AccForceIRStmt* stmt) override {
    if (stmt->value_x) ForceAcc(stmt->index.get(), stmt->value_x.get(), "0");
    if (stmt->value_y) ForceAcc(stmt->index.get(), stmt->value_y.get(), "1");
    if (stmt->value_z) ForceAcc(stmt->index.get(), stmt->value_z.get(), "2");
  }

  void VisitParameter(LookupIRExpr* expr) override {
    context.push_back(kIRDataTypeInt);
    auto& name = expr->id;
    if (!expr->args.empty()) {
      cb->Raw("kk_");
    }
    if (expr->lookup_kind == LookupIRExpr::kOutlinedParam) cb->Raw("outlined_param_");
    if (expr->lookup_kind == LookupIRExpr::kParam) cb->Raw("param_");
    if (expr->lookup_kind == LookupIRExpr::kPerAtom) cb->Raw("peratom_");
    if (expr->lookup_kind == LookupIRExpr::kPerAtomAdjoint) cb->Raw("peratom_adjoint_");
    cb->Raw("%s", name.c_str());
    if (params.count(name) > 0) {
      assert(params[name] == expr->args.size());
    } else {
      params[name] = expr->args.size();
    }
    if (!expr->args.empty()) {
      cb->Raw("(");
      bool first = true;
      for (auto& arg : expr->args) {
        if (!first) cb->Raw(", ");
        first = false;
        arg->Accept(this);
      }
      cb->Raw(")");
    }
    context.pop_back();
  }
  std::vector<std::string> GetLookupFragments(
      LookupIRExpr::LookupKind kind) override {
    if (kind == LookupIRExpr::kPosX) {
      return {"x(", ", 0)"};
    } else if (kind == LookupIRExpr::kPosY) {
      return {"x(", ", 1)"};
    } else if (kind == LookupIRExpr::kPosZ) {
      return {"x(", ", 2)"};
    } else if (kind == LookupIRExpr::kNeighborNum) {
      return {"d_numneigh(", ")"};
    } else if (kind == LookupIRExpr::kNeighborEntry) {
      return {"d_neighbors(", ", ", ")"};
    } else if (kind == LookupIRExpr::kNeighborHalfNum) {
      return {"d_numneigh(", ")"};
    } else if (kind == LookupIRExpr::kType) {
      return {"type(", ")"};
    } else if (kind == LookupIRExpr::kTypeMap) {
      return {"kk_type_map(type(", "))"};
    } else {
      return WriteRegularVisitor::GetLookupFragments(kind);
    }
  }
  void Visit(LookupIRExpr* expr) override {
    if (expr->lookup_kind == LookupIRExpr::kMasterCutoff)
      cb->Fragment("(cutmax*cutmax)");
    else
      WriteRegularVisitor::Visit(expr);
  }
  void Visit(ForIRStmt* stmt) override {
    if (is_in_first_for) {
      WriteRegularVisitor::Visit(stmt);
    } else {
      assert(is_first_for);
      assert(!is_in_first_for);
      is_in_first_for = true;
      // assert low = 0, hi = nlocal
      assert(stmt->initial->ToString() == LitIRExpr("0").ToString());
      if (stmt->top_value->ToString() ==
             LookupIRExpr(LookupIRExpr::kNumLocal).ToString()) {
        cb->Fragment(indent)->Fragment("if (ii >= nlocal) return;\n");
      } else if (stmt->top_value->ToString() ==
             LookupIRExpr(LookupIRExpr::kNumAll).ToString()) {
        cb->Fragment(indent)->Fragment("if (ii >= nall) return;\n");
      } else { assert(0); }
      cb->Fragment(indent)->Raw("int %s = ii;\n",
                                stmt->index.ToString().c_str());
      stmt->body->Accept(this);
      is_in_first_for = false;
    }
  }

  void Visit(AccEnergyIRStmt* stmt) override {
    cb->Raw("%sev.evdwl += ", indent.c_str());
    context.push_back(kIRDataTypeDouble);
    stmt->value->Accept(this);
    context.pop_back();
    cb->Raw(";\n");
  }

  static const int NUM_LIST_ELEMS = 20;

  void Visit(DeclListIRStmt* stmt) override {
    auto n = stmt->name.ToString();
    cb->Raw("%sint listnum_%s = 0;\n", indent.c_str(), n.c_str());
    cb->Raw("%sint listentry_%s[%d];\n", indent.c_str(), n.c_str(),
            NUM_LIST_ELEMS);
  }

  void Visit(AddListIRStmt* stmt) override {
    auto n = stmt->name.ToString();
    cb->Raw(
        "%sif (listnum_%s >= %d) Kokkos::abort(\"Exceed short list "
        "limit.\");\n",
        indent.c_str(), n.c_str(), NUM_LIST_ELEMS);
    cb->Raw("%slistentry_%s[listnum_%s++] = ", indent.c_str(), n.c_str(),
            n.c_str());
    stmt->value->Accept(this);
    cb->Raw(";\n");
  }

  std::string TagStr(std::string idx) override { return "tag(" + idx + ")"; }
  std::string PosStr(std::string idx, int dim) override {
    return "x(" + idx + ", " + std::to_string(dim) + ")";
  }
  void Visit(InvalidIRStmt* stmt) override {
    cb->Raw("%sKokkos::abort(\"%s\");\n", indent.c_str(), stmt->reason.c_str());
  }

  std::string GetPerAtomViewName(PerAtomActionIRStmt* stmt) {
    assert(stmt->IsComm());
    if (stmt->IsAdjoint()) return "kk_view_peratom_adjoint_" + stmt->id;
    return "kk_view_peratom_" + stmt->id;
  }
  void Visit(PerAtomActionIRStmt* stmt) override {
    if (stmt->kind == PerAtomActionIRStmt::kPerAtomAcc || stmt->kind == PerAtomActionIRStmt::kPerAtomAdjointAcc) {
      cb->Raw("%sa_kk_peratom_", indent.c_str());
      if (stmt->kind == PerAtomActionIRStmt::kPerAtomAdjointAcc)
        cb->Raw("adjoint_");
      cb->Raw("%s(", stmt->id.c_str());
      stmt->args[0]->Accept(this);
      cb->Raw(") += ");
      stmt->args[1]->Accept(this);
      cb->Raw(";\n");
    } else if (stmt->kind == PerAtomActionIRStmt::kPerAtomZero || stmt->kind == PerAtomActionIRStmt::kPerAtomAdjointZero) {
      cb->Raw("%sa_kk_peratom_", indent.c_str());
      if (stmt->kind == PerAtomActionIRStmt::kPerAtomAdjointZero)
        cb->Raw("adjoint_");
      cb->Raw("%s(", stmt->id.c_str());
      stmt->args[0]->Accept(this);
      cb->Raw(") = 0;\n");
    } else if (stmt->IsComm()) {
      if (ana_comm->should_generate[stmt]) {
        int stage = ana_comm->stage_assignment_reverse[stmt];
        cb->Raw("%sstage_peratom = %d;\n", indent.c_str(), stage);
        if (ana_comm->IsReverse(stage)) {
          for (auto decl : ana_comm->stage_assignment[stage]) {
            auto name = GetPerAtomViewName(decl);
            cb->Raw("%s%s.template modify<DeviceType>();\n", indent.c_str(), name.c_str());
            cb->Raw("%s%s.template sync<LMPHostType>();\n", indent.c_str(), name.c_str());
          }
          cb->Raw("%scomm->reverse_comm_pair(this);  // comm: %s\n",
                  indent.c_str(), stmt->id.c_str());
          for (auto decl : ana_comm->stage_assignment[stage]) {
            auto name = GetPerAtomViewName(decl);
            cb->Raw("%s%s.template modify<LMPHostType>();\n", indent.c_str(), name.c_str());
            cb->Raw("%s%s.template sync<DeviceType>();\n", indent.c_str(), name.c_str());
          }
        }
        if (ana_comm->IsForward(stage)) {
          cb->Raw("%scomm->forward_comm_pair(this);  // comm: %s\n",
                  indent.c_str(), stmt->id.c_str());
        }
      } else {
        cb->Raw("%s// comm: %s\n", indent.c_str(), stmt->id.c_str());
      }
    } else {
      WriteRegularVisitor::Visit(stmt);
    }
  }
};

}  // namespace gen

}  // namespace potc

#endif  // GEN_KOKKOS_VISITOR_H_
