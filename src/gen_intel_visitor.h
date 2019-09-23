// Copyright 2018 Markus Hoehnerbach
#ifndef GEN_INTEL_VISITOR_H_
#define GEN_INTEL_VISITOR_H_ GEN_INTEL_VISITOR_H_

#include <string>
#include <vector>

#include "gen_regular_visitor.h"

namespace potc {

namespace gen {

struct WriteIntelVisitor : public WriteRegularVisitor {
  explicit WriteIntelVisitor(CodeBuilder* out, AnalyzeCommunication* ana_comm)
      : WriteRegularVisitor(out, ana_comm) {
    indent = "      ";
  }

  void Visit(AccForceIRStmt* stmt) override {
    if (stmt->value_x) ForceAcc(stmt->index.get(), stmt->value_x.get(), ".x");
    if (stmt->value_y) ForceAcc(stmt->index.get(), stmt->value_y.get(), ".y");
    if (stmt->value_z) ForceAcc(stmt->index.get(), stmt->value_z.get(), ".z");
  }

  void Visit(ContinueIRStmt* stmt) override {
    if (stmt->half) return;
    //  WriteRegularVisitor::Visit(stmt);
  }

  std::vector<std::string> GetLookupFragments(
      LookupIRExpr::LookupKind kind) override {
    if (kind == LookupIRExpr::kPosX) {
      return {"x[", "].x"};
    } else if (kind == LookupIRExpr::kPosY) {
      return {"x[", "].y"};
    } else if (kind == LookupIRExpr::kPosZ) {
      return {"x[", "].z"};
    } else if (kind == LookupIRExpr::kNeighborEntry) {
      return {"firstneigh[cnumneigh[", "]+", "]"};
    } else if (kind == LookupIRExpr::kNeighborHalfNum) {
      return {"numneighhalf[", "]"};
    } else if (kind == LookupIRExpr::kThreadNumLocalFrom) {
      return {"iifrom"};
    } else if (kind == LookupIRExpr::kThreadNumLocalTo) {
      return {"iito"};
    } else if (kind == LookupIRExpr::kTypeMap) {
      return {"type_map[x[", "].w]"};
    } else if (kind == LookupIRExpr::kType) {
      return {"x[", "].w"};
    } else {
      return WriteRegularVisitor::GetLookupFragments(kind);
    }
  }

  void Visit(AccEnergyIRStmt* stmt) override {
    cb->Raw("%soevdwl += ", indent.c_str());
    context.push_back(kIRDataTypeDouble);
    stmt->value->Accept(this);
    context.pop_back();
    cb->Raw(";\n");
  }
};

struct WriteIntelVectorVisitor : public WriteIntelVisitor {
  bool is_in_vector_region;
  IRIdentifier vector_index;
  IRExpr* vector_top_value;
  int temp_counter;
  std::vector<int> mask_context;
  std::vector<IRIdentifier> loop_context;
  bool debug_check_let_finite = false;
  bool ana_neigh_need_full = true;

  // enum class VectorKind {
  //  Scalar, Broadcasted, Linear, Vector
  //};
  // std::map<IRIdentifier, VectorKind> vector_kinds;

  explicit WriteIntelVectorVisitor(CodeBuilder* out,
                                   AnalyzeCommunication* ana_comm)
      : WriteIntelVisitor(out, ana_comm) {
    indent = "      ";
    is_in_vector_region = false;
    temp_counter = 1;
    mask_context.push_back(0);
  }

  void Visit(LetIRStmt* stmt) override {
    switch (stmt->type) {
      case kIRDataTypeDouble:
        cb->Raw("%sfvec %s = ", indent.c_str(), stmt->name.ToString().c_str());
        context.push_back(kIRDataTypeDouble);
        stmt->value->Accept(this);
        context.pop_back();
        cb->Raw(";\n");
        if (debug_check_let_finite) {
          cb->Raw("%sfor (int lane = 0; lane < fvec::VL; lane++) {\n",
                  indent.c_str());
          cb->Raw("%s  if (! bvec::test_at(t_%d, lane)) continue;\n",
                  indent.c_str(), mask_context.back());
          cb->Raw("%s  assert(isfinite(fvec::at(%s, lane)));\n", indent.c_str(),
                  stmt->name.ToString().c_str());
          cb->Raw("%s}\n", indent.c_str());
        }
        return;
      case kIRDataTypeInt:
        cb->Raw("%sivec %s = ", indent.c_str(), stmt->name.ToString().c_str());
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
    cb->Raw("%sfvec %s;\n", indent.c_str(), stmt->name.ToString().c_str());
  }
  void Visit(AssignIRStmt* stmt) {
    auto name = stmt->name.ToString();
    cb->Raw("%s%s = fvec::mask_blend(t_%d, %s, ", indent.c_str(), name.c_str(),
            mask_context.back(), name.c_str());
    context.push_back(kIRDataTypeDouble);
    stmt->value->Accept(this);
    context.pop_back();
    cb->Raw(");\n");
  }
  void Visit(DeclAccIRStmt* stmt) override {
    cb->Raw("%sfvec %s = fvec::setzero();\n", indent.c_str(),
            stmt->name.ToString().c_str());
  }
  void Visit(AccIRStmt* stmt) override {
    auto name = stmt->name.ToString();
    cb->Raw("%s%s = fvec::mask_add(%s, t_%d, %s, ", indent.c_str(),
            name.c_str(), name.c_str(), mask_context.back(), name.c_str());
    context.push_back(kIRDataTypeDouble);
    stmt->value->Accept(this);
    context.pop_back();
    cb->Raw(");\n");
  }
  void Visit(AccForceIRStmt* stmt) override {
    int mask_temp = temp_counter++;
    int idx_temp = temp_counter++;
    int val_temp_x = temp_counter++;
    int val_temp_y = temp_counter++;
    int val_temp_z = temp_counter++;
    // cb->Raw("%sivec t_%d = (", indent.c_str(), idx_temp);
    cb->Raw("%sivec t_%d = ivec::shift<2>(", indent.c_str(), idx_temp);
    context.push_back(kIRDataTypeInt);
    stmt->index->Accept(this);
    context.pop_back();
    cb->Raw(");\n");
    int nacc = 0;
    if (stmt->value_x) {
      cb->Raw("%savec t_%d = ", indent.c_str(), val_temp_x);
      context.push_back(kIRDataTypeDouble);
      stmt->value_x->Accept(this);
      context.pop_back();
      cb->Raw(";\n");
      nacc += 1;
    }
    if (stmt->value_y) {
      cb->Raw("%savec t_%d = ", indent.c_str(), val_temp_y);
      context.push_back(kIRDataTypeDouble);
      stmt->value_y->Accept(this);
      context.pop_back();
      cb->Raw(";\n");
      nacc += 1;
    }
    if (stmt->value_z) {
      cb->Raw("%savec t_%d = ", indent.c_str(), val_temp_z);
      context.push_back(kIRDataTypeDouble);
      stmt->value_z->Accept(this);
      context.pop_back();
      cb->Raw(";\n");
      nacc += 1;
    }
    cb->Raw("%sbvec t_%d = t_%d;\n", indent.c_str(), mask_temp,
            mask_context.back());
    cb->Raw("%savec::accumulate_%d(t_%d, t_%d", indent.c_str(), nacc, mask_temp,
            idx_temp);
    if (stmt->value_x) {
      cb->Raw(", &f[0].x, t_%d", val_temp_x);
    }
    if (stmt->value_y) {
      cb->Raw(", &f[0].y, t_%d", val_temp_y);
    }
    if (stmt->value_z) {
      cb->Raw(", &f[0].z, t_%d", val_temp_z);
    }
    cb->Raw(");\n");
  }
  void Visit(IfIRStmt* stmt) override {
    int mask_id_then = temp_counter++;
    int mask_id_else = temp_counter++;
    cb->Raw("%sbvec t_%d = bvec::kand(t_%d, ", indent.c_str(), mask_id_then,
            mask_context.back());
    context.push_back(kIRDataTypeDouble);
    stmt->cond->Accept(this);
    context.pop_back();
    cb->Raw(");\n");
    cb->Raw("%sbvec t_%d = bvec::kandn(t_%d, t_%d);\n", indent.c_str(),
            mask_id_else, mask_id_then, mask_context.back());
    cb->Raw("%sif (bvec::test_any_set(t_%d)) {\n", indent.c_str(),
            mask_id_then);
    // TODO(markus) should be evaluated in `bool' context,
    // but for now literals can only occur s.t. they are real
    std::string old_indent = indent;
    indent += "  ";
    mask_context.push_back(mask_id_then);
    stmt->then->Accept(this);
    mask_context.pop_back();
    cb->Raw("%s}\n", old_indent.c_str());
    cb->Raw("%sif(bvec::test_any_set(t_%d)) {\n", old_indent.c_str(),
            mask_id_else);
    mask_context.push_back(mask_id_else);
    stmt->otherwise->Accept(this);
    mask_context.pop_back();
    cb->Raw("%s}\n", old_indent.c_str());
    indent = old_indent;
  }
  void Visit(ForIRStmt* stmt) override {
    if (is_in_vector_region) {
      // index and so on are vector values
      std::string idx = stmt->index.ToString();
      cb->Raw("%sivec %s = ", indent.c_str(), idx.c_str());
      context.push_back(kIRDataTypeInt);
      stmt->initial->Accept(this);
      cb->Raw(";\n");
      int top_val = temp_counter++;
      cb->Raw("%sivec t_%d = ", indent.c_str(), top_val);
      stmt->top_value->Accept(this);
      cb->Raw(";\n");
      context.pop_back();
      int mask_temp = temp_counter++;
      // cb->Raw(
      //        "%sfor (; ! bvec::kortestz(t_%d, t_%d); %s = "
      //        "ivec::add(%s, ivec::set1(1)), t_%d = "
      //        "bvec::kand(t_%d, ivec::cmplt(%s, t_%d))) {\n",
      //        indent.c_str(), mask_temp, mask_temp, idx.c_str(), idx.c_str(),
      //        mask_temp, mask_context.back(), idx.c_str(), top_val);
      //
      cb->Raw("%sfor (;;) {\n", indent.c_str());
      std::string old_indent = indent;
      indent += "  ";
      cb->Raw(
          "%sbvec t_%d = bvec::kand(t_%d, ivec::cmplt(%s, "
          "t_%d));\n",
          indent.c_str(), mask_temp, mask_context.back(), idx.c_str(), top_val);
      cb->Raw("%sif (bvec::test_all_unset(t_%d)) break;\n", indent.c_str(),
              mask_temp, mask_temp);
      mask_context.push_back(mask_temp);
      loop_context.push_back(stmt->index);
      stmt->body->Accept(this);
      loop_context.pop_back();
      mask_context.pop_back();
      cb->Raw("%s%s = %s + ivec::set1(1);\n", indent.c_str(), idx.c_str(),
              idx.c_str());
      indent = old_indent;
      cb->Raw("%s}\n", indent.c_str());
    } else {
      is_in_vector_region = true;

      int counter_temp = temp_counter++;
      cb->Raw("%sfor (int t_%d = ", indent.c_str(), counter_temp);
      context.push_back(kIRDataTypeInt);
      stmt->initial->Accept(
          this);  // TODO(markus) this needs to be computed scalar
      cb->Raw("; t_%d < ", counter_temp);
      // TODO(markus) should be evaluated in `bool' context,
      // but for now literals can only occur s.t. they are real
      stmt->top_value->Accept(this);
      cb->Raw("; t_%d += fvec::VL", counter_temp);
      context.pop_back();
      cb->Raw(") {\n");
      std::string old_indent = indent;
      indent += "  ";
      cb->Raw(
          "%sivec %s = ivec::set_consecutive() + "
          "ivec::set1(t_%d);\n",
          indent.c_str(), stmt->index.ToString().c_str(), counter_temp);
      int for_mask = temp_counter++;
      cb->Raw(
          "%sbvec t_%d = bvec::kand(bvec::full(), ivec::cmplt(%s, "
          "ivec::set1(",
          indent.c_str(), for_mask, stmt->index.ToString().c_str());
      context.push_back(kIRDataTypeInt);
      stmt->top_value->Accept(this);
      context.pop_back();
      cb->Raw(")));\n");
      mask_context.push_back(for_mask);
      stmt->body->Accept(this);
      mask_context.pop_back();
      cb->Raw("%s}\n", old_indent.c_str());
      cb->Raw("%1$s#ifdef _OPENMP\n%1$s#pragma omp barrier\n%1s#endif\n", old_indent.c_str());
      indent = old_indent;

      is_in_vector_region = false;
    }
  }
  void Visit(ContinueIRStmt* stmt) override {
    if (stmt->half) return;
    context.push_back(kIRDataTypeDouble);
    int mask_temp = temp_counter++;
    // cb->Raw("%sbvec t_%d = bvec::kandn(", indent.c_str(),
    //        mask_temp);
    // stmt->cond->Accept(this);
    // cb->Raw(", t_%d);\n", mask_context.back());
    // cb->Raw("%sif (bvec::kortestz(t_%d, t_%d)) continue;\n",
    //        indent.c_str(), mask_temp, mask_temp);
    // mask_context.pop_back();
    // mask_context.push_back(mask_temp);

    cb->Raw("%sbvec t_%d = bvec::kand(", indent.c_str(), mask_temp);
    stmt->cond->Accept(this);
    cb->Raw(", t_%d);\n", mask_context.back());
    cb->Raw("%sif (bvec::test_any_set(t_%d)) {\n", indent.c_str(), mask_temp,
            mask_temp);
    assert(loop_context.size() > 0);
    auto idx = loop_context.back().ToString();
    cb->Raw("%s  %s = ivec::mask_add(%s, t_%d, %s, ivec::set1(1));\n",
            indent.c_str(), idx.c_str(), idx.c_str(), mask_temp, idx.c_str());
    cb->Raw("%s  continue;\n", indent.c_str());
    cb->Raw("%s}\n", indent.c_str());
    context.pop_back();
  }
  void Visit(AccEnergyIRStmt* stmt) override {
    cb->Raw("%soevdwl += fvec::reduce_add(", indent.c_str());
    context.push_back(kIRDataTypeDouble);
    stmt->value->Accept(this);
    context.pop_back();
    cb->Raw(");\n");
  }

  void Visit(BinIRExpr* expr) override {
    assert(!op_context.empty());
    std::string op = expr->op;
    std::string type = expr->sub_type == kIRDataTypeDouble ? "fvec" : "ivec";
    std::string op1 = ", ";
    std::string op2 = ")";
    if (op == "+") {
      op = "(";
      op1 = " + ";
    } else if (op == "-") {
      op = "(";
      op1 = " - ";
    } else if (op == "*") {
      op = "(";
      op1 = " * ";  // mullo?
    } else if (op == "/") {
      // op = "("; op1 = " / ";
      assert(expr->sub_type == kIRDataTypeDouble);
      op = "(";
      op1 = " * fvec::recip(";
      op2 = "))";
    } else if (op == ">") {
      op = type + "::cmpnle(";
    } else if (op == "<") {
      op = type + "::cmplt(";
    } else if (op == "==") {
      op = type + "::cmpeq(";
    } else if (op == "&&") {
      op = "bvec::kand(";
    } else if (op == "<=") {
      op = type + "::cmple(";
    } else if (op == ">=") {
      op = type + "::cmpnlt(";
    } else {
      assert(0);
    }
    op_context.push_back(expr->op);
    cb->Raw("%s(", op.c_str());
    context.push_back(expr->sub_type);
    expr->left->Accept(this);
    cb->Raw(")%s(", op1.c_str());
    expr->right->Accept(this);
    context.pop_back();
    cb->Raw(")%s", op2.c_str());
    op_context.pop_back();
  }
  void Visit(UnIRExpr* expr) override {
    assert(!op_context.empty());
    assert(expr->op == "-");
    cb->Raw("(fvec::setzero() - (");
    expr->inner->Accept(this);
    cb->Raw("))");
  }
  void Visit(LitIRExpr* expr) {
    assert(!context.empty());
    if (context.back() == kIRDataTypeDouble) {
      // cb->Raw("((double) %s)", expr->value.c_str());
      if (expr->value == "0")
        cb->Raw("fvec::setzero()");
      else
        cb->Raw("fvec::set1(%s)", expr->value.c_str());
    } else if (context.back() == kIRDataTypeInt) {
      if (expr->value == "0")
        cb->Raw("ivec::setzero()");
      else
        cb->Raw("ivec::set1(%s)", expr->value.c_str());
    } else if (context.back() == kIRDataTypeBool) {
      if (expr->value == "0")
        cb->Raw("bvec::empty()");
      else if (expr->value == "1")
        cb->Raw("bvec::full()");
      else assert("Booleans have only two literals." && 0);
    }
  }
  void Visit(FunCallIRExpr* expr) override {
    std::string name = expr->name;
    name = "fvec::" + name + "";
    cb->Raw("%s(", name.c_str());
    bool first = true;
    for (auto& arg : expr->args) {
      if (!first) cb->Raw(", ");
      first = false;
      arg->Accept(this);
    }
    cb->Raw(")");
  }
  void cb_MulStr(int n, const char* str) {
    for (int i = 0; i < n; i++) {
      cb->Raw(str);
    }
  }
  void VisitParameter(std::string name,
                      const std::vector<std::unique_ptr<IRExpr>>& args,
                      bool as_double = true) {
    context.push_back(kIRDataTypeInt);
    cb->Raw("(ONETYPE ? fvec::set1(%s", name.c_str());
    cb_MulStr(args.size(), "[1]");
    cb->Raw(") : ");
    if (as_double) {
      cb->Raw(
          "fvec::mask_gather_double(t_%d, "
          "",
          mask_context.back());
    } else {
      cb->Raw(
          "fvec::mask_gather(t_%d, "
          "",
          mask_context.back());
    }
    for (int i = args.size() - 1; i >= 0; i--) {
      auto& arg = args[i];
      cb->Raw("((");
      arg->Accept(this);
      cb->Raw(") + ivec::mullo(ivec::set1(tp1), ");
    }
    cb->Raw("ivec::setzero()");
    cb_MulStr(args.size(), "))");
    cb->Raw(", &%s", name.c_str());
    cb_MulStr(args.size(), "[0]");
    cb->Raw(")");
    cb->Raw(")");
    context.pop_back();
  }
  void Visit(LookupIRExpr* expr) override {
    auto kind = expr->lookup_kind;
    if (kind == LookupIRExpr::kParam || kind == LookupIRExpr::kOutlinedParam) {
      if (params.count(expr->id) > 0) {
        assert(params[expr->id] == expr->args.size());
      } else {
        params[expr->id] = expr->args.size();
      }
      if (kind == LookupIRExpr::kOutlinedParam)
        VisitParameter("this->outlined_param_" + expr->id, expr->args);
      else
        VisitParameter("this->param_" + expr->id, expr->args);
      return;
    }
    std::string pos = "";
    if (kind == LookupIRExpr::kThreadNumLocalFrom) {
      cb->Raw("iifrom");
    } else if (kind == LookupIRExpr::kThreadNumLocalTo) {
      cb->Raw("iito");
    } else if (kind == LookupIRExpr::kThreadNumAllFrom) {
      cb->Raw("iafrom");
    } else if (kind == LookupIRExpr::kThreadNumAllTo) {
      cb->Raw("iato");
    } else if (kind == LookupIRExpr::kPosX) {
      pos = "x";
    } else if (kind == LookupIRExpr::kPosY) {
      pos = "y";
    } else if (kind == LookupIRExpr::kPosZ) {
      pos = "z";
    } else if (kind == LookupIRExpr::kType) {
      cb->Raw(
          "ivec::mask_gather(ivec::undefined(), t_%d, "
          "ivec::mul_fwidth(",
          mask_context.back());
      expr->args[0]->Accept(this);
      cb->Raw("), &x[0].w, 4)");
    } else if (kind == LookupIRExpr::kNeighborEntry) {
      cb->Raw(
          "ivec::mask_gather(ivec::undefined(), t_%d, "
          "ivec::mask_gather(ivec::undefined()"
          ", t_%d, ",
          mask_context.back(), mask_context.back());
      expr->args[0]->Accept(this);
      cb->Raw(", cnumneigh, 4) + (");
      expr->args[1]->Accept(this);
      cb->Raw("), firstneigh, 4)");
    } else if (kind == LookupIRExpr::kNeighborNum || (kind == LookupIRExpr::kNeighborHalfNum && ! ana_neigh_need_full)) {
      cb->Raw("ivec::mask_gather(ivec::undefined(), t_%d, ",
              mask_context.back());
      expr->args[0]->Accept(this);
      cb->Raw(", numneigh, 4)");
    } else if (kind == LookupIRExpr::kNeighborHalfNum) {
      cb->Raw("ivec::mask_gather(ivec::undefined(), t_%d, ",
              mask_context.back());
      expr->args[0]->Accept(this);
      cb->Raw(", numneighhalf, 4)");
    } else if (kind == LookupIRExpr::kTypeMap) {
      cb->Raw(
          "ivec::mask_gather(ivec::undefined(), t_%d, "
          "ivec::mask_gather(ivec::undefined(), t_%d, "
          "ivec::mul_fwidth(",
          mask_context.back(), mask_context.back());
      expr->args[0]->Accept(this);
      cb->Raw("), &x[0].w, 4), type_map, 4)");
    } else if (kind == LookupIRExpr::kListNum) {
      cb->Raw("listnum_");
      expr->args[0]->Accept(this);
    } else if (kind == LookupIRExpr::kListHalfNum) {
      cb->Raw("listhalfnum_");
      expr->args[0]->Accept(this);
    } else if (kind == LookupIRExpr::kListEntry) {
      cb->Raw(
          "ivec::mask_gather(ivec::undefined(), t_%d, "
          "ivec::set_consecutive() + ivec::mul_vl(",
          mask_context.back());
      expr->args[1]->Accept(this);
      cb->Raw("), listentry_");
      expr->args[0]->Accept(this);
      cb->Raw(", 4)");
    } else if (kind == LookupIRExpr::kMasterCutoff) {
      VisitParameter("this->cutsq", expr->args);
    } else if (kind == LookupIRExpr::kNumAll) {
      cb->Raw("(nlocal + atom->nghost)");
    } else if (kind == LookupIRExpr::kPerAtom) {
      cb->Raw(
          "fvec::mask_gather_double(t_%d, ",
          mask_context.back());
      context.push_back(kIRDataTypeInt);
      expr->args[0]->Accept(this);
      context.pop_back();
      cb->Raw(", this->peratom_%s)", expr->id.c_str());
    } else if (kind == LookupIRExpr::kPerAtomAdjoint) {
      cb->Raw(
          "fvec::mask_gather_double(t_%d, ",
          mask_context.back());
      context.push_back(kIRDataTypeInt);
      expr->args[0]->Accept(this);
      context.pop_back();
      cb->Raw(", this->peratom_adjoint_%s)", expr->id.c_str());
    } else {
      printf("Unhandled kind: %d.\n", kind);
      assert(0);
    }
    if (pos.empty()) return;
    cb->Raw(
        "fvec::mask_gather<4>(fvec::undefined(), t_%d, "
        "ivec::mul_fwidth(",
        mask_context.back());
    expr->args[0]->Accept(this);
    cb->Raw("), &x[0].%s)", pos.c_str());
  }
  void Visit(DeclListIRStmt* stmt) override {
    auto name = stmt->name.ToString();
    auto n = name.c_str();
    cb->Raw("%sivec listnum_%s = ivec::setzero();\n", indent.c_str(), n);
    cb->Raw("%sivec listhalfnum_%s = ivec::setzero();\n", indent.c_str(), n);
    cb->Raw("%sint listentry_%s[fvec::VL * neighbor->oneatom];\n",
            indent.c_str(), n);
  }
  void Visit(AddListIRStmt* stmt) override {
    auto name = stmt->name.ToString();
    auto n = name.c_str();
    cb->Raw(
        "%sivec::mask_scatter(listentry_%s, t_%d, "
        "ivec::set_consecutive() + ivec::mul_vl(listnum_%s), ",
        indent.c_str(), n, mask_context.back(), n);
    stmt->value->Accept(this);
    cb->Raw(", 4);\n");
    cb->Raw(
        "%slistnum_%s = ivec::mask_add(listnum_%s, t_%d, "
        "listnum_%s, ivec::set1(1));\n",
        indent.c_str(), n, n, mask_context.back(), n);
    if (stmt->half_check) {
      cb->Raw(
          "%slisthalfnum_%s = ivec::mask_add(listhalfnum_%s, "
          "bvec::kand(t_%d, ",
          indent.c_str(), n, n, mask_context.back());
      stmt->half_check->Accept(this);
      cb->Raw("), listhalfnum_%s, ivec::set1(1));\n", n);
    }
  }
  void Visit(LetComplexFunCallIRStmt* stmt) override {
    // TODO(markus) not yet implemented
    cb->Raw("/* letcomplex is not optimized yet */\n");
    // int spline_temp = temp_counter++;
    if (!stmt->other_names.empty()) {
      cb->Raw("%sfvec  ", indent.c_str());
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

    cb->Raw("%sfvec %s = fns<flt_t,acc_t>(fc, this, t_%d).%s(", indent.c_str(),
            stmt->name.ToString().c_str(), mask_context.back(), stmt->function.c_str());
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

  void Visit(PerAtomActionIRStmt* stmt) override {
    // TODO(markus) Need(!) to generate the "compute" pass differently when
    // using Intel
    // If done well in kokkos, this should work since the updates are all atomic
    // Could also add an "atomic accumulate" thing to the vec lib
    // There is effectively no guarantee that adjoint accumulation in the
    // general
    // case can be performed without going beyond thread boundaries
    // As such, need accumulate_atomic_double(...);
    // The other answer is to disable threading, which might be cheaper
    // TODO(markus) re-model the threading to occur on a per-loop basis
    if (stmt->kind == PerAtomActionIRStmt::kPerAtomAdjointAcc) {
      cb->Raw("%sfvec::mask_accumulate_double(peratom_adjoint_%s, t_%d, ",
              indent.c_str(), stmt->id.c_str(), mask_context.back());
      context.push_back(kIRDataTypeInt);
      stmt->args[0]->Accept(this);
      context.pop_back();
      cb->Raw(", ");
      context.push_back(kIRDataTypeDouble);
      stmt->args[1]->Accept(this);
      context.pop_back();
      cb->Raw(");\n");
    } else if (stmt->kind == PerAtomActionIRStmt::kPerAtomAcc) {
      cb->Raw("%sfvec::mask_accumulate_double(peratom_%s, t_%d, ",
              indent.c_str(), stmt->id.c_str(), mask_context.back());
      context.push_back(kIRDataTypeInt);
      stmt->args[0]->Accept(this);
      context.pop_back();
      cb->Raw(", ");
      context.push_back(kIRDataTypeDouble);
      stmt->args[1]->Accept(this);
      context.pop_back();
      cb->Raw(");\n");
    } else if (stmt->kind == PerAtomActionIRStmt::kPerAtomComm) {
      if (ana_comm->should_generate[stmt]) {
        cb->Raw("%sstage_peratom = %d;\n", indent.c_str(),
                ana_comm->stage_assignment_reverse[stmt]);
        cb->Raw("%scomm->reverse_comm_pair(this);  // comm: %s\n",
                indent.c_str(), stmt->id.c_str());
      } else {
        cb->Raw("%s// comm: %s\n", indent.c_str(), stmt->id.c_str());
      }
    } else if (stmt->kind == PerAtomActionIRStmt::kPerAtomAdjointComm) {
      if (ana_comm->should_generate[stmt]) {
        cb->Raw("%sstage_peratom = %d;\n", indent.c_str(),
                ana_comm->stage_assignment_reverse[stmt]);
        cb->Raw("%scomm->forward_comm_pair(this);  // comm: %s\n",
                indent.c_str(), stmt->id.c_str());
      } else {
        cb->Raw("%s// comm: %s\n", indent.c_str(), stmt->id.c_str());
      }
    } else if (stmt->kind == PerAtomActionIRStmt::kPerAtomZero) {
      assert(stmt->args.size() == 1);
      cb->Raw("%sfvec::mask_scatter_double_zero(peratom_%s, t_%d, ",
              indent.c_str(), stmt->id.c_str(), mask_context.back());
      context.push_back(kIRDataTypeInt);
      stmt->args[0]->Accept(this);
      context.pop_back();
      cb->Raw(");\n");
      // simple scatter sufficient and general
      // (even simple store would work, but requires reasoning about
      //  linear vs vector types...)
    } else if (stmt->kind == PerAtomActionIRStmt::kPerAtomAdjointZero) {
      assert(stmt->args.size() == 1);
      cb->Raw("%sfvec::mask_scatter_double_zero(peratom_adjoint_%s, t_%d, ",
              indent.c_str(), stmt->id.c_str(), mask_context.back());
      context.push_back(kIRDataTypeInt);
      stmt->args[0]->Accept(this);
      context.pop_back();
      cb->Raw(");\n");
    } else {
      assert(0);
    }
  }
};
}  // namespace gen

}  // namespace potc

#endif  // GEN_INTEL_VISITOR_H_
