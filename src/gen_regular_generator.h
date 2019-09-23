#ifndef GEN_REGULAR_GENERATOR_H_
#define GEN_REGULAR_GENERATOR_H_

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "ana_comm.h"
#include "expr.h"
#include "gen_regular_visitor.h"
#include "gen_spline.h"
#include "ir.h"
#include "ir_visitor_empty.h"

namespace potc {

namespace gen {

struct AnalyzeNeighbors : public TraverseIRVisitor {
  using TraverseIRVisitor::Visit;
  bool need_full = false;
  bool need_ghost = false;
  std::map<IRIdentifier, IRExpr*> lookup;
  int max_num_skips = 0;
  std::map<IRIdentifier, int> num_skips;
  std::map<std::string, int> peratom_num_skips;
  std::map<std::string, int> peratom_adjoint_num_skips;
  int the_num_skip = -1;
  void Finalize() {
    for (auto& elem : num_skips) {
      printf("%s %d\n", elem.first.ToString().c_str(), elem.second);
      if (elem.second > max_num_skips) {
        max_num_skips = elem.second;
      }
    }
    for (auto& elem : peratom_num_skips) {
      int num = elem.second + peratom_adjoint_num_skips[elem.first];
      printf("%s %d %d\n", elem.first.c_str(), elem.second,
             peratom_adjoint_num_skips[elem.first]);
      if (num > max_num_skips) {
        max_num_skips = num;
      }
    }
  }
  IRExpr* UnRef(IRExpr* expr) {
    if (!expr) return expr;
    auto ref = expr->asa<RefIRExpr>();
    if (!ref) return expr;
    return UnRef(lookup[ref->ref]);
  }
  int GetNumSkip(LookupIRExpr* expr) {
    if (expr->lookup_kind == LookupIRExpr::kNeighborNum ||
        expr->lookup_kind == LookupIRExpr::kNeighborHalfNum) {
      auto ur = UnRef(expr->args[0].get());
      if (!ur) {
        return 0;
      } else {
        auto lookup = ur->asa<LookupIRExpr>();
        assert(lookup);  // atoms can either source straight from a loop
                         // (captured above) or from a lookup
        assert(lookup->lookup_kind == LookupIRExpr::kNeighborEntry ||
               lookup->lookup_kind == LookupIRExpr::kListEntry);
        return num_skips[lookup->args[1]->asa<RefIRExpr>()->ref];
      }
    }
    if (expr->lookup_kind == LookupIRExpr::kListNum ||
        expr->lookup_kind == LookupIRExpr::kListHalfNum) {
      return num_skips[expr->args[0]->asa<RefIRExpr>()->ref];
    }
  }
  int GetNumSkips(IRExpr* expr) {
    if (!expr) return 0;
    auto l = expr->asa<LookupIRExpr>();
    if (l) {
      if (l->lookup_kind == LookupIRExpr::kNeighborEntry) {
        return num_skips[l->args[1]->asa<RefIRExpr>()->ref];
      } else if (l->lookup_kind == LookupIRExpr::kListEntry) {
        return num_skips[l->args[1]->asa<RefIRExpr>()->ref];
      }
    }
    assert(0);
  }
  void Visit(LookupIRExpr* expr) override {
    TraverseIRVisitor::Visit(expr);
    if (expr->lookup_kind == LookupIRExpr::kNeighborNum) {
      need_full = true;
    }
    if (expr->lookup_kind == LookupIRExpr::kNeighborNum ||
        expr->lookup_kind == LookupIRExpr::kNeighborHalfNum) {
      assert(expr->args.size() == 1);
      auto ur = UnRef(expr->args[0].get());
      if (ur) {
        auto lookup = ur->asa<LookupIRExpr>();
        assert(lookup);  // atoms can either source straight from a loop
                         // (captured above) or from a lookup
        assert(lookup->lookup_kind == LookupIRExpr::kNeighborEntry ||
               lookup->lookup_kind == LookupIRExpr::kListEntry);
        need_ghost = true;
      }
    }
    if (expr->lookup_kind == LookupIRExpr::kListNum ||
        expr->lookup_kind == LookupIRExpr::kListHalfNum) {
      the_num_skip = num_skips[expr->args[0]->asa<RefIRExpr>()->ref];
    }
    if (expr->lookup_kind == LookupIRExpr::kNeighborNum ||
        expr->lookup_kind == LookupIRExpr::kNeighborHalfNum) {
      assert(expr->args.size() == 1);
      the_num_skip = 1 + GetNumSkips(UnRef(expr->args[0].get()));
    }
    // TODO(markus) Is it necessary to handle PerAtom in this manner?
    // Or is it sufficient to rely on comm?
    // Note that this is conservative:
    // It certainly works this way. But does it work without?
    if (expr->lookup_kind == LookupIRExpr::kPerAtom) {
      int num = GetNumSkips(UnRef(expr->args[0].get()));
      if (peratom_num_skips[expr->id] < num) {
        peratom_num_skips[expr->id] = num;
      }
    }
    if (expr->lookup_kind == LookupIRExpr::kPerAtomAdjoint) {
      int num = GetNumSkips(UnRef(expr->args[0].get()));
      if (peratom_adjoint_num_skips[expr->id] < num) {
        peratom_adjoint_num_skips[expr->id] = num;
      }
    }
  }
  void Visit(LetIRStmt* stmt) override {
    TraverseIRVisitor::Visit(stmt);
    lookup[stmt->name] = stmt->value.get();
  }
  void Visit(ForIRStmt* stmt) override {
    the_num_skip = -1;
    stmt->top_value->Accept(this);
    if (the_num_skip != -1) {
      num_skips[stmt->index] = the_num_skip;
      if (the_num_skip > max_num_skips) max_num_skips = the_num_skip;
      the_num_skip = -1;
    }
    stmt->initial->Accept(this);
    stmt->body->Accept(this);
  }
  void Visit(AddListIRStmt* stmt) override {
    TraverseIRVisitor::Visit(stmt);
    auto l = UnRef(stmt->value.get())->asa<LookupIRExpr>();
    auto r = l->args[1]->asa<RefIRExpr>();
    num_skips[stmt->name] = num_skips[r->ref];
  }
};

using potc::ana::AnalyzeCommunication;

struct RegularGenerator {
  const int MAX_ARITY = 5;

  const char* LICENSE_BLOB =
      "/* -*- c++ -*- "
      "----------------------------------------------------------\n"
      "   LAMMPS - Large-scale Atomic/Molecular Massively Parallel Simulator\n"
      "   http://lammps.sandia.gov, Sandia National Laboratories\n"
      "   Steve Plimpton, sjplimp@sandia.gov\n"
      "   Copyright (2003) Sandia Corporation.  Under the terms of Contract\n"
      "   DE-AC04-94AL85000 with Sandia Corporation, the U.S. Government "
      "retains\n"
      "   certain rights in this software.  This software is distributed "
      "under\n"
      "   the GNU General Public License.\n"
      "   See the README file in the top-level LAMMPS directory.\n"
      "------------------------------------------------------------------------"
      "- "
      "*/\n"
      "\n"
      "/* "
      "----------------------------------------------------------------------\n"
      "   Contributing author: PotC / Markus Hoehnerbach (RWTH Aachen)\n"
      "------------------------------------------------------------------------"
      "- "
      "*/\n\n"
      "";

  const char* LINE =
      "\n/* "
      "---------------------------------------------------------------------- "
      "*/\n";

  std::string GetClassName(std::string name) {
    std::string result = "Pair";
    bool next_cap = true;
    for (auto c : name) {
      if (c == '/') {
        next_cap = true;
      } else {
        if (next_cap) {
          result += toupper(c);
          next_cap = false;
        } else {
          result += c;
        }
      }
    }
    return result;
  }

  std::string GetFileName(std::string name) {
    std::string result = "pair_";
    for (auto c : name) {
      if (c == '/')
        result += '_';
      else
        result += c;
    }
    return result;
  }

  std::string Capitalize(std::string str) {
    for (auto& c : str) c = toupper(c);
    return str;
  }

  std::string name, name_class, name_file, name_file_cap;
  FILE* out_header;
  FILE* out_impl;
  bool nan_catch;
  bool require_newton_on;
  bool require_tag;
  bool require_q;
  PotentialDef* def;
  CompoundIRStmt* compute;
  std::vector<std::pair<int, std::unique_ptr<IRExpr>>>* outlined;
  CodeBuilder c_impl;
  CodeBuilder c_header;
  CodeBuilder* cb_impl;
  CodeBuilder* cb_header;

  int num_peratom;
  AnalyzeNeighbors ana_neigh;
  AnalyzeCommunication ana_comm;

  std::string COEFF_OUT_LINE;
  std::string COEFF_IN_LINE;
  std::string SPLINE_REGULAR_IDX;

  std::string EVAL_MODIFIER;

  RegularGenerator(std::string cname, CompoundIRStmt* ccompute,
                   PotentialDef* cdef,
                   std::vector<std::pair<int, std::unique_ptr<IRExpr>>>* coutl)
      : RegularGenerator(cname, cname, ccompute, cdef, coutl) {}
  RegularGenerator(
      std::string cname, std::string cstyle_name, CompoundIRStmt* ccompute,
      PotentialDef* cdef,
      std::vector<std::pair<int, std::unique_ptr<IRExpr>>>* coutl) {
    name = cstyle_name;
    def = cdef;
    compute = ccompute;
    outlined = coutl;
    nan_catch = false;
    require_newton_on = true;
    require_tag = true;
    require_q = false;
    name_class = GetClassName(cname);
    name_file = GetFileName(cname);
    name_file_cap = Capitalize(name_file);
    out_header = fopen((name_file + ".h").c_str(), "w");
    assert(out_header);
    out_impl = fopen((name_file + ".cpp").c_str(), "w");
    assert(out_impl);
    c_impl.out = out_impl;
    c_header.out = out_header;
    cb_impl = &c_impl;
    cb_header = &c_header;
    COEFF_OUT_LINE = R"(@
      @const double* coeff = &spline_coeffs_@name@@loop@@
      @@iftype@[arg_@idx@]@/@@ifcont@[idx_@idx@]@/@@/@[0];
    )";
    COEFF_IN_LINE = R"(@
      @@full_indent@double the_coeff = coeff[@loopi@@ificont@@
      @@loopj@@ifjcont@@ifjidx<iidx@(@order@ + 1) * @/@@/@@/@p_@iidx@ + @/@@/@0];
    )";
    SPLINE_REGULAR_IDX = R"(@
    @arg_@idx@ = fmax(spline_limits_@idx@_@name@[0], fmin(spline_limits_@idx@_@name@@
    @[@size@ - 1], arg_@idx@));
    int idx_@idx@ = 0;
    for (int i = 0; i < @size@ - 1; i++) {
      if (arg_@idx@ >= spline_limits_@idx@_@name@[i] && @
      @arg_@idx@ <= spline_limits_@idx@_@name@[i+1])
        idx_@idx@ = i;
    }
    )";

  }
  virtual CodeBuilder* WriteHeaderForwardDecls() { return cb_header; }
  virtual CodeBuilder* WriteHeaderParam(int arity, const std::string& name,
                                        const std::string& type, bool gets_modified = false) {
    cb_header->Raw("  %s ", type.c_str());
    assert(arity <= MAX_ARITY &&
           "Only up to 5d parameters are currently supported.");
    for (int i = 0; i < arity; i++) {
      cb_header->Raw("*");
    }
    return cb_header->Raw("%s;\n", name.c_str());
  }

  struct Array {
    int arity;
    std::string type;
    std::string name;
    std::vector<std::string> dims;
    std::string guard;
    bool gets_modified;
  };
  std::vector<Array> arrays;
  void CollectArraysSpline(ParameterDecl* param) {
    int num_args = param->GetArguments().size();
    SplineParameterSpec* spec =
        static_cast<SplineParameterSpec*>(param->spec.get());
    std::string guard = "spline_allocated_" + param->GetName();
    for (int i = 0; i < num_args; i++) {
      if (param->GetArgumentType(i) == Type::kAtomType) continue;
      if ((!spec->is_grid) && (!spec->is_integer)) {
        std::vector<std::string> dims;
        dims.push_back("that->" + SplineSize(param, i));
        arrays.push_back({1, "double", "spline_limits_" + std::to_string(i) +
                                           "_" + param->GetName(),
                          dims, guard, false});
      }
    }
    std::vector<std::string> dims;
    for (int i = 0; i < num_args; i++) {
      dims.push_back("that->" + SplineSize(param, i));
    }
    dims.push_back(std::to_string(1 + spec->derivatives.size()));
    arrays.push_back({1 + num_args, "double",
                      "spline_nodes_" + param->GetName(), dims, guard, false});
    dims.pop_back();
    dims.push_back(std::to_string(
        potc::gen::PowInt(spec->order + 1, param->GetNumContinuousArgs())));
    arrays.push_back({1 + num_args, "double",
                      "spline_coeffs_" + param->GetName(), dims, guard, false});
  }
  void CollectArrays() {
    for (auto param : def->GetParameterDecls()) {
      if (param->spec->GetKind() == ParameterSpec::kSpline) {
        CollectArraysSpline(param);
        continue;
      }
      std::vector<std::string> dims;
      for (int i = 0; i < param->GetNumArgs(); i++) {
        dims.push_back("ntypes+1");
      }
      arrays.push_back({param->GetNumArgs(), "double",
                        "param_" + param->GetName(), dims, "allocated", false});
    }
    int outlined_num = 0;
    for (auto& param : *outlined) {
      std::vector<std::string> dims;
      for (int i = 0; i < param.first; i++) {
        dims.push_back("ntypes+1");
      }
      arrays.push_back({param.first, "double",
                        "outlined_param_" + std::to_string(outlined_num++),
                        dims, "allocated", false});
    }
    num_peratom = 0;
    for (auto decl : def->GetDecls()) {
      if (decl->GetKind() != Decl::kPerAtom) continue;
      std::vector<std::string> dims;
      dims.push_back("nmax");
      PerAtomDecl* per_decl = static_cast<PerAtomDecl*>(decl);
      arrays.push_back(
          {1, "double", "peratom_" + per_decl->GetName(), dims, "nmax", true});
      arrays.push_back({1, "double", "peratom_adjoint_" + per_decl->GetName(),
                        dims, "nmax", true});
      num_peratom += 1;
    }
    std::vector<std::string> dims;
    dims.push_back("ntypes+1");
    arrays.push_back({1, "int", "type_map", dims, "allocated", false});
  }
  virtual void DeclArrays() {
    for (auto& array : arrays) {
      WriteHeaderParam(array.arity, array.name, array.type, array.gets_modified);
    }
  }
  virtual void AllocateArrays(std::string guard) {
    for (auto& array : arrays) {
      if (array.arity == 0) continue;
      if (array.guard == guard) {
        cb_impl->Raw("    memory->create(%s, ", array.name.c_str());
        for (auto& dim : array.dims) {
          cb_impl->Raw("%s, ", dim.c_str());
        }
        cb_impl->Raw("\"pair:%s\");\n", array.name.c_str());
      }
    }
  }
  virtual void DeallocateArrays(std::string guard = "") {
    for (auto& array : arrays) {
      if (array.arity == 0) continue;
      if (guard != "" && guard != array.guard) continue;
      if (array.guard == "") {
        cb_impl->Raw("  if (1) {\n");
      } else {
        cb_impl->Raw("  if (%s) {\n", array.guard.c_str());
      }
      cb_impl->Raw("    memory->destroy(%s);\n  }\n", array.name.c_str());
    }
  }

  inline void WriteSplineHeader(CodeBuilder* cb_header, ParameterDecl* param) {
    // Need: Nodal values, limit values, coefficients, evaluation functions,
    // fitting functions
    int num_args = param->GetArguments().size();
    SplineParameterSpec* spec =
        static_cast<SplineParameterSpec*>(param->spec.get());
    cb_header->Raw("  int spline_allocated_%s;\n", param->GetName().c_str());
    for (int i = 0; i < num_args; i++) {
      cb_header->Raw("  int %s;\n", SplineSize(param, i).c_str());
      if (param->GetArgumentType(i) == Type::kAtomType) continue;
      if (spec->is_grid) {
        WriteHeaderParam(
            0, "spline_low_" + std::to_string(i) + "_" + param->GetName(),
            "double");
        WriteHeaderParam(
            0, "spline_step_" + std::to_string(i) + "_" + param->GetName(),
            "double");
        WriteHeaderParam(
            0, "spline_step_inv_" + std::to_string(i) + "_" + param->GetName(),
            "double");
      } else if (!spec->is_integer) {
      }
    }
    cb_header->Raw("  %s double spline_eval_%s(", EVAL_MODIFIER.c_str(), param->GetName().c_str());
    WriteSplineEvalArgs(cb_header, param);
    cb_header->Raw(") const;\n");
    cb_header->Raw("  void spline_fit_%s();\n\n", param->GetName().c_str());
  }

inline void WriteSplineEval(CodeBuilder* cb_impl, const std::string& name_class,
                            ParameterDecl* param) {
  int num_args = param->GetArguments().size();
  auto name_str = param->GetName();
  const char* name = name_str.c_str();
  SplineParameterSpec* spec =
      static_cast<SplineParameterSpec*>(param->spec.get());
  WriteFunctionImpl(EVAL_MODIFIER + " double ");
  cb_impl->Raw("spline_eval_%s(", 
               param->GetName().c_str());
  WriteSplineEvalArgs(cb_impl, param);
  cb_impl->Raw(") const {\n");
  // for each arg: limit to range, then find correct "section"
  std::vector<int> cont_args;
  for (int i = 0; i < num_args; i++) {
    if (param->GetArgumentType(i) == Type::kAtomType) continue;
    cont_args.push_back(i);
  }


  //auto str = R"(
  //  @loop@@ifcont@@
  //  @@ifinteger@@
  //  @arg_@idx@ = fmax(0, fmin(@size@ - 1, arg_@idx@));
  //  int idx_@idx@ = floor(arg_@idx@);
  //  if (idx_@idx@ == @size@ - 1) idx_@idx@ -= 1;
  //  @/@@ifgrid@@
  //  @arg_@idx@ = fmax(spline_low_@idx@_@name@, fmin(spline_low_@idx@_@name@ + @
  //  @spline_step_@idx@_@name@ * (@size@ - 1), arg_@idx@));

  auto str = R"(
    @loop@@ifcont@@
    @@ifinteger@@
    @if (arg_@idx@ < 0)
      arg_@idx@ = 0;
    if (arg_@idx@ > @size@ - 1)
      arg_@idx@ = @size@ - 1;
    int idx_@idx@ = floor(arg_@idx@);
    if (idx_@idx@ == @size@ - 1) idx_@idx@ -= 1;
    @/@@ifgrid@@
    @if (arg_@idx@ < spline_low_@idx@_@name@)
      arg_@idx@ = spline_low_@idx@_@name@;
    if (arg_@idx@ > spline_low_@idx@_@name@ + @
    @spline_step_@idx@_@name@ * (@size@ - 1))
      arg_@idx@ = spline_low_@idx@_@name@ + @
      @spline_step_@idx@_@name@ * (@size@ - 1);
    int idx_@idx@ = floor((arg_@idx@ - spline_low_@idx@_@name@) * @
    @spline_step_inv_@idx@_@name@);
    if (idx_@idx@ == @size@ - 1) idx_@idx@ -= 1;
    @/@@ifregular@)" + SPLINE_REGULAR_IDX + "    @/@@/@@/@" + COEFF_OUT_LINE + R"(
    double result = 0;
    @loop@@ifcont@@
    @double dt_@idx@ = 0;
    @/@@/@@loop@@ifcont@@
    @@indent@double x_@idx@ = 1;
    @indent@double dx_@idx@ = 0;
    @indent@for (int p_@idx@ = 0; p_@idx@ <= @order@; p_@idx@++) {
    @/@@/@)" + COEFF_IN_LINE + R"(
    @full_indent@result += the_coeff@loop@@ifcont@ * x_@idx@@/@@/@;
    @loop@@ifcont@@
    @@full_indent@dt_@idx@ += p_@idx@ * the_coeff@
    @@loopj@@ifjcont@ * @ifidx=jidx@d@/@x_@jidx@@/@@/@;
    @/@@/@@
    @@rloop@@ifcont@@
    @@indent@  dx_@idx@ = x_@idx@;
    @indent@  x_@idx@ *= arg_@idx@;
    @indent@}
    @/@@/@@
    @@loop@@ifcont@@
    @*d_@idx@ = dt_@idx@;
    @/@@/@@
    @return result;
    )";
  std::map<std::string, std::string> args;
  args["name"] = param->GetName();
  args["order"] = std::to_string(spec->order);
  std::vector<std::map<std::string, std::string>> loop_args;
  std::string indent = "";
  for (int i = 0; i < num_args; i++) {
    loop_args.push_back({});
    loop_args[i]["indent"] = indent;
    loop_args[i]["idx"] = std::to_string(i);
    loop_args[i]["size"] = SplineSize(param, i);
    loop_args[i]["regular"] = !spec->is_integer && !spec->is_grid ? "true" : "";
    loop_args[i]["integer"] = spec->is_integer ? "true" : "";
    loop_args[i]["grid"] = spec->is_grid ? "true" : "";
    if (param->GetArgumentType(i) == Type::kAtomType) {
      loop_args[i]["cont"] = "";
      loop_args[i]["type"] = "true";
    } else {
      loop_args[i]["cont"] = "true";
      loop_args[i]["type"] = "";
      indent += "  ";
    }
  }
  args["full_indent"] = indent;
  cb_impl->Fragment(Indent(FormatTemplate(Undent(str), args, loop_args)));
  // for (auto i : cont_args) {
  //  if (spec->is_integer) {
  //    cb_impl->Raw("  arg_%1$d = fmax(0, fmin(%3$s - 1, arg_%1$d));\n", i,
  //    name,
  //                 SplineSize(param, i).c_str());
  //    cb_impl->Raw("  int idx_%1$d = floor(arg_%1$d);\n", i);
  //    cb_impl->Raw("  if (idx_%1$d == %2$s - 1) idx_%1$d -= 1;\n", i,
  //                 SplineSize(param, i).c_str());
  //  } else if (spec->is_grid) {
  //    cb_impl->Raw(
  //        "  arg_%1$d = fmax(spline_low_%1$d_%2$s, fmin(spline_low_%1$d_%2$s +
  //        "
  //        "spline_step_%1$d_%2$s * (%3$s - 1), arg_%1$d));\n",
  //        i, name, SplineSize(param, i).c_str());
  //    cb_impl->Raw(
  //        "  int idx_%1$d = floor((arg_%1$d - spline_low_%1$d_%2$s) * "
  //        "spline_step_inv_%1$d_%2$s);\n",
  //        i, name);
  //    cb_impl->Raw("  if (idx_%1$d == %2$s - 1) idx_%1$d -= 1;\n", i,
  //                 SplineSize(param, i).c_str());
  //  } else {
  //    cb_impl->Raw(
  //        "  arg_%1$d = fmax(spline_limits_%1$d_%2$s[0], "
  //        "fmin(spline_limits_%1$d_%2$s[%3$s - 1], arg_%1$d));\n",
  //        i, name, SplineSize(param, i).c_str());
  //    cb_impl->Raw("  int idx_%d = 0;\n", i);
  //    cb_impl->Raw("  for (int i = 0; i < %s - 1; i++) {\n",
  //                 SplineSize(param, i).c_str());
  //    cb_impl->Raw(
  //        "    if (arg_%d >= spline_limits_%d_%s[i] && arg_%d <= "
  //        "spline_limits_%d_%s[i+1])\n",
  //        i, i, name, i, i, name);
  //    cb_impl->Raw("      idx_%d = i;\n", i);
  //    cb_impl->Raw("  }\n");
  //  }
  //}
  // cb_impl->Raw("  double* coeff = &spline_coeffs_%s", name);
  // for (int i = 0; i < num_args; i++) {
  //  if (param->GetArgumentType(i) == Type::kAtomType) {
  //    cb_impl->Raw("[arg_%d]", i);
  //  } else {
  //    cb_impl->Raw("[idx_%d]", i);
  //  }
  //}
  // cb_impl->Raw("[0];\n");
  // cb_impl->Raw("  double result = 0;\n");
  // for (int i : cont_args) {
  //  cb_impl->Raw("  double dt_%d = 0;\n", i);
  //}
  // std::string indent = "";
  // for (int i : cont_args) {
  //  cb_impl->Raw("  %sdouble x_%d = 1;\n", indent.c_str(), i);
  //  cb_impl->Raw("  %sdouble dx_%d = 0;\n", indent.c_str(), i);
  //  cb_impl->Raw("  %sfor (int p_%d = 0; p_%d <= %d; p_%d++) {\n",
  //               indent.c_str(), i, i, spec->order, i);
  //  indent += "  ";
  //}
  // cb_impl->Raw("  %sdouble the_coeff = coeff[", indent.c_str());
  // for (int i : cont_args) {
  //  for (int j : cont_args) {
  //    if (j >= i) continue;
  //    cb_impl->Raw("%d * ", spec->order + 1);
  //  }
  //  cb_impl->Raw("p_%d + ", i);
  //}
  // cb_impl->Raw("0];\n");
  // cb_impl->Raw("  %sresult += the_coeff", indent.c_str());
  // for (int i : cont_args) {
  //  cb_impl->Raw(" * x_%d", i);
  //}
  // cb_impl->Raw(";\n");
  // for (int i : cont_args) {
  //  cb_impl->Raw("  %sdt_%d += p_%d * the_coeff", indent.c_str(), i, i);
  //  for (int j : cont_args) {
  //    cb_impl->Raw(" * %sx_%d", i == j ? "d" : "", j);
  //  }
  //  cb_impl->Raw(";\n");
  //}
  // for (int i : cont_args) {
  //  cb_impl->Raw("  %sdx_%d = x_%d;\n", indent.c_str(), i, i);
  //  cb_impl->Raw("  %sx_%d *= arg_%d;\n", indent.c_str(), i, i);
  //  indent = indent.substr(2);
  //  cb_impl->Raw("  %s}\n", indent.c_str());
  //}
  // for (int i : cont_args) {
  //  cb_impl->Raw("  *d_%d = dt_%d;\n", i, i);
  //}
  // cb_impl->Raw("  return result;\n");
  cb_impl->Raw("}\n");
  // select correct coefficients
  // evaluate with the coefficients
}

inline void WriteSplineInit(CodeBuilder* cb_impl, const std::string& name_class,
                            ParameterDecl* param) {
  WriteFunctionImpl("void ");
  cb_impl->Raw("spline_fit_%s() {\n",
               param->GetName().c_str());
  // derive the equations for the spline
  // essentially symbolically evaluate a polynomia of <order> at the 2^dim nodes
  // of a patch
  // also evaluate its derivatives there (the ones given in the definition)
  // TODO(markus): How do you fix the other derivatives if not enough are given
  //   want: to set these to zero, but is it always possible to find a set that
  //   does not overconstrain us?
  // uncode as operations against coefficients
  // do the patch adjustment
  int num_args = param->GetArguments().size();
  SplineParameterSpec* spec =
      static_cast<SplineParameterSpec*>(param->spec.get());
  auto name_str = param->GetName();
  auto name = name_str.c_str();

  cb_impl->Raw("  const int binomial[%d][%d] = {\n", spec->order + 1,
               spec->order + 1);
  for (int i = 0; i < spec->order + 1; i++) {
    cb_impl->Raw("    { ");
    for (int j = 0; j <= i; j++) {
      int value = 1;
      for (int k = 1; k <= i; k++) {
        value *= k;
      }
      for (int k = 1; k <= j; k++) {
        value /= k;
      }
      for (int k = 1; k <= i - j; k++) {
        value /= k;
      }
      cb_impl->Raw("%d, ", value);
    }
    cb_impl->Raw("},\n");
  }
  cb_impl->Raw("  };\n");

  std::string indent = "";
  for (int i = 0; i < num_args; i++) {
    if (param->GetArgumentType(i) == Type::kAtomType) {
      cb_impl->Raw("  %1$sfor (int i_%2$d = 1; i_%2$d < %3$s; i_%2$d++) {\n",
                   indent.c_str(), i, SplineSize(param, i).c_str());
    } else {
      cb_impl->Raw(
          "  %1$sfor (int i_%2$d = 0; i_%2$d < %3$s - 1; i_%2$d++) {\n",
          indent.c_str(), i, SplineSize(param, i).c_str());
    }
    indent += "  ";
  }

  for (int i = 0; i < num_args; i++) {
    if (param->GetArgumentType(i) == Type::kAtomType) continue;
    if (spec->is_grid) {
      cb_impl->Raw("  %3$sdouble delta_%1$d = spline_step_%1$d_%2$s;\n", i,
                   name, indent.c_str());
      cb_impl->Raw(
          "  %3$sdouble lo_%1$d = spline_low_%1$d_%2$s + i_%1$d * "
          "delta_%1$d;\n",
          i, name, indent.c_str());
    } else if (spec->is_integer) {
      cb_impl->Raw("  %2$sdouble delta_%1$d = 1;\n", i, indent.c_str());
      cb_impl->Raw("  %2$sdouble lo_%1$d = i_%1$d;\n", i, indent.c_str());
    } else {
      cb_impl->Raw("  %sdouble lo_%d = spline_limits_%d_%s[i_%d];\n",
                   indent.c_str(), i, i, name, i);
      cb_impl->Raw("  %sdouble hi_%d = spline_limits_%d_%s[i_%d+1];\n",
                   indent.c_str(), i, i, name, i);
      cb_impl->Raw("  %sdouble delta_%d = hi_%d - lo_%d;\n", indent.c_str(), i,
                   i, i);
    }
  }
  cb_impl->Raw("  %sdouble* coeff = &spline_coeffs_%s", indent.c_str(), name);
  for (int i = 0; i < num_args; i++) {
    cb_impl->Raw("[i_%d]", i);
  }
  cb_impl->Raw("[0];\n");

  int num_cont_args = param->GetNumContinuousArgs();
  std::vector<std::vector<int>> adjusted_derivatives = {{}};
  adjusted_derivatives.insert(adjusted_derivatives.end(),
                              spec->derivatives.begin(),
                              spec->derivatives.end());
  std::vector<int> to_adjusted(num_args), from_adjusted(num_cont_args);
  int adjusted = 0;
  for (int i = 0; i < num_args; i++) {
    if (param->GetArgumentType(i) == Type::kAtomType) continue;
    to_adjusted[i] = adjusted;
    from_adjusted[adjusted] = i;
    adjusted += 1;
  }
  for (auto& der : adjusted_derivatives) {
    for (auto& elem : der) {
      // elem -= 1;
      elem = to_adjusted[elem - 1];
    }
  }

  auto coeffs = potc::gen::GetSplineFittingMatrix(spec->order, num_cont_args,
                                                  adjusted_derivatives);

  for (int coeff_idx = 0; coeff_idx < coeffs[0].size(); coeff_idx++) {
    auto& coeff_num = coeffs[0][coeff_idx];
    auto& coeff_den = coeffs[1][coeff_idx];
    assert(coeff_num.size() == coeff_den.size());
    assert(coeffs[2].size() == coeff_den.size());

    cb_impl->Raw("  %scoeff[%d] = 0", indent.c_str(), coeff_idx);
    for (int order_idx = 0; order_idx < coeffs[2].size(); order_idx++) {
      auto& ordering = coeffs[2][order_idx];
      if (coeff_num[order_idx] == 0) continue;

      cb_impl->Raw(" + %d * spline_nodes_%s", coeff_num[order_idx], name);

      assert(ordering.size() == num_cont_args + 1);
      int kk = 0;
      for (int k = 0; k < num_args; k++) {
        if (param->GetArgumentType(k) == Type::kAtomType) {
          cb_impl->Raw("[i_%d]", k);
        } else {
          cb_impl->Raw("[i_%d+%d]", k, ordering[kk++]);
        }
      }
      cb_impl->Raw("[%d]", ordering[num_cont_args]);

      if (coeff_den[order_idx] != 1) {
        cb_impl->Raw(" / %d", coeff_den[order_idx]);
      }
      for (auto adjustment : adjusted_derivatives[ordering[num_cont_args]]) {
        cb_impl->Raw(" * delta_%d", from_adjusted[adjustment]);
      }
    }
    cb_impl->Raw(";\n");
  }

  // test if spline edges match up with 0/1 boundaries

  for (int adjust_arg_idx = 0; adjust_arg_idx < num_args; adjust_arg_idx++) {
    if (param->GetArgumentType(adjust_arg_idx) == Type::kAtomType) continue;
    cb_impl->Raw("\n");

    for (int i = 0; i < num_args; i++) {
      if (param->GetArgumentType(i) == Type::kAtomType) continue;
      cb_impl->Raw("  %sfor (int j_%d = 0; j_%d <= %d; j_%d++) {\n",
                   indent.c_str(), i, i, spec->order, i);
      indent += "  ";
    }

    cb_impl->Raw("  %sdouble new_coeff = 0.0;\n", indent.c_str());
    cb_impl->Raw("  %sfor (int k = j_%d; k <= %d; k++) {\n", indent.c_str(),
                 adjust_arg_idx, spec->order);
    cb_impl->Raw(
        "    %snew_coeff += pow(delta_%d, -k) * pow(-lo_%d, k - j_%d) * "
        "binomial[k][j_%d] * coeff[0",
        indent.c_str(), adjust_arg_idx, adjust_arg_idx, adjust_arg_idx,
        adjust_arg_idx);
    int ii = 0;
    for (int i = 0; i < num_args; i++) {
      if (param->GetArgumentType(i) == Type::kAtomType) continue;
      if (i == adjust_arg_idx) {
        cb_impl->Raw(" + %d * k", potc::gen::PowInt(spec->order + 1, ii++));
      } else {
        cb_impl->Raw(" + %d * j_%d", potc::gen::PowInt(spec->order + 1, ii++),
                     i);
      }
    }
    cb_impl->Raw("];\n");
    cb_impl->Raw("  %s}\n", indent.c_str());

    cb_impl->Raw("  %scoeff[0", indent.c_str());
    ii = 0;
    for (int i = 0; i < num_args; i++) {
      if (param->GetArgumentType(i) == Type::kAtomType) continue;
      cb_impl->Raw(" + %d * j_%d", potc::gen::PowInt(spec->order + 1, ii++), i);
    }
    cb_impl->Raw("] = new_coeff;\n");

    for (int i = 0; i < num_args; i++) {
      if (param->GetArgumentType(i) == Type::kAtomType) continue;
      indent = indent.substr(2);
      cb_impl->Raw("  %s}\n", indent.c_str());
    }
  }

  for (int i = 0; i < num_args; i++) {
    indent = indent.substr(2);
    cb_impl->Raw("  %s}\n", indent.c_str());
  }
  cb_impl->Raw("}\n");
  // for each "patch"
  //   fill an array with all the relevant values and derivatives
}

  virtual void WritePackUnpackDecl() {
    cb_header->Raw("  int nmax;\n");
    cb_header->Raw(
        "  int pack_forward_comm(int, int*, double*, int, int*);\n");
    cb_header->Raw("  void unpack_forward_comm(int, int, double*);\n");
    cb_header->Raw("  int pack_reverse_comm(int, int, double*);\n");
    cb_header->Raw("  void unpack_reverse_comm(int, int*, double*);\n");
  }
  virtual CodeBuilder* WriteHeaderClassContent() {
    cb_header->Raw("  %s(class LAMMPS *);\n", name_class.c_str());
    cb_header->Raw("  ~%s();\n", name_class.c_str());
    cb_header->Line("  void compute(int, int);");
    cb_header->Line("  void settings(int, char**);");
    cb_header->Line("  void coeff(int, char**);");
    cb_header->Line("  double init_one(int, int);");
    cb_header->Line("  void init_style();");
    cb_header->Line("  virtual void allocate();");
    cb_header->Line(
        "  virtual void file_process_line(int narg, char **arg, char "
        "**coeff_arg);\n");
    cb_header->Line("  double cutmax;\n");
    //WriteHeaderParam(1, "type_map", "int");
    if (def->types.size() > 0) {
      cb_header->Raw("  enum { ");
      for (auto type : def->types)
        cb_header->Raw("type_var_%s, ", type.c_str());
      cb_header->Raw(" };\n\n");
    }
    DeclArrays();
    cb_header->Raw("\n");
    for (auto param : def->GetParameterDecls()) {
      if (param->spec->GetKind() != ParameterSpec::kSpline) continue;
      WriteSplineHeader(cb_header, param);
    }

    if (num_peratom) cb_header->Raw("  int stage_peratom;\n");
    if (num_peratom > 0) {
      WritePackUnpackDecl();
    }
    return cb_header;
  }
  virtual CodeBuilder* WriteHeaderPairStyle() {
    return cb_header->Raw("PairStyle(%s, %s)\n", name.c_str(),
                          name_class.c_str());
  }
  virtual CodeBuilder* WriteHeaderClass() {
    return cb_header->Raw("class %s : public Pair {\n", name_class.c_str());
  }
  virtual CodeBuilder* WriteHeaderIncludes() {
    return cb_header->Line("#include \"pair.h\"");
  }
  virtual void WriteHeader() {
    cb_header->Raw("%s", LICENSE_BLOB);
    cb_header->Raw("#ifdef PAIR_CLASS\n\n");
    WriteHeaderPairStyle();
    cb_header->Raw("\n#else\n\n");
    cb_header->Raw("#ifndef LMP_%s_H\n", name_file_cap.c_str());
    cb_header->Raw("#define LMP_%s_H LMP_%s_H\n\n", name_file_cap.c_str(),
                   name_file_cap.c_str());
    WriteHeaderIncludes();
    cb_header->Raw("namespace LAMMPS_NS {\n\n");
    WriteHeaderForwardDecls();
    WriteHeaderClass();
    cb_header->Raw(" public:\n");
    WriteHeaderClassContent();
    cb_header->Raw("};\n\n");
    cb_header->Raw("}  // namespace LAMMPS_NS\n\n");
    cb_header->Raw("#endif  // LMP_%s_H\n\n", name_file_cap.c_str());
    cb_header->Raw("#endif  // PAIR_CLASS\n");
  }

  virtual CodeBuilder* WriteImplStart() {
    cb_impl->Raw("%s", LICENSE_BLOB);
    cb_impl->Raw("#include \"%s.h\"\n", name_file.c_str());
    cb_impl->Raw("#include <math.h>\n");  // TODO(markus) check if all needed
    cb_impl->Raw("#include <stdio.h>\n");
    cb_impl->Raw("#include <stdlib.h>\n");
    cb_impl->Raw("#include <string.h>\n");
    cb_impl->Raw("#include <fenv.h>\n");
    cb_impl->Raw("#include \"atom.h\"\n");
    cb_impl->Raw("#include \"comm.h\"\n");
    cb_impl->Raw("#include \"force.h\"\n");
    cb_impl->Raw("#include \"neighbor.h\"\n");
    cb_impl->Raw("#include \"neigh_list.h\"\n");
    cb_impl->Raw("#include \"neigh_request.h\"\n");
    cb_impl->Raw("#include \"memory.h\"\n");
    cb_impl->Raw("#include \"error.h\"\n");
    cb_impl->Raw("#include \"math_const.h\"\n\n");
    cb_impl->Raw("using namespace LAMMPS_NS;\n");
    cb_impl->Raw("using namespace MathConst;\n");
    return cb_impl;
  }

  virtual CodeBuilder* WriteFunctionImpl(const std::string& prefix) {
    cb_impl->Raw("%s\n", LINE);
    cb_impl->Raw("%s%s::", prefix.c_str(), name_class.c_str());
    return cb_impl;
  }

  virtual CodeBuilder* WriteConstructor() {
    WriteFunctionImpl("");
    cb_impl->Raw("%s(LAMMPS *lmp) : Pair(lmp) {\n", name_class.c_str());
    cb_impl->Raw("  manybody_flag = 1;\n");
    cb_impl->Raw("  one_coeff = 1;\n");
    cb_impl->Raw("  single_enable = 0;\n");
    cb_impl->Raw("  restartinfo = 0;\n");
    if (ana_neigh.need_ghost) {
      cb_impl->Raw("  ghostneigh = 1;\n");
    }
    for (auto& array : arrays) {
      if (array.arity == 0) continue;
      cb_impl->Raw("  %s = NULL;\n", array.name.c_str());
      if (array.guard != "") cb_impl->Raw("  %s = 0;\n", array.guard.c_str());
    }
    if (num_peratom > 0) {
      cb_impl->Raw("  nmax = 0;\n");
      cb_impl->Raw("\n  comm_forward = %d;\n  comm_reverse = %d;\n",
                   num_peratom, num_peratom);
    }
    return cb_impl->Indent();
  }

  virtual CodeBuilder* WriteDestructor() {
    WriteFunctionImpl("");
    cb_impl->Raw("~%s() {\n", name_class.c_str())->Indent();
    cb_impl->Line("if (copymode) return;");
    cb_impl->If("allocated");
    cb_impl->Line("memory->destroy(setflag);");
    cb_impl->Line("memory->destroy(cutsq);\n");
    //cb_impl->Line("memory->destroy(type_map);\n");
    if (ana_neigh.need_ghost) {
      cb_impl->Line("memory->destroy(cutghost);\n");
    }
    cb_impl->End();
    DeallocateArrays();
    return cb_impl;
  }

  virtual CodeBuilder* WriteCompute() {
    WriteFunctionImpl("void ");
    cb_impl->Raw("compute(int eflag, int vflag) {\n");
    if (nan_catch) {
      cb_impl->Raw(
          "  feenableexcept(FE_INVALID | FE_DIVBYZERO | FE_OVERFLOW);\n");
    }
    cb_impl->Raw("  if (eflag || vflag) ev_setup(eflag,vflag);\n");
    cb_impl->Raw("  else evflag = vflag_fdotr = 0;\n\n");
    cb_impl->Raw("  double **x = atom->x;\n");
    cb_impl->Raw("  double **f = atom->f;\n");
    cb_impl->Raw("  double *q = atom->q;\n");
    cb_impl->Raw("  int *type = atom->type;\n");
    cb_impl->Raw("  int nlocal = atom->nlocal;\n");
    cb_impl->Raw("  int newton_pair = force->newton_pair;\n");
    // TODO(markus) actually respect inum and ilist
    cb_impl->Raw("  int inum = list->inum;\n");
    cb_impl->Raw("  int *ilist = list->ilist;\n");
    if (require_tag) {
      cb_impl->Raw("  tagint *tag = atom->tag;\n");
    }
    cb_impl->Raw("  int *numneigh = list->numneigh;\n");
    cb_impl->Raw("  int **firstneigh = list->firstneigh;\n");
    if (num_peratom > 0) {
      cb_impl->Raw("  if (atom->nmax > nmax) {\n");
      cb_impl->Raw("    nmax = atom->nmax;\n");
      DeallocateArrays("nmax");
      AllocateArrays("nmax");
      cb_impl->Raw("  }\n");
    }
    WriteRegularVisitor vis(cb_impl, &ana_comm);
    vis.neigh_is_half = !ana_neigh.need_full;
    vis.Visit(compute);
    cb_impl->Raw("\n  if (vflag_fdotr) virial_fdotr_compute();\n");
    if (nan_catch) {
      cb_impl->Raw("  fesetenv(FE_DFL_ENV);\n");
    }
    // cb_impl->Raw("}\n");
    return cb_impl->Indent();
  }

  virtual CodeBuilder* WriteInitStyle() {
    WriteFunctionImpl("void ");
    cb_impl->Raw("init_style() {\n");
    if (require_newton_on) {
      cb_impl->Raw("  if (force->newton_pair == 0)\n");
      cb_impl->Raw(
          "    error->all(FLERR, \"Pair style %s requires newton pair on\");\n",
          name.c_str());
    }
    if (require_tag) {
      cb_impl->Raw("  if (atom->tag_enable == 0)\n");
      cb_impl->Raw(
          "    error->all(FLERR, \"Pair style %s requires atom ID's\");\n",
          name.c_str());
    }
    if (require_q) {
      cb_impl->Raw("  if (! atom->q_flag)\n");
      cb_impl->Raw(
          "    error->all(FLERR, \"Pair style %s requires atom attribute "
          "q\");\n",
          name.c_str());
    }
    cb_impl->Raw("  int irequest = neighbor->request(this, instance_me);\n");
    if (ana_neigh.need_full) {
      cb_impl->Raw("  neighbor->requests[irequest]->half = 0;\n");
      cb_impl->Raw("  neighbor->requests[irequest]->full = 1;\n");
    } else {
      cb_impl->Raw("  neighbor->requests[irequest]->half = 1;\n");
      cb_impl->Raw("  neighbor->requests[irequest]->full = 0;\n");
    }
    if (ana_neigh.need_ghost) {
      cb_impl->Raw("  neighbor->requests[irequest]->ghost = 1;\n");
    }
    // cb_impl->Raw("}\n");
    return cb_impl->Indent();
  }

  virtual CodeBuilder* WriteAllocate() {
    WriteFunctionImpl("void ");
    cb_impl->Raw("allocate() {\n");
    cb_impl->Indent();
    cb_impl->Raw("  allocated = 1;\n");
    cb_impl->Raw("  int n = atom->ntypes;\n\n");
    cb_impl->Raw("  memory->create(setflag, n+1, n+1, \"pair:setflag\");\n");
    //cb_impl->Raw("  memory->create(type_map, n+1, \"pair:typemap\");\n");
    cb_impl->Raw("  for (int i = 1; i <= n; i++)\n");
    cb_impl->Raw("    for (int j = i; j <= n; j++)\n");
    cb_impl->Raw("      setflag[i][j] = 0;\n\n");
    cb_impl->Raw("  memory->create(cutsq, n+1, n+1, \"pair:cut\");\n\n");
    if (ana_neigh.need_ghost) {
      cb_impl->Raw(
          "  memory->create(cutghost, n+1, n+1, \"pair:cutghost\");\n\n");
    }
    cb_impl->Raw("  int ntypes = n;\n");
    AllocateArrays("allocated");
    return cb_impl;
  }

  virtual CodeBuilder* WriteInitOne() {
    WriteFunctionImpl("double ");
    cb_impl->Raw("init_one(int i, int j) {\n");
    cb_impl->Raw(
        "  if (setflag[i][j] == 0) error->all(FLERR, \"All pair coeffs are "
        "not set\");\n");
    if (ana_neigh.need_ghost) {
      cb_impl->Raw("  cutghost[i][j] = cutmax;\n");
      cb_impl->Raw("  cutghost[j][i] = cutmax;\n");
    }
    cb_impl->Raw("  return %d * cutmax;\n", ana_neigh.max_num_skips);
    // cb_impl->Raw("}\n");
    return cb_impl->Indent();
  }

  void WriteSplineAlloc(ParameterDecl* param, SplineParameterSpec* spec) {
    int num_args = param->GetArguments().size();
    auto name_str = param->GetName();
    auto name = name_str.c_str();
    // if (spec->is_grid) {
    //} else {
    //  for (int i = 0; i < num_args; i++) {
    //    cb_impl->Raw("  spline_size_%d_%s = %d;\n", i, name,
    //    spec->nodes_along[i]);
    //  }
    //}
    cb_impl->Indent()->Indent()->Indent()->Indent();
    if (!spec->is_grid && !spec->is_integer) {
      for (int i = 0; i < num_args; i++) {
        cb_impl->RawL(
            "memory->create(spline_limits_%1$d_%2$s, %3$s, "
            "\"pair:spline:limit:%2$s\");\n",
            i, name, SplineSize(param, i).c_str());
      }
    }
    cb_impl->StartFragment()->Raw("memory->create(spline_nodes_%s", name);
    for (int i = 0; i < num_args; i++) {
      cb_impl->Fragment(", ")->Fragment(SplineSize(param, i));
    }
    cb_impl->Raw(", %zu", spec->derivatives.size() + 1);
    cb_impl->Raw(", \"pair:spline:nodes:%s\");", name)->EndFragment();
    cb_impl->StartFragment()->Raw("memory->create(spline_coeffs_%s", name);
    for (int i = 0; i < num_args; i++) {
      cb_impl->Fragment(", ")->Fragment(SplineSize(param, i));
    }
    cb_impl->Raw(", %d", potc::gen::PowInt(spec->order + 1,
                                           param->GetNumContinuousArgs()));
    cb_impl->Raw(", \"pair:spline:coeffs:%s\");", name)->EndFragment();

    if (!spec->is_grid && !spec->is_integer) {
      for (int i = 0; i < num_args; i++) {
        cb_impl->RawL(
            "memset(&spline_limits_%d_%s[0], 0, %s * sizeof(double));\n", i,
            name, SplineSize(param, i).c_str());
      }
    }
    for (int i = 0; i < num_args; i++) {
      cb_impl
          ->RawL("for (int i_%1$d = 0; i_%1$d < %2$s; i_%1$d++) {", i,
                 SplineSize(param, i).c_str())
          ->Indent();
    }
    cb_impl->StartFragment()->Raw("memset(&spline_nodes_%s", name);
    for (int i = 0; i < num_args; i++) {
      cb_impl->Raw("[i_%d]", i);
    }
    cb_impl
        ->Raw("[0], 0, %zu * sizeof(double));\n", 1 + spec->derivatives.size())
        ->EndFragment();
    for (int i = 0; i < num_args; i++) {
      cb_impl->End();
    }
    cb_impl->Undent()->Undent()->Undent()->Undent();
  }

  void WriteFileProcessLineSpline(ParameterDecl* param,
                                  SplineParameterSpec* spec) {
    auto name_str = param->GetName();
    auto name = name_str.c_str();
    cb_impl->Raw(
        "    if (strcmp(arg[0] + strlen(SPLINE_PREFIX), \"%s\") == 0) {\n",
        name);
    int num_args = param->GetArguments().size();
    cb_impl->Raw("      if (strcmp(arg[1], \"size\") == 0) {\n");
    cb_impl->Raw(
        "        if (narg != %d + 2) error->all(FLERR, \"Invalid spline size "
        "definition\");\n",
        param->GetArguments().size());
    for (int i = 0; i < param->GetArguments().size(); i++) {
      if (param->GetArgumentType(i) == Type::kAtomType) {
        cb_impl->Raw("        %s = atom->ntypes + 1;\n",
                     SplineSize(param, i).c_str());
        cb_impl->Raw(
            "        if (strcmp(arg[2 + %d], \"_\") != 0) error->all(FLERR, "
            "\"Invalid spline size definition\");\n",
            i);
      } else {
        cb_impl->Raw("        %1$s = atoi(arg[2 + %2$d]);\n",
                     SplineSize(param, i).c_str(), i);
      }
      // now, allocate everything
    }
    cb_impl->Raw("        spline_allocated_%s = 1;\n", name);
    cb_impl->Raw("        %s *that = this;\n", name_class.c_str());
    
    AllocateArrays("spline_allocated_" + name_str);
    cb_impl->Raw("        return;\n");
    cb_impl->Raw("      }\n");
    for (int i = 0; i < num_args; i++) {
      if (spec->is_grid) continue;
      if (spec->is_integer) continue;
      cb_impl->Raw("      if (strcmp(arg[1], \"limits:%d\") == 0) {\n", i + 1);
      cb_impl->Raw(
          "        if (narg != %s + 2) error->all(FLERR, \"Invalid spline "
          "limit definition\");\n",
          SplineSize(param, i).c_str());
      cb_impl->Raw("        for (int i = 0; i < %s; i++) {\n",
                   SplineSize(param, i).c_str());
      cb_impl->Raw("          spline_limits_%d_%s[i] = atof(arg[i+2]);\n", i,
                   name);
      cb_impl->Raw("        }\n");
      cb_impl->Raw("        return;\n");
      cb_impl->Raw("      }\n");
    }
    cb_impl->Raw(
        "      if (narg != 1 + %d + 1 + %zu) error->all(FLERR, \"Invalid "
        "spline node defintion\");\n",
        num_args, spec->derivatives.size());
    std::string indent = "";
    for (int j = 0; j < num_args; j++) {
      if (param->GetArgumentType(j) != Type::kAtomType) continue;
      cb_impl->Raw(
          "      %2$sfor (int i_%1$d = 1; i_%1$d <= atom->ntypes; i_%d++) {\n",
          j, indent.c_str());
      cb_impl->Raw(
          "      %2$s  if (strcmp(coeff_arg[i_%1$d], arg[1 + %1$d]) != 0) "
          "continue;\n",
          j, indent.c_str());
      indent += "  ";
    }
    for (int j = 0; j < spec->derivatives.size() + 1; j++) {
      cb_impl->Raw("      %sspline_nodes_%s", indent.c_str(), name);
      for (int i = 0; i < num_args; i++) {
        if (param->GetArgumentType(i) == Type::kAtomType) {
          cb_impl->Raw("[i_%d]", i);
        } else {
          cb_impl->Raw("[atoi(arg[1 + %d])]", i);
        }
      }
      cb_impl->Raw("[%d] = atof(arg[1 + %d + %d]);\n", j, num_args, j);
    }
    for (int j = 0; j < num_args; j++) {
      if (param->GetArgumentType(j) != Type::kAtomType) continue;
      indent = indent.substr(2);
      cb_impl->Raw("      %s}\n", indent.c_str());
    }
    cb_impl->Raw("    }\n");
  }
  void WriteFileProcessLine() {
    WriteFunctionImpl("void ");
    cb_impl->Raw(
        "file_process_line(int narg, char **arg, char **coeff_arg) {\n");

    // count number of header elems
    cb_impl->Raw("  if (narg == 0) return;\n");

    cb_impl->Raw("  const char * SPLINE_PREFIX = \"spline:\";\n");
    cb_impl->Raw(
        "  if (strncmp(arg[0], SPLINE_PREFIX, strlen(SPLINE_PREFIX)) == 0) "
        "{\n");
    for (auto param : def->GetParameterDecls()) {
      if (param->spec->GetKind() != ParameterSpec::kSpline) continue;
      SplineParameterSpec* spec =
          static_cast<SplineParameterSpec*>(param->spec.get());
      WriteFileProcessLineSpline(param, spec);
    }
    cb_impl->Raw("    return;\n");
    cb_impl->Raw("  }\n");

    cb_impl->Raw("  int num_header = 0;\n");
    cb_impl->Raw("  while (isalpha(arg[num_header][0])) num_header += 1;\n");
    for (int i = 0; i < 6; i++) {
      bool need_i = false;
      for (auto param : def->GetParameterDecls()) {
        if (param->GetArguments().size() == i &&
            param->spec->GetKind() != ParameterSpec::kSpline)
          need_i = true;
      }
      if (!need_i) continue;
      cb_impl->Raw("  if (%d == num_header) {\n", i);
      std::string indent;
      for (int j = 0; j < i; j++) {
        cb_impl->Raw(
            "%s    for (int i_%d = 1; i_%d <= this->atom->ntypes; i_%d++) {\n",
            indent.c_str(), j, j, j);
        cb_impl->Raw(
            "%s      if (strcmp(coeff_arg[i_%d], arg[%d]) != 0) continue;\n",
            indent.c_str(), j, j);
        indent += "  ";
      }
      for (auto param : def->GetParameterDecls()) {
        if (param->GetArguments().size() != i) continue;
        if (param->spec->GetKind() != ParameterSpec::kFile) continue;
        cb_impl->Raw("%s    this->param_%s", indent.c_str(),
                     param->GetName().c_str());
        for (int j = 0; j < i; j++) {
          cb_impl->Raw("[i_%d]", j);
        }
        cb_impl->Raw(
            " = atof(arg[%d]);\n",
            static_cast<FileParameterSpec*>(param->spec.get())->index + i - 1);
      }
      for (int j = 0; j < i; j++) {
        cb_impl->Raw("%s  }\n", indent.c_str());
        indent = indent.substr(2);
      }
      cb_impl->Raw("    return;\n");
      cb_impl->Raw("  }\n");
    }
    cb_impl->Raw("  error->all(FLERR, \"Could not process file input.\");\n");
    cb_impl->Raw("}\n");
  }

  virtual CodeBuilder* WriteSettings() {
    WriteFunctionImpl("void ");
    cb_impl->Raw("settings(int narg, char **arg) {\n");
    cb_impl->Raw(
        "  if (narg != 1) error->all(FLERR, \"Illegal pair_style "
        "command\");\n");
    cb_impl->Raw("  cutmax = atof(arg[0]);\n");
    cb_impl->Raw("}\n");
    WriteFunctionImpl("void ");
    cb_impl->Raw("coeff(int narg, char **arg) {\n", name_class.c_str());
    cb_impl->Raw("  if (!allocated) allocate();\n\n");

    cb_impl->Raw("  if (narg != 3 + atom->ntypes)\n");
    cb_impl->Raw(
        "    error->all(FLERR,\"Incorrect args for pair coefficients\");\n\n");
    cb_impl->Raw("  // insure I,J args are * *\n\n");
    cb_impl->Raw(
        "  if (strcmp(arg[0], \"*\") != 0 || strcmp(arg[1], \"*\") != 0)\n");
    cb_impl->Raw(
        "    error->all(FLERR, \"Incorrect args for pair coefficients\");\n\n");
    // first, read all zero order params (if exist)
    // second, read in record header
    // third, read in all the parameters
    if (!def->types.empty()) {
      cb_impl->Raw("  for (int i = 3; i < narg; i++) {\n");
      for (auto& type : def->types) {
        cb_impl->Raw("    if (strcmp(arg[i], \"%s\") == 0) {\n", type.c_str());
        cb_impl->Raw("      type_map[i - 2] = type_var_%s;\n", type.c_str());
        cb_impl->Raw("    }\n");
      }
      cb_impl->Raw("  }\n\n");
    }

    for (auto param : def->GetParameterDecls()) {
      if (param->spec->GetKind() == ParameterSpec::kSpline) {
        if (!static_cast<SplineParameterSpec*>(param->spec.get())->is_grid)
          WriteSplineZero(cb_impl, param);
        continue;
      }
    }

    cb_impl->Raw("  char * file = arg[2];\n");
    cb_impl->Raw("  FILE *fp;\n");
    cb_impl->Raw("  if (comm->me == 0) {\n");
    cb_impl->Raw("    fp = force->open_potential(file);\n");
    cb_impl->Raw("    if (fp == NULL) {\n");
    cb_impl->Raw("      char str[128];\n");
    cb_impl->Raw(
        "      sprintf(str, \"Cannot open potential file %%s\",file);\n");
    cb_impl->Raw("      error->one(FLERR,str);\n");
    cb_impl->Raw("    }\n");
    cb_impl->Raw("  }\n\n");
    cb_impl->Raw("  const int MAXLINE = 1024;");
    cb_impl->Raw("  char line[MAXLINE],*ptr;\n");
    cb_impl->Raw("  int n, eof = 0;\n\n");
    cb_impl->Raw("  const int MAX_WORDS = 128;\n");
    cb_impl->Raw("  char * words[MAX_WORDS] = {0};\n");
    cb_impl->Raw("  int next_word = 0;\n\n");
    cb_impl->Raw("  int read_header = 0; // 0 = read data, 1 = read header\n");
    cb_impl->Raw("  while (1) {\n");
    cb_impl->Raw("    if (comm->me == 0) {\n");
    cb_impl->Raw("      ptr = fgets(line,MAXLINE,fp);\n");
    cb_impl->Raw("      if (ptr == NULL) {\n");
    cb_impl->Raw("        eof = 1;\n");
    cb_impl->Raw("        fclose(fp);\n");
    cb_impl->Raw("      } else n = strlen(line) + 1;\n");
    cb_impl->Raw("    }\n");
    cb_impl->Raw("    MPI_Bcast(&eof,1,MPI_INT,0,world);\n");
    cb_impl->Raw("    if (eof) break;\n");
    cb_impl->Raw("    MPI_Bcast(&n,1,MPI_INT,0,world);\n");
    cb_impl->Raw("    MPI_Bcast(line,n,MPI_CHAR,0,world);\n\n");
    cb_impl->Raw("    // strip comment, skip line if blank\n");
    cb_impl->Raw("    if ((ptr = strchr(line,'#'))) *ptr = '\\0';\n\n");
    cb_impl->Raw("    char *word = strtok(line,\" \\t\\n\\r\\f\");\n");
    cb_impl->Raw("    while (word) {\n");
    cb_impl->Raw(
        "      if (next_word > MAX_WORDS) error->all(FLERR, \"Too many words "
        "in line.\");\n");
    cb_impl->Raw("      free(words[next_word]);\n");
    cb_impl->Raw("      words[next_word] = strdup(word);\n");
    cb_impl->Raw("      int is_header = isalpha(words[next_word][0]);\n");
    cb_impl->Raw("      if ((read_header == 0) && is_header) {\n");
    cb_impl->Raw("        file_process_line(next_word, words, arg + 3 - 1);\n");
    cb_impl->Raw("        if (next_word != 0) {\n");
    cb_impl->Raw("          words[0] = words[next_word];\n");
    cb_impl->Raw("          words[next_word] = NULL;\n");
    cb_impl->Raw("        }\n");
    cb_impl->Raw("        next_word = 0;\n");
    cb_impl->Raw("        read_header = 1;\n");
    cb_impl->Raw("      } else read_header = is_header;\n");
    cb_impl->Raw("      next_word += 1;\n");
    cb_impl->Raw("      word = strtok(NULL, \" \\t\\n\\r\\f\");\n");
    cb_impl->Raw("    }\n");
    cb_impl->Raw("  }\n");
    cb_impl->Raw("  file_process_line(next_word, words, arg + 3 - 1);\n\n");
    cb_impl->Raw("  n = atom->ntypes;\n");
    cb_impl->Raw("  for (int i = 1; i <= n; i++)\n");
    cb_impl->Raw("    for (int j = i; j <= n; j++)\n");
    cb_impl->Raw("      setflag[i][j] = 0;\n\n");
    cb_impl->Raw(
        "  // set setflag i,j for type pairs where both are mapped to "
        "elements\n\n");
    cb_impl->Raw("  int count = 0;\n");
    cb_impl->Raw("  for (int i = 1; i <= n; i++)\n");
    cb_impl->Raw("    for (int j = i; j <= n; j++)\n");
    cb_impl->Raw(
        "      if ((strcmp(\"NULL\", arg[i + 3 - 1]) != 0) && "
        "(strcmp(\"NULL\", arg[j + 3 - 1]) != 0)) {\n");
    cb_impl->Raw("        setflag[i][j] = 1;\n");
    cb_impl->Raw("        count++;\n");
    cb_impl->Raw("      }\n\n");
    cb_impl->Raw(
        "  if (count == 0) error->all(FLERR,\"Incorrect args for pair "
        "coefficients\");\n\n");
    for (auto param : def->GetParameterDecls()) {
      if (param->spec->GetKind() != ParameterSpec::kSpline) continue;
      auto spec = static_cast<SplineParameterSpec*>(param->spec.get());
      if (spec->is_grid) {
        for (int i = 0; i < param->GetNumArgs(); i++) {
          if (param->GetArgumentType(i) == Type::kAtomType) continue;
          WriteRegularVisitor vis(cb_impl, &ana_comm);
          vis.indent = "    ";
          cb_impl->Raw("  {\n");
          auto low = potc::der::GenerateCode(def, spec->low.get());
          low.first->Accept(&vis);
          vis.context.push_back(kIRDataTypeDouble);
          cb_impl->Raw("    spline_low_%d_%s = ", i, param->GetNameC());
          low.second->Accept(&vis);
          cb_impl->Raw(";\n");
          cb_impl->Raw("  }\n");
          cb_impl->Raw("  {\n");
          auto step = potc::der::GenerateCode(def, spec->step.get());
          step.first->Accept(&vis);
          cb_impl->Raw("    spline_step_%d_%s = ", i, param->GetNameC());
          step.second->Accept(&vis);
          cb_impl->Raw(";\n");
          cb_impl->Raw("  }\n");
          cb_impl->Raw(
              "    spline_step_inv_%1$d_%2$s = 1 / spline_step_%1$d_%2$s;\n", i,
              param->GetNameC());
        }
      }
      cb_impl->Raw("  spline_fit_%s();\n", param->GetName().c_str());
    }
    for (int outlined_num = 0; outlined_num < outlined->size();
         outlined_num++) {
      int cardinality = outlined->at(outlined_num).first;
      std::string indent = "  ";
      for (int i = 1; i <= cardinality; i++) {
        cb_impl->Raw(
            "%sfor (int i_ty_%d = 1; i_ty_%d <= atom->ntypes; i_ty_%d++) {\n",
            indent.c_str(), i, i, i);
        indent = indent + "  ";
      }
      cb_impl->Raw("%soutlined_param_%d", indent.c_str(), outlined_num);
      for (int i = 1; i <= cardinality; i++) {
        cb_impl->Raw("[i_ty_%d]", i);
      }
      cb_impl->Raw(" = ");
      WriteRegularVisitor vis(cb_impl, &ana_comm);
      vis.context.push_back(kIRDataTypeDouble);
      outlined->at(outlined_num).second->Accept(&vis);
      cb_impl->Raw(";\n");
      for (int i = 0; i < cardinality; i++) {
        indent = indent.substr(2);
        cb_impl->Raw("%s}\n", indent.c_str());
      }
    }
    // TODO(markus) setup cutmax appropriately here
    // cb_impl->Raw("\n}\n");
    return cb_impl->Indent();
  }

  virtual std::string GetPerAtomName(PerAtomActionIRStmt* stmt) {
    assert(stmt->IsComm());
    if (stmt->IsAdjoint()) return "peratom_adjoint_" + stmt->id + "[j]";
    return "peratom_" + stmt->id + "[j]";
  }

  virtual void WriteReverse() {
    WriteFunctionImpl("int ");
    cb_impl->Line("pack_reverse_comm(int n, int first, double* buf) {")
        ->Indent()
        ->Line("int m = 0;");
    for (int i = 0; i < ana_comm.stage_assignment.size(); i++) {
      if (!ana_comm.IsReverse(i)) continue;
      cb_impl->RawL("if (stage_peratom == %d) {", i)
          ->Indent()
          ->Line("for (int i = 0; i < n; i++) {")
          ->Indent()->Line("int j = first + i;");
      for (auto decl : ana_comm.stage_assignment[i]) {
        if (!decl->IsReverse()) continue;
        cb_impl->Line("buf[m++] = " + GetPerAtomName(decl) + ";");
      }
      cb_impl->End()->End();
    }
    cb_impl->Line("return m;")->End();
    WriteFunctionImpl("void ");
    cb_impl->Line("unpack_reverse_comm(int n, int *list, double* buf) {")
        ->Indent()
        ->Line("int m = 0;");
    for (int i = 0; i < ana_comm.stage_assignment.size(); i++) {
      if (!ana_comm.IsReverse(i)) continue;
      cb_impl->RawL("if (stage_peratom == %d) {", i)
          ->Indent()
          ->Line("for (int i = 0; i < n; i++) {")
          ->Indent()->Line("int j = list[i];");
      for (auto decl : ana_comm.stage_assignment[i]) {
        if (!decl->IsReverse()) continue;
        cb_impl->Line(GetPerAtomName(decl) + " += buf[m++];");
      }
      cb_impl->End()->End();
    }
    cb_impl->End();
  }
  virtual void WriteForward() {
    WriteFunctionImpl("int ");
    cb_impl
        ->Line(
            "pack_forward_comm(int n, int *list, double* buf, int "
            "/*pbc_flag*/, int* /*pbc*/) {")
        ->Indent()
        ->Line("int m = 0;");
    for (int i = 0; i < ana_comm.stage_assignment.size(); i++) {
      if (!ana_comm.IsForward(i)) continue;
      cb_impl->RawL("if (stage_peratom == %d) {", i)
          ->Indent()
          ->Line("for (int i = 0; i < n; i++) {")
          ->Indent()->Line("int j = list[i];");
      for (auto decl : ana_comm.stage_assignment[i]) {
        if (!decl->IsForward()) continue;
        cb_impl->Line("buf[m++] = " + GetPerAtomName(decl) + ";");
      }
      cb_impl->End()->End();
    }
    cb_impl->Line("return m;")->End();
    WriteFunctionImpl("void ");
    cb_impl->Line("unpack_forward_comm(int n, int first, double* buf) {")
        ->Indent()
        ->Line("int m = 0;");
    for (int i = 0; i < ana_comm.stage_assignment.size(); i++) {
      if (!ana_comm.IsForward(i)) continue;
      cb_impl->RawL("if (stage_peratom == %d) {", i)
          ->Indent()
          ->Line("for (int i = 0; i < n; i++) {")
          ->Indent()->Line("int j = first + i;");
      for (auto decl : ana_comm.stage_assignment[i]) {
        if (!decl->IsForward()) continue;
        cb_impl->Line(GetPerAtomName(decl) + " = buf[m++];");
      }
      cb_impl->End()->End();
    }
    cb_impl->End();
  }
  virtual void WritePackUnpack() {
    WriteForward();
    WriteReverse();
  }

  virtual void WriteRegular() {
    ana_neigh.Visit(compute);
    ana_neigh.Finalize();
    printf("Max skip: %d\n", ana_neigh.max_num_skips);
    ana_comm.Visit(compute);
    printf("Num stages: %zu\n", ana_comm.stage_assignment.size());
    CollectArrays();
    WriteHeader();
    WriteImplStart();
    WriteConstructor()->End();
    WriteDestructor()->End();
    WriteCompute()->End();
    WriteInitStyle()->End();
    WriteAllocate()->End();
    WriteInitOne()->End();
    WriteSettings()->End();
    WriteFileProcessLine();
    for (auto param : def->GetParameterDecls()) {
      if (param->spec->GetKind() != ParameterSpec::kSpline) continue;

      WriteSplineEval(cb_impl, name_class, param);
      WriteSplineInit(cb_impl, name_class, param);
    }
    if (num_peratom > 0) {
      WritePackUnpack();
    }
  }

  virtual void Close() {
    fclose(out_impl);
    fclose(out_header);
  }
};

}  // namespace gen

}  // namespace potc

#endif  // GEN_REGULAR_GENERATOR_H_
