// Copyright 2018 Markus Hoehnerbach
#ifndef GEN_INTEL_GENERATOR_H_
#define GEN_INTEL_GENERATOR_H_ GEN_INTEL_GENERATOR_H_

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "gen_intel_visitor.h"
#include "gen_regular_generator.h"

namespace potc {

namespace gen {

struct IntelGenerator : public RegularGenerator {
  IntelGenerator(std::string cname, CompoundIRStmt* ccompute,
                 PotentialDef* cdef,
                 std::vector<std::pair<int, std::unique_ptr<IRExpr>>>* coutl)
      : RegularGenerator(cname + "/intel", ccompute, cdef, coutl) {
    nan_catch = false;
  }

  void WriteSplineVecEvalArgs(CodeBuilder* cb, ParameterDecl* param) {
    for (int i = 0; i < param->GetNumArgs(); i++) {
      if (i != 0) cb->Fragment(", ");
      if (param->GetArgumentType(i) == Type::kAtomType) {
        cb->Fragment("ivec");
      } else {
        cb->Fragment("fvec");
      }
      cb->Raw(" arg_%d", i);
    }
    for (int i = 0; i < param->GetNumArgs(); i++) {
      if (param->GetArgumentType(i) == Type::kAtomType) continue;
      cb->Raw(", fvec *d_%d", i);
    }
  }
  void WriteSplineVecDecl(CodeBuilder* cb, ParameterDecl* param) {
    cb->StartFragment()->Raw("fvec spline_eval_%s(",
                             param->GetNameC());
    WriteSplineVecEvalArgs(cb, param);
    cb->Raw(");")->EndFragment();
  }
  void WriteSplineVecEval(CodeBuilder* cb_impl, const std::string& name_class,
                          ParameterDecl* param) {
    int num_args = param->GetArguments().size();
    auto name_str = param->GetName();
    const char* name = name_str.c_str();
    SplineParameterSpec* spec =
        static_cast<SplineParameterSpec*>(param->spec.get());
    cb_impl->Raw("\ntemplate<typename flt_t, typename acc_t>");
    cb_impl->Raw("\ntypename %1$s::fns<flt_t,acc_t>::fvec %1$s::fns<flt_t,acc_t>::spline_eval_%2$s(", name_class.c_str(),
                 param->GetName().c_str());
    WriteSplineVecEvalArgs(cb_impl, param);
    cb_impl->Raw(") {\n");

    //_mm512_min_pd()
    //_mm512_max_pd()
    //_mm512_set1_pd()
    //_mm512_floor_pd()
    //_mm512_cvtpd_epi32()
    //_mm512_sub_pd()
    //_mm512_add_pd()
    //_mm512_mul_pd()
    //_mm512_set1_epi32()
    //_mm512_castsi256_si512
    //_mm512_cmplt_mask_epi32(

    auto str = R"(
      @loop@@ifcont@@
      @@ifinteger@@
      @arg_@idx@ = fvec::max(fvec::setzero(), fvec::min(fvec::set1(that->@size@ - 1), arg_@idx@));
      ivec idx_@idx@ = fvec::cvt_ivec(fvec::floor(arg_@idx@));
      idx_@idx@ = ivec::min(idx_@idx@, ivec::set1(that->@size@ - 2));
      @/@@ifgrid@@
      @arg_@idx@ = fvec::max(@
      @fvec::set1(that->spline_low_@idx@_@name@), @
      @fvec::min(fvec::set1(that->spline_low_@idx@_@name@ + @
      @that->spline_step_@idx@_@name@ * (that->@size@ - 1)), arg_@idx@));
      ivec idx_@idx@ = fvec::cvt_ivec(fvec::floor(@
      @(@
      @(arg_@idx@ - fvec::set1(that->spline_low_@idx@_@name@)) * @
      @fvec::set1(that->spline_step_inv_@idx@_@name@))));
      idx_@idx@ = ivec::min(idx_@idx@, ivec::set1(that->@size@ - 2));
      @/@@ifregular@@
      @arg_@idx@ = fvec::max(fvec::set1(fc.spline_limits_@idx@_@name@[0]), fvec::min(fvec::set1(fc.spline_limits_@idx@_@name@@
      @[that->@size@ - 1]), arg_@idx@));
      ivec idx_@idx@ = ivec::setzero();
      for (int i = 0; i < that->@size@ - 1; i++) {
        bvec mask_gt = fvec::cmpnle(arg_@idx@, fvec::set1(fc.spline_limits_@idx@_@name@[i]));
        bvec mask_lt = fvec::cmple(arg_@idx@, fvec::set1(fc.spline_limits_@idx@_@name@[i+1]));
        bvec mask_and = mask_gt & mask_lt;
        idx_@idx@ = ivec::mask_blend(mask_and, idx_@idx@, ivec::set1(i));
      }
      @/@@/@@/@@
      @flt_t* coeff = &fc.spline_coeffs_@name@@loop@[0]@/@[0];
      ivec offset = @
      @@loop@@
        @(
        @
        @@loopj@@
          @@ifjidx>idx@@
          @ivec::mullo(ivec::set1(that->@jsize@), @
          @@/@@
        @@/@@
        @@iftype@@
          @arg_@idx@@
        @@/@@
        @@ifcont@@
          @idx_@idx@@
        @@/@@
        @@loopj@@
          @@ifjidx>idx@@
            @)@
          @@/@@
        @@/@ + 
      @/@@
      @ivec::setzero()@
      @@loop@@
        @)@
      @@/@;
      fvec result = fvec::setzero();
      @loop@@ifcont@@
      @fvec dt_@idx@ = fvec::setzero();
      @/@@/@@loop@@ifcont@@
      @@indent@fvec x_@idx@ = fvec::set1(1);
      @indent@fvec dx_@idx@ = fvec::setzero();
      @indent@ivec old_offset_@idx@ = offset;
      @indent@for (int p_@idx@ = 0; p_@idx@ <= @order@; p_@idx@++) {
      @indent@  ivec offset = (ivec::mullo(old_offset_@idx@, @
      @ivec::set1(@order@ + 1)) + ivec::set1(p_@idx@));
      @/@@/@@
      @@full_indent@fvec the_coeff = fvec::mask_gather<sizeof(flt_t)>(fvec::setzero(), mask, offset, coeff);
      @full_indent@result = (result + @
      @@loop@@ifcont@(x_@idx@ * @/@@/@the_coeff@
      @@loop@@ifcont@)@/@@/@);
      @loop@@ifcont@@
      @@full_indent@dt_@idx@ = dt_@idx@ + @
      @fvec::set1(p_@idx@) * the_coeff @
      @@loopj@@ifjcont@ * @ifidx=jidx@d@/@x_@jidx@@/@@/@@
      @;
      @/@@/@@
      @@rloop@@ifcont@@
      @@indent@  dx_@idx@ = x_@idx@;
      @indent@  x_@idx@ = (x_@idx@ * arg_@idx@);
      @indent@}
      @/@@/@@
      @@loop@@ifcont@@
      @*d_@idx@ = dt_@idx@;
      @/@@/@@
      @return result;)";
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
      loop_args[i]["regular"] =
          !spec->is_integer && !spec->is_grid ? "true" : "";
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
    // for (int i = 0; i < param->GetNumArgs(); i++) {
    //  auto atom_template = "  int arr_arg_%1$d[16];\n
    //  _mm512_storeu_si512(arr_arg_%1$d, arg_%1$d);\n";
    //  auto real_template = "  double arr_arg_%1$d[8];\n
    //  _mm512_storeu_pd(arr_arg_%1$d, arg_%1$d);\n  double arr_d_%1$d[8];\n";
    //  cb_impl->Raw(param->GetArgumentType(i) == Type::kAtomType ?
    //  atom_template : real_template, i);
    //}
    // cb_impl->Raw("  double ret[8];");
    // cb_impl->Raw("  for (int i = 0; i < 8; i++) {\n");

    // cb_impl->Raw("    ret[i] = spline_eval_%s(", param->GetNameC());
    // for (int i = 0; i < param->GetNumArgs(); i++) {
    //  if (i != 0) cb_impl->Fragment(", ");
    //  cb_impl->Raw("arr_arg_%d[i]", i);
    //}
    // for (int i = 0; i < param->GetNumArgs(); i++) {
    //  if (param->GetArgumentType(i) != Type::kAtomType) {
    //    cb_impl->Raw(", &arr_d_%d[i]", i);
    //  }
    //}
    // cb_impl->Raw(");\n");
    // cb_impl->Raw("  }\n");
    // for (int i = 0; i < param->GetNumArgs(); i++) {
    //  if (param->GetArgumentType(i) != Type::kAtomType) {
    //    cb_impl->Raw("  *d_%d = _mm512_loadu_pd(arr_d_%d);\n", i, i);
    //  }
    //}
    // cb_impl->Raw("  \n");
    // cb_impl->Raw("  return _mm512_loadu_pd(ret);\n");
    cb_impl->Raw("}\n");
  }
  CodeBuilder* WriteImplStart() override {
    return RegularGenerator::WriteImplStart()
        ->Line("")
        ->Line("#include \"suffix.h\"")
        ->Line("#include \"modify.h\"")
        ->Line("#include \"fix_intel.h\"")
        ->Line("#include <immintrin.h>")
        ->Line("#include \"intel_intrinsics_potc.h\"");
  }
  CodeBuilder* WriteHeaderForwardDecls() override {
    return RegularGenerator::WriteHeaderForwardDecls()
        ->Line("class FixIntel;\n")
        ->Line("template<class flt_t, class acc_t>")
        ->Line("class IntelBuffers;\n");
  }
  virtual CodeBuilder* WriteHeaderIncludes() {
    return RegularGenerator::WriteHeaderIncludes()->Line(
        "#include <immintrin.h>")->Line("")->Line("template<typename flt_t, typename acc_t>")->Line("struct potc_intr_types;");
  }
  CodeBuilder* WriteHeaderClassContent() override {
    auto cb =
        RegularGenerator::WriteHeaderClassContent()
            ->Indent()
            ->Line("FixIntel* fix;")
            ->Line("int onetype;")
            ->Line("")
            ->Line("template <class flt_t> class ForceConst;")
            ->Line("")
            ->Line("template <class flt_t, class acc_t>")
            ->Line(
                "void compute(int eflag, int vflag, IntelBuffers<flt_t,acc_t> "
                "*buffers,")
            ->Line("             const ForceConst<flt_t> &fc);")
            ->Line(
                "template <int ONETYPE, int EFLAG, class flt_t, class acc_t>")
            ->Line("void eval(const int offload, const int vflag,")
            ->Line(
                "          IntelBuffers<flt_t,acc_t> * buffers, const "
                "ForceConst<flt_t> &fc,")
            ->Line("          const int astart, const int aend);")
            ->Line("")
            ->Line("template <class flt_t, class acc_t>")
            ->Line("void pack_force_const(ForceConst<flt_t> &fc,")
            ->Line(
                "                      IntelBuffers<flt_t, acc_t> *buffers);")
            ->Line("")
            ->Line(
                "// "
                "--------------------------------------------------------------"
                "----"
                "----")
            ->Line("")
            ->Line("template <class flt_t>")
            ->Line("struct ForceConst {")
            ->Line("  ForceConst() : _ntypes(0)  {}")
            ->Line("  ~ForceConst() { set_ntypes(0, NULL, _cop, NULL); }");
    for (auto& array : arrays) {
      if (array.type != "double") continue;
      if (array.guard == "nmax") continue;
      if (array.arity == 0) continue;
      WriteHeaderParam(array.arity, array.name, "flt_t");
    }
    cb->Line("")
        ->RawL(
            "  void set_ntypes(const int ntypes, Memory *memory, const int "
            "cop, %s *that);", name_class.c_str())
        ->Line("")
        ->Line(" private:")
        ->Line("  int _ntypes, _cop;")
        ->Line("  Memory *_memory;")
        ->Line("};")
        ->Line("ForceConst<float> force_const_single;")
        ->Line("ForceConst<double> force_const_double;")
        ->Line("template<typename flt_t, typename acc_t>")
        ->Line("struct fns {")
        ->Indent()
        ->Line("typedef typename potc_intr_types<flt_t, acc_t>::fvec fvec;")
        ->Line("typedef typename potc_intr_types<flt_t, acc_t>::avec avec;")
        ->Line("typedef typename potc_intr_types<flt_t, acc_t>::ivec ivec;")
        ->Line("typedef typename potc_intr_types<flt_t, acc_t>::bvec bvec;")
        ->Line("const ForceConst<flt_t> &fc;")
        ->RawL("const %s *that;", name_class.c_str())
        ->RawL("const bvec &mask;", name_class.c_str())
        ->RawL("fns(const ForceConst<flt_t> &cfc, const %s *cthat, const bvec& cmask) : fc(cfc), that(cthat), mask(cmask) {}", name_class.c_str());
    for (auto param : def->GetParameterDecls()) {
      if (param->spec->GetKind() != ParameterSpec::kSpline) continue;

      WriteSplineVecDecl(cb, param);
    }
    cb->Undent()->Line("};");
    return cb->Undent();
  }
  CodeBuilder* WriteConstructor() override {
    return RegularGenerator::WriteConstructor()->Line(
        "suffix_flag |= Suffix::INTEL;");
  }
  CodeBuilder* WriteInitStyle() override {
    auto cb = RegularGenerator::WriteInitStyle()
        ->Line("neighbor->requests[irequest]->intel = 1;")
        ->Line("int ifix = modify->find_fix(\"package_intel\");")
        ->If("ifix < 0")
        ->Line(
            "error->all(FLERR, \"The 'package intel' command is required for "
            "/intel styles\");")
        ->End()
        ->Line("fix = static_cast<FixIntel *>(modify->fix[ifix]);")
        ->Line("fix->pair_init_check();");
    if (ana_neigh.need_full) {
      cb->Line("fix->three_body_neighbor(1);");
    }
    cb->Line("if (fix->precision() == FixIntel::PREC_MODE_MIXED) {")
        ->Line(
            "  pack_force_const(force_const_single, fix->get_mixed_buffers());")
        ->Line("  fix->get_mixed_buffers()->need_tag(1);")
        ->Line("} else if (fix->precision() == FixIntel::PREC_MODE_DOUBLE) {")
        ->Line(
            "  pack_force_const(force_const_double, "
            "fix->get_double_buffers());")
        ->Line("  fix->get_double_buffers()->need_tag(1);")
        ->Line("} else {")
        ->Line(
            "  pack_force_const(force_const_single, "
            "fix->get_single_buffers());")
        ->Line("  fix->get_single_buffers()->need_tag(1);")
        ->Line("}");
    return cb;

    // three_body_flag, ccache, onetype specialization
    // TODO(markus) Need to set cutneighsq (I think)
  }
  CodeBuilder* WriteCompute() {
    // Write compute dispatch for precision
    auto c =
        cb_impl->Line(LINE)
            ->Line("void " + name_class + "::compute(int eflag, int vflag) {")
            ->Indent();
    if (nan_catch)
      c->Line("feenableexcept(FE_INVALID | FE_DIVBYZERO | FE_OVERFLOW);")
          ->Line("");
    c->Line("if (fix->precision() == FixIntel::PREC_MODE_MIXED) {")
        ->Line(
            "  compute<float,double>(eflag, vflag, fix->get_mixed_buffers(),")
        ->Line("                        force_const_single);")
        ->Line("} else if (fix->precision() == FixIntel::PREC_MODE_DOUBLE) {")
        ->Line(
            "  compute<double,double>(eflag, vflag, fix->get_double_buffers(),")
        ->Line("                         force_const_double);")
        ->Line("} else {")
        ->Line(
            "  compute<float,float>(eflag, vflag, fix->get_single_buffers(),")
        ->Line("                       force_const_single);")
        ->Line("}")
        ->Line("")
        ->Line("fix->balance_stamp();")
        ->Line("vflag_fdotr = 0;");
    if (nan_catch) c->Line("")->Line("fesetenv(FE_DFL_ENV);");
    c->End()
        ->Line(LINE)
        ->Line("template <class flt_t, class acc_t>")
        ->Line("void " + name_class + "::compute(int eflag, int vflag,")
        ->Line("    IntelBuffers<flt_t,acc_t> *buffers,")
        ->Line("    const ForceConst<flt_t> &fc")
        ->Line(") {")
        ->Indent()
        ->Line("if (eflag || vflag) ev_setup(eflag,vflag);")
        ->Line("else evflag = vflag_fdotr = 0;\n")
        ->Line("const int inum = list->inum;")
        ->Line("const int nthreads = comm->nthreads;")
        ->Line("const int host_start = fix->host_start_pair();")
        ->Line("const int offload_end = fix->offload_end_pair();")
        ->Line("const int ago = neighbor->ago;")
        ->Line("")
        ->Line("if (ago != 0 && fix->separate_buffers() == 0) {")
        ->Line("  fix->start_watch(TIME_PACK);")
        ->Line("")
        ->Line("  int packthreads;")
        ->Line("  if (nthreads > INTEL_HTHREADS) packthreads = nthreads;")
        ->Line("  else packthreads = 1;")
        ->Line("  #if defined(_OPENMP)")
        ->Line("  #pragma omp parallel if(packthreads > 1)")
        ->Line("  #endif")
        ->Line("  {")
        ->Line("    int ifrom, ito, tid;")
        ->Line(
            "    IP_PRE_omp_range_id_align(ifrom, ito, tid, atom->nlocal + "
            "atom->nghost,")
        ->Line("                              packthreads, sizeof(ATOM_T));")
        ->Line("    buffers->thr_pack(ifrom, ito, ago);")
        ->Line("  }")
        ->Line("")
        ->Line("  fix->stop_watch(TIME_PACK);")
        ->Line("}")
        ->Line("")
        ->Line("int ovflag = 0;")
        ->Line("if (vflag_fdotr) ovflag = 2;")
        ->Line("else if (vflag) ovflag = 1;")
        ->Line("onetype = atom->ntypes == 1;")
        ->Line("if (onetype) {")
        ->Line("  if (eflag) {")
        ->Line("    eval<1, 1>(1, ovflag, buffers, fc, 0, offload_end);")
        ->Line("    eval<1, 1>(0, ovflag, buffers, fc, host_start, inum);")
        ->Line("  } else {")
        ->Line("    eval<1, 0>(1, ovflag, buffers, fc, 0, offload_end);")
        ->Line("    eval<1, 0>(0, ovflag, buffers, fc, host_start, inum);")
        ->Line("  }")
        ->Line("} else {")
        ->Line("  if (eflag) {")
        ->Line("    eval<0, 1>(1, ovflag, buffers, fc, 0, offload_end);")
        ->Line("    eval<0, 1>(0, ovflag, buffers, fc, host_start, inum);")
        ->Line("  } else {")
        ->Line("    eval<0, 0>(1, ovflag, buffers, fc, 0, offload_end);")
        ->Line("    eval<0, 0>(0, ovflag, buffers, fc, host_start, inum);")
        ->Line("  }")
        ->Line("}")
        ->End()
        ->Line(LINE)
        //->Line("template <int ONETYPE, int EFLAG, int NEWTON_PAIR, class
        // flt_t, class acc_t>")
        ->Line("template <int ONETYPE, int EFLAG, class flt_t, class acc_t>")
        ->Line("void " + name_class +
               "::eval(const int offload, const int vflag,")
        ->Line("                        IntelBuffers<flt_t,acc_t> *buffers,")
        ->Line("                        const ForceConst<flt_t> &fc,")
        ->Line("                        const int astart, const int aend)")
        ->Line("{")
        ->Indent()
        ->Line("const bool NEWTON_PAIR = true;")
        ->Line("const int inum = aend - astart;")
        ->Line("if (inum == 0) return;")
        ->Line("")
        ->Line("int nlocal, nall, minlocal;")
        ->Line("fix->get_buffern(offload, nlocal, nall, minlocal);")
        ->Line("")
        ->Line("const int ago = neighbor->ago;")
        ->Line(
            "IP_PRE_pack_separate_buffers(fix, buffers, ago, offload, nlocal, "
            "nall);")
        ->Line("")
        ->Line("ATOM_T * _noalias const x = buffers->get_x(offload);")
        ->Line("")
        ->Line("const int * _noalias const numneigh = list->numneigh;")
        ->Line(
            "const int * _noalias const numneighhalf = buffers->get_atombin();")
        ->Line(
            "const int * _noalias const cnumneigh = buffers->cnumneigh(list);")
        ->Line(
            "const int * _noalias const firstneigh = "
            "buffers->firstneigh(list);")
        ->Line("const int eatom = this->eflag_atom;")
        ->Line("int tp1 = atom->ntypes + 1;")
        ->Line("")
        ->Line("// Determine how much data to transfer")
        ->Line("int x_size, q_size, f_stride, ev_size, separate_flag;")
        ->Line("IP_PRE_get_transfern(ago, NEWTON_PAIR, EFLAG, vflag,")
        ->Line("                     buffers, offload, fix, separate_flag,")
        ->Line("                     x_size, q_size, ev_size, f_stride);")
        ->Line("")
        ->Line("int tc;")
        ->Line("FORCE_T * _noalias f_start;")
        ->Line("acc_t * _noalias ev_global;")
        ->Line(
            "IP_PRE_get_buffers(offload, buffers, fix, tc, f_start, "
            "ev_global);")
        ->Line("const int nthreads = tc;")
        ->Line("int *overflow = fix->get_off_overflow_flag();")
        ->Line("")
        ->Line("{")
        ->Line("  #if defined(__MIC__) && defined(_LMP_INTEL_OFFLOAD)")
        ->Line("  *timer_compute = MIC_Wtime();")
        ->Line("  #endif")
        ->Line("")
        ->Line(
            "  IP_PRE_repack_for_offload(NEWTON_PAIR, separate_flag, nlocal, "
            "nall,")
        ->Line("                            f_stride, x, 0);")
        ->Line("")
        ->Line("  acc_t oevdwl, ov0, ov1, ov2, ov3, ov4, ov5;")
        ->Line("  if (EFLAG) oevdwl = (acc_t)0;")
        ->Line("  if (vflag) ov0 = ov1 = ov2 = ov3 = ov4 = ov5 = (acc_t)0;")
        ->Line("")
        ->Line("  // loop over neighbors of my atoms")
        ->Line("  #if defined(_OPENMP)")
        ->Line(
            "  #pragma omp parallel "
            "reduction(+:oevdwl,ov0,ov1,ov2,ov3,ov4,ov5)")
        ->Line("  #endif")
        ->Line("  {")
        ->Line("    int iifrom, iito, tid;")
        ->Line("    IP_PRE_omp_range_id_vec(iifrom, iito, tid, inum, nthreads,")
        ->Line("                            INTEL_VECTOR_WIDTH);")
        ->Line("    iifrom += astart;")
        ->Line("    iito += astart;")
        ->Line("    int iafrom, iato;")
        ->Line("    IP_PRE_omp_range_id_vec(iafrom, iato, tid, atom->nlocal + atom->nghost, nthreads,")
        ->Line("                            INTEL_VECTOR_WIDTH);")
        ->Line("    iafrom += astart;")
        ->Line("    iato += astart;")
        ->Line("")
        ->Line(
            "    FORCE_T * _noalias const f = f_start - minlocal + (tid * "
            "f_stride);")
        ->Line("    memset(f + minlocal, 0, f_stride * sizeof(FORCE_T));")
        ->Line("")
        ->Line("")
        ->Line("  typedef typename potc_intr_types<flt_t, acc_t>::fvec fvec;")
        ->Line("  typedef typename potc_intr_types<flt_t, acc_t>::avec avec;")
        ->Line("  typedef typename potc_intr_types<flt_t, acc_t>::ivec ivec;")
        ->Line("  typedef typename potc_intr_types<flt_t, acc_t>::bvec bvec;")
        ->Line("");
    if (num_peratom > 0) {
      cb_impl->Raw("  if (atom->nmax > nmax) {\n");
      cb_impl->Raw("    nmax = atom->nmax;\n");
      DeallocateArrays("nmax");
      AllocateArrays("nmax");
      cb_impl->Raw("  }\n");
    }
    c->Line("")->Line("  bvec t_0 = bvec::full();");
    WriteIntelVectorVisitor vis(cb_impl, &ana_comm);
    vis.ana_neigh_need_full = ana_neigh.need_full;
    static_cast<IRStmtVisitor*>(&vis)->Visit(compute);
    c->Line("")
        ->Line("")
        ->Line(
            "    IP_PRE_fdotr_reduce_omp(1, nall, minlocal, nthreads, f_start, "
            "f_stride,")
        ->Line(
            "                            x, offload, vflag, ov0, ov1, ov2, "
            "ov3, ov4, ov5);")
        ->Line("  } // end omp")
        ->Line("")
        ->Line("  IP_PRE_fdotr_reduce(1, nall, nthreads, f_stride, vflag,")
        ->Line("                      ov0, ov1, ov2, ov3, ov4, ov5);")
        ->Line("")
        ->Line("  if (EFLAG) {")
        ->Line("    ev_global[0] = oevdwl;")
        ->Line("    ev_global[1] = (acc_t)0.0;")
        ->Line("  }")
        ->Line("  if (vflag) {")
        ->Line("    ev_global[2] = ov0;")
        ->Line("    ev_global[3] = ov1;")
        ->Line("    ev_global[4] = ov2;")
        ->Line("    ev_global[5] = ov3;")
        ->Line("    ev_global[6] = ov4;")
        ->Line("    ev_global[7] = ov5;")
        ->Line("  }")
        ->Line("  #if defined(__MIC__) && defined(_LMP_INTEL_OFFLOAD)")
        ->Line("  *timer_compute = MIC_Wtime() - *timer_compute;")
        ->Line("  #endif")
        ->Line("} // end offload")
        ->Line("if (offload)")
        ->Line("  fix->stop_watch(TIME_OFFLOAD_LATENCY);")
        ->Line("else")
        ->Line("  fix->stop_watch(TIME_HOST_PAIR);")
        ->Line("")
        ->Line("if (EFLAG || vflag)")
        ->Line(
            "  fix->add_result_array(f_start, ev_global, offload, eatom, 0, "
            "vflag);")
        ->Line("else")
        ->Line("  fix->add_result_array(f_start, 0, offload);");
    return c;
    // Write compute with thr_pack and dispatch into eval
    // Write eval
    // Make it like eam/intel since it does not support offload
    // -> Avoid the corresponding subtleties
    // Dispatch is at least on EFLAG, but sometime EVFLAG (y tho), NEWTON,
    // ONETYPE, SPQ (eam)
  }
  CodeBuilder* WritePackForce() {
    auto cb = cb_impl->Line(LINE)
                  ->Line("template <class flt_t, class acc_t>")
                  ->Line("void " + name_class + "::pack_force_const(")
                  ->Line("    ForceConst<flt_t> &fc,")
                  ->Line("    IntelBuffers<flt_t,acc_t> *buffers")
                  ->Line(") {")
                  ->Indent()
                  ->Line("int tp1 = atom->ntypes + 1;")
                  //->Line("fc.set_ntypes(tp1,memory,_cop);")
                  ->Line("fc.set_ntypes(tp1,memory, -1, this);")
                  ->Line("buffers->set_ntypes(tp1, 1);")
                  ->Line("flt_t **cutneighsq = buffers->get_cutneighsq();")
                  ->Line("")
                  ->Line(
                      "// Repeat cutsq calculation because done after call to "
                      "init_style")
                  ->Line("double cut, cutneigh;")
                  ->Line("for (int i = 1; i <= atom->ntypes; i++) {")
                  ->Line("  for (int j = i; j <= atom->ntypes; j++) {")
                  ->Line(
                      "    if (setflag[i][j] != 0 || (setflag[i][i] != 0 && "
                      "setflag[j][j] != 0)) {")
                  ->Line("      cut = init_one(i,j);")
                  ->Line("      cutneigh = cut + neighbor->skin;")
                  ->Line("      cutsq[i][j] = cutsq[j][i] = cut*cut;")
                  ->Line(
                      "      cutneighsq[i][j] = cutneighsq[j][i] = cutneigh * "
                      "cutneigh;")
                  ->Line("    }")
                  ->Line("  }")
                  ->Line("}");
    cb->RawL("int ntypes = atom->ntypes;");
    cb->RawL("        %s *that = this;", name_class.c_str());
    for (auto& array : arrays) {
      if (array.type != "double") continue;
      if (array.guard == "nmax") continue;
      if (array.arity == 0) continue;
      std::string index = "";
      for (int i = 0; i < array.arity; i++) {
        cb->RawL("for (int i_%1$d = 0; i_%1$d < %2$s; i_%1$d++) {", i,
                 array.dims[i].c_str())
            ->Indent();
        index += "[i_" + std::to_string(i) + "]";
      }
      cb->Line("fc." + array.name + index + " = " + array.name + index + ";");
      for (int i = 0; i < array.arity; i++) {
        cb->End();
      }
    }
    cb->End()
        ->Line("template <class flt_t>")
        ->Line("void " + name_class + "::ForceConst<flt_t>::set_ntypes(")
        ->Line("    const int ntypes,")
        ->Line("    Memory *memory,")
        ->Line("    const int cop,")
        ->RawL("    %s *that", name_class.c_str())
        ->Line(") {");
    cb->Indent()->If("_ntypes > 0");
    for (auto& array : arrays) {
      if (array.type != "double") continue;
      if (array.guard == "nmax") continue;
      if (array.arity == 0) continue;
      cb->Line("_memory->destroy(" + array.name + ");");
    }
    cb->End()->If("ntypes > 0");
    for (auto& array : arrays) {
      if (array.type != "double") continue;
      if (array.guard == "nmax") continue;
      if (array.arity == 0) continue;
      cb->Raw("    memory->create(%s, ", array.name.c_str());
      for (auto& dim : array.dims) {
        cb->Raw("%s, ", dim.c_str());
      }
      cb->Raw("\"pair:%s\");\n", array.name.c_str());
    }
    cb->End()->Line("_ntypes = ntypes;")->Line("_memory = memory;");
    return cb;
  }
  void WriteRegular() override {
    RegularGenerator::WriteRegular();
    WritePackForce()->End();
    for (auto param : def->GetParameterDecls()) {
      if (param->spec->GetKind() != ParameterSpec::kSpline) continue;

      WriteSplineVecEval(cb_impl, name_class, param);
    }
  }
};

}  // namespace gen

}  // namespace potc

#endif  // GEN_INTEL_GENERATOR_H_
