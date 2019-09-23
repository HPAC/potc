#ifndef GEN_KOKKOS_GENERATOR_H_
#define GEN_KOKKOS_GENERATOR_H_

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "gen_kokkos_visitor.h"
#include "gen_regular_generator.h"

namespace potc {

namespace gen {

struct KokkosGenerator : public RegularGenerator {
  KokkosGenerator(std::string cname, CompoundIRStmt* ccompute,
                  PotentialDef* cdef,
                  std::vector<std::pair<int, std::unique_ptr<IRExpr>>>* coutl)
      : RegularGenerator(cname + "/kokkos", cname + "/kk", ccompute, cdef,
                         coutl) {
    nan_catch = false;

    COEFF_OUT_LINE = "";
    COEFF_IN_LINE = R"(@
      @@full_indent@double the_coeff = kk_spline_coeffs_@name@(@loop@@
      @@iftype@arg_@idx@@/@@ifcont@idx_@idx@@/@, @/@@loopi@@ificont@@
      @@loopj@@ifjcont@@ifjidx<iidx@(@order@ + 1) * @/@@/@@/@p_@iidx@ + @/@@/@0);
    )";
    EVAL_MODIFIER = "KOKKOS_INLINE_FUNCTION";
    //SPLINE_REGULAR_IDX = R"(@
    //@arg_@idx@ = fmax(kk_spline_limits_@idx@_@name@(0), fmin(kk_spline_limits_@idx@_@name@@
    //@(@size@ - 1), arg_@idx@));
    //int idx_@idx@ = 0;
    //for (int i = 0; i < @size@ - 1; i++) {
    //  if (arg_@idx@ >= kk_spline_limits_@idx@_@name@(i) && @
    //  @arg_@idx@ <= kk_spline_limits_@idx@_@name@(i+1))
    //    idx_@idx@ = i;
    //}
    //)";
    SPLINE_REGULAR_IDX = R"(@
    @if (arg_@idx@ < kk_spline_limits_@idx@_@name@(0))
      arg_@idx@ = kk_spline_limits_@idx@_@name@(0);
    if (arg_@idx@  > kk_spline_limits_@idx@_@name@(@size@ - 1))
      arg_@idx@  = kk_spline_limits_@idx@_@name@(@size@ - 1);
    int idx_@idx@ = 0;
    for (int i = 0; i < @size@ - 1; i++) {
      if (arg_@idx@ >= kk_spline_limits_@idx@_@name@(i) && @
      @arg_@idx@ <= kk_spline_limits_@idx@_@name@(i+1))
        idx_@idx@ = i;
    }
    )";
  }
  CodeBuilder* WriteHeaderPairStyle() override {
    return cb_header
        ->Raw("PairStyle(%s,%s<LMPDeviceType>)\n", name.c_str(),
              name_class.c_str())
        ->Raw("PairStyle(%s/device,%s<LMPDeviceType>)\n", name.c_str(),
              name_class.c_str())
        ->Raw("PairStyle(%s/host,%s<LMPHostType>)\n", name.c_str(),
              name_class.c_str());
  }
  CodeBuilder* WriteHeaderParam(int arity, const std::string& name,
                                const std::string& type, bool gets_modified) override {
    std::string kk_type = "Kokkos::DualView<" + type + std::string(arity, '*') +
                          ", Kokkos::LayoutRight, DeviceType>";
    std::string kk_const = gets_modified ? "" : "const_";
    return RegularGenerator::WriteHeaderParam(arity, name, type)
        ->Raw("  %s kk_view_%s;\n", kk_type.c_str(), name.c_str())
        ->Raw("  typename %s::t_dev_%sum kk_%s;\n", kk_type.c_str(), kk_const.c_str(), name.c_str());
  }
  CodeBuilder* WriteHeaderClass() override {
    cb_header->Line("template<class DeviceType>");
    return cb_header->Raw("class %s : public Pair, public KokkosBase {\n", name_class.c_str());
  }
  CodeBuilder* WriteHeaderForwardDecls() override {
    int num = 0;
    for (auto& stmt : compute->body) {
      if (! stmt->isa<ForIRStmt>()) continue;
      cb_header->Line("template<int NEIGHFLAG, int EVFLAG>")
          ->RawL("class Tag" + name_class + "Compute%d {};", num)
          ->Line("");
      num += 1;
    }
    return cb_header;
  }
  CodeBuilder* WriteHeaderClassContent() override {
    cb_header->Indent()->Line("typedef DeviceType device_type;")
        ->Line("typedef ArrayTypes<DeviceType> AT;")
        ->Line("typedef EV_FLOAT value_type;")->Undent();
    auto cb = RegularGenerator::WriteHeaderClassContent();
    cb->Indent();
    int num = 0;
    for (auto& stmt : compute->body) {
      if (! stmt->isa<ForIRStmt>()) continue;
      cb->Line("template<int NEIGHFLAG, int EVFLAG>")
        ->Line("KOKKOS_INLINE_FUNCTION")
        ->Line("void operator()(Tag" + name_class +
               "Compute" + std::to_string(num) + "<NEIGHFLAG, EVFLAG>, const int&, EV_FLOAT&) const;")
        ->Line("")
        ->Line("template<int NEIGHFLAG, int EVFLAG>")
        ->Line("KOKKOS_INLINE_FUNCTION")
        ->Line("void operator()(Tag" + name_class +
               "Compute" + std::to_string(num) + "<NEIGHFLAG, EVFLAG>, const int&) const;")
        ->Line("");
      num += 1;
    }
    cb->Line("int inum;")
        ->Line("typename AT::t_x_array_randomread x;")
        ->Line("typename AT::t_f_array f;")
        ->Line("typename AT::t_int_1d_randomread type;")
        ->Line("typename AT::t_tagint_1d tag;")
        ->Line("")
        ->Line("DAT::tdual_efloat_1d k_eatom;")
        ->Line("DAT::tdual_virial_array k_vatom;")
        ->Line("typename ArrayTypes<DeviceType>::t_efloat_1d d_eatom;")
        ->Line("typename ArrayTypes<DeviceType>::t_virial_array d_vatom;")
        ->Line("")
        ->Line("typename AT::t_neighbors_2d d_neighbors;")
        ->Line("typename AT::t_int_1d_randomread d_ilist;")
        ->Line("typename AT::t_int_1d_randomread d_numneigh;")
        ->Line("")
        ->Line("int neighflag,newton_pair;")
        ->Line("int nlocal,nall,eflag,vflag;")
        ->Line("")
        ->Line("friend void pair_virial_fdotr_compute<" + name_class + ">(" +
               name_class + "*);")
        ->Undent();
    return cb;
  }
  CodeBuilder* WriteHeaderIncludes() override {
    return RegularGenerator::WriteHeaderIncludes()->Line(
        "#include \"pair_kokkos.h\"")->Line("#include \"kokkos_base.h\"");
  }
  CodeBuilder* WriteFunctionImpl(const std::string& prefix) override {
    cb_impl->Raw("%s\n", LINE);
    cb_impl->Raw("template<class DeviceType>\n");
    cb_impl->Raw("%s%s<DeviceType>::", prefix.c_str(), name_class.c_str());
    return cb_impl;
  }
  CodeBuilder* WriteConstructor() override {
    return RegularGenerator::WriteConstructor()
        ->Line("atomKK = (AtomKokkos *) atom;")
        ->Line("execution_space = ExecutionSpaceFromDevice<DeviceType>::space;")
        ->Line(
            "datamask_read = X_MASK | F_MASK | TYPE_MASK | ENERGY_MASK | "
            "VIRIAL_MASK;")
        ->Line("datamask_modify = F_MASK | ENERGY_MASK | VIRIAL_MASK;");
  }
  CodeBuilder* WriteDestructor() override {
    return RegularGenerator::WriteDestructor()
        ->Line("memoryKK->destroy_kokkos(k_eatom,eatom);")
        ->Line("memoryKK->destroy_kokkos(k_vatom,vatom); ");
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
        std::string arg = "";
        for (int i = 0; i < array.arity; i++) arg += ", " + array.dims[i];
        cb_impl->Line("kk_view_" + array.name + " = Kokkos::DualView<" + array.type +
               std::string(array.arity, '*') +
               ",Kokkos::LayoutRight, DeviceType>(\"pair_kk:" + array.name + "\"" +
               arg + ");")
        ->Line("kk_" + array.name + " = kk_view_" + array.name +
               ".template view<DeviceType>();");
      }
    }
  }
  CodeBuilder* WriteInitStyle() override {
    return RegularGenerator::WriteInitStyle()
        ->Line("neighbor->requests[irequest]->")
        ->Line(
            "  kokkos_host = "
            "Kokkos::Impl::is_same<DeviceType,LMPHostType>::value &&")
        ->Line("      !Kokkos::Impl::is_same<DeviceType,LMPDeviceType>::value;")
        ->Line("neighbor->requests[irequest]->")
        ->Line(
            "  kokkos_device = "
            "Kokkos::Impl::is_same<DeviceType,LMPDeviceType>::value;");
  }
  CodeBuilder* WriteSettings() override {
    RegularGenerator::WriteSettings();
    std::string np1s;
    cb_impl->Line("int ntypes = atom->ntypes;")->Line(name_class + " *that = this;");
    for (auto& array : arrays) {
      if (array.arity == 0) continue;
      std::string arg = "";
      for (int i = 0; i < array.arity; i++) arg += ", " + array.dims[i];
      //cb_impl->Line("kk_view_" + array.name + " = Kokkos::DualView<double" +
      //       std::string(array.arity, '*') +
      //       ",Kokkos::LayoutRight, DeviceType>(\"pair_kk:" + array.name + "\"" +
      //       arg + ");")
      //->Line("kk_" + array.name + " = kk_view_" + array.name +
      //       ".template view<DeviceType>();");
      std::string kk_idx, reg_idx;
      for (int i = 0; i < array.arity; i++) {
        cb_impl->RawL("for (int i_%1$d = 0; i_%1$d < %2$s; i_%1$d++) {", i, array.dims[i].c_str())->Indent();
        if (i != 0) kk_idx += ", ";
        kk_idx += "i_" + std::to_string(i);
        reg_idx += "[i_" + std::to_string(i) + "]";
      }
      cb_impl->Line("kk_view_" + array.name + ".h_view(" + kk_idx + ") = " + array.name + reg_idx + ";");
      for (int i = 0; i < array.arity; i++) {
        cb_impl->End();
      }
      cb_impl->Line("kk_view_" + array.name +
                    ".template modify<LMPHostType>();");
    }
    return cb_impl;
  }
  CodeBuilder* WriteCompute() override {
    WriteFunctionImpl("void ");
    cb_impl->Fragment("compute(int eflag_in, int vflag_in) {\n")->Indent();
    cb_impl->Line("eflag = eflag_in;");
    cb_impl->Line("vflag = vflag_in;");
    cb_impl->Line("");
    cb_impl->Line("if (neighflag == FULL) no_virial_fdotr_compute = 1;");
    cb_impl->Line("");
    cb_impl->Line("if (eflag || vflag) ev_setup(eflag,vflag,0);");
    cb_impl->Line("else evflag = vflag_fdotr = 0;");
    cb_impl->Line("");
    cb_impl->Line("// reallocate per-atom arrays if necessary");
    cb_impl->Line("");
    cb_impl->Line("if (eflag_atom) {");
    cb_impl->Line("  memoryKK->destroy_kokkos(k_eatom,eatom);");
    cb_impl->Line(
        "  memoryKK->create_kokkos(k_eatom,eatom,maxeatom,\"pair:eatom\");");
    cb_impl->Line("  d_eatom = k_eatom.view<DeviceType>();");
    cb_impl->Line("}");
    cb_impl->Line("if (vflag_atom) {");
    cb_impl->Line("  memoryKK->destroy_kokkos(k_vatom,vatom);");
    cb_impl->Line(
        "  memoryKK->create_kokkos(k_vatom,vatom,maxvatom,6,\"pair:vatom\");");
    cb_impl->Line("  d_vatom = k_vatom.view<DeviceType>();");
    cb_impl->Line("}");
    cb_impl->Line("");
    cb_impl->Line("atomKK->sync(execution_space,datamask_read);");
    for (auto& array : arrays) {
      if (array.arity == 0) continue;
      cb_impl->Line("kk_view_" + array.name + ".template sync<DeviceType>();");
    }
    cb_impl->Line(
        "if (eflag || vflag) "
        "atomKK->modified(execution_space,datamask_modify);");
    cb_impl->Line("else atomKK->modified(execution_space,F_MASK);");
    cb_impl->Line("");
    cb_impl->Line("x = atomKK->k_x.view<DeviceType>();");
    cb_impl->Line("f = atomKK->k_f.view<DeviceType>();");
    cb_impl->Line("type = atomKK->k_type.view<DeviceType>();");
    cb_impl->Line("tag = atomKK->k_tag.view<DeviceType>();");
    cb_impl->Line("nlocal = atom->nlocal;");
    cb_impl->Line("nall = atom->nlocal + atom->nghost;");
    cb_impl->Line("newton_pair = force->newton_pair;");
    cb_impl->Line("");
    if (num_peratom > 0) {
      cb_impl->Raw("  if (atom->nmax > nmax) {\n");
      cb_impl->Raw("    nmax = atom->nmax;\n");
      DeallocateArrays("nmax");
      AllocateArrays("nmax");
      cb_impl->Raw("  }\n");
    }
    cb_impl->Line("inum = list->inum;");
    cb_impl->Line("const int ignum = inum + list->gnum;");
    cb_impl->Line(
        "NeighListKokkos<DeviceType>* k_list = "
        "static_cast<NeighListKokkos<DeviceType>*>(list);");
    cb_impl->Line("d_numneigh = k_list->d_numneigh;");
    cb_impl->Line("d_neighbors = k_list->d_neighbors;");
    cb_impl->Line("d_ilist = k_list->d_ilist;");
    cb_impl->Line("");
    cb_impl->Line("copymode = 1;");
    cb_impl->Line("");
    cb_impl->Line("EV_FLOAT ev_all;");
    cb_impl->Line("");
    {
      int num = 0;
      for (auto& stmt : compute->body) {
        if (! stmt->isa<ForIRStmt>()) {
          WriteKokkosVisitor vis(cb_impl, &ana_comm);
          vis.neigh_is_half = !ana_neigh.need_full;
          stmt->Accept(&vis);
          continue;
        }
        std::string upper;
        if (stmt->asa<ForIRStmt>()->top_value->ToString() ==
               LookupIRExpr(LookupIRExpr::kNumLocal).ToString()) {
          upper = "inum";
        } else if (stmt->asa<ForIRStmt>()->top_value->ToString() ==
               LookupIRExpr(LookupIRExpr::kNumAll).ToString()) {
          upper = "nall";
        } else { assert(0); }
        cb_impl->RawL("EV_FLOAT ev%d;", num);
        cb_impl->Line("if (evflag)");
        cb_impl->RawL(
            "  Kokkos::parallel_reduce(Kokkos::RangePolicy<DeviceType, Tag" +
            name_class + "Compute%1$d<HALFTHREAD,1> >(0,%2$s), *this, ev%1$d);", num, upper.c_str());
        cb_impl->Line("else");
        cb_impl->RawL("  Kokkos::parallel_for(Kokkos::RangePolicy<DeviceType, Tag" +
                      name_class + "Compute%d<HALFTHREAD,0> >(0,%s), *this);", num, upper.c_str());
        cb_impl->RawL("ev_all += ev%d;", num);
        num += 1;
      }
    }
    cb_impl->Line("");
    cb_impl->Line("if (eflag_global) eng_vdwl += ev_all.evdwl;");
    cb_impl->Line("if (vflag_global) {");
    cb_impl->Line("  virial[0] += ev_all.v[0];");
    cb_impl->Line("  virial[1] += ev_all.v[1];");
    cb_impl->Line("  virial[2] += ev_all.v[2];");
    cb_impl->Line("  virial[3] += ev_all.v[3];");
    cb_impl->Line("  virial[4] += ev_all.v[4];");
    cb_impl->Line("  virial[5] += ev_all.v[5];");
    cb_impl->Line("}");
    cb_impl->Line("");
    cb_impl->Line("if (eflag_atom) {");
    cb_impl->Line("  k_eatom.template modify<DeviceType>();");
    cb_impl->Line("  k_eatom.template sync<LMPHostType>();");
    cb_impl->Line("}");
    cb_impl->Line("");
    cb_impl->Line("if (vflag_atom) {");
    cb_impl->Line("  k_vatom.template modify<DeviceType>();");
    cb_impl->Line("  k_vatom.template sync<LMPHostType>();");
    cb_impl->Line("}");
    cb_impl->Line("");
    cb_impl->Line("if (vflag_fdotr) pair_virial_fdotr_compute(this);");
    cb_impl->Line("");
    cb_impl->Line("copymode = 0;");
    {
      int num = 0;
      for (auto& stmt : compute->body) {
        if (! stmt->isa<ForIRStmt>()) continue;
        cb_impl->End();
        WriteFunctionImpl(
            "template<int NEIGHFLAG, int EVFLAG>\nKOKKOS_INLINE_FUNCTION\nvoid ");
        cb_impl
            ->Fragment("operator()(Tag" + name_class +
                       "Compute" + std::to_string(num) + "<NEIGHFLAG, EVFLAG>, const int &ii, EV_FLOAT& "
                       "ev) const {\n")
            ->Indent();
        cb_impl->Line("// The f array is atomic for Half/Thread neighbor style");
        cb_impl->Line(
            "Kokkos::View<F_FLOAT*[3], typename "
            "DAT::t_f_array::array_layout,DeviceType,Kokkos::MemoryTraits<AtomicF<"
            "NEIGHFLAG>::value> > a_f = f;");
        for (auto decl : def->GetDecls()) {
          if (decl->GetKind() != Decl::kPerAtom) continue;
          cb_impl->RawL(
            "Kokkos::View<double*, "
            "Kokkos::LayoutRight,DeviceType,Kokkos::MemoryTraits<AtomicF<"
            "NEIGHFLAG>::value> > a_kk_peratom_%1$s = kk_peratom_%1$s;", decl->GetNameC());
          cb_impl->RawL(
            "Kokkos::View<double*, "
            "Kokkos::LayoutRight,DeviceType,Kokkos::MemoryTraits<AtomicF<"
            "NEIGHFLAG>::value> > a_kk_peratom_adjoint_%1$s = kk_peratom_adjoint_%1$s;", decl->GetNameC());
        }
        WriteKokkosVisitor vis(cb_impl, &ana_comm);
        vis.neigh_is_half = !ana_neigh.need_full;
        stmt->Accept(&vis);
        cb_impl->End();
        WriteFunctionImpl(
            "template<int NEIGHFLAG, int EVFLAG>\nKOKKOS_INLINE_FUNCTION\nvoid ");
        cb_impl
            ->Fragment("operator()(Tag" + name_class +
                       "Compute" + std::to_string(num) + "<NEIGHFLAG, EVFLAG>, const int &ii) const {\n")
            ->Indent();
        cb_impl->Line("EV_FLOAT ev;");
        cb_impl->Line("this->template operator()<NEIGHFLAG,EVFLAG>(Tag" +
                      name_class + "Compute" + std::to_string(num) + "<NEIGHFLAG,EVFLAG>(), ii, ev);");
        num += 1;
      }
    }

    return cb_impl;
  }
  void WriteRegular() override {
    RegularGenerator::WriteRegular();
    cb_impl->Line("namespace LAMMPS_NS {");
    cb_impl->Line("template class " + name_class + "<LMPDeviceType>;");
    cb_impl->Line("#ifdef KOKKOS_HAVE_CUDA");
    cb_impl->Line("template class " + name_class + "<LMPHostType>;");
    cb_impl->Line("#endif");
    cb_impl->Line("}");
  }
  CodeBuilder* WriteImplStart() override {
    return RegularGenerator::WriteImplStart()
        ->Line("#include \"kokkos.h\"")
        ->Line("#include \"atom_kokkos.h\"")
        ->Line("#include \"neigh_list_kokkos.h\"")
        ->Line("#include \"memory_kokkos.h\"")
        ->Line("#include \"atom_masks.h\"");
  }
  void WritePackUnpackDecl() override {
    RegularGenerator::WritePackUnpackDecl();
    cb_header->Raw(
        "  int pack_forward_comm_kokkos(int, DAT::tdual_int_2d, int, DAT::tdual_xfloat_1d&, int, int*);\n");
    cb_header->Raw("  void unpack_forward_comm_kokkos(int, int, DAT::tdual_xfloat_1d&);\n");
    cb_header->Raw("  struct TagPackForward {};\n");
    cb_header->Raw("  struct TagUnpackForward {};\n");
    cb_header->Raw("  KOKKOS_INLINE_FUNCTION void operator() (TagPackForward, const int &i) const;\n");
    cb_header->Raw("  KOKKOS_INLINE_FUNCTION void operator() (TagUnpackForward, const int &i) const;\n");
    cb_header->Line("int iswap;");
    cb_header->Line("int first;");
    cb_header->Line("typename AT::t_int_2d d_sendlist;");
    cb_header->Line("typename AT::t_xfloat_1d_um v_buf;");
  }
  std::string GetPerAtomNameDevice(PerAtomActionIRStmt* stmt) {
    assert(stmt->IsComm());
    if (stmt->IsAdjoint()) return "kk_peratom_adjoint_" + stmt->id + "(j)";
    return "kk_peratom_" + stmt->id + "(j)";
  }
  void WriteForward() override {
    RegularGenerator::WriteForward();
    WriteFunctionImpl("int ");
    cb_impl->Line("pack_forward_comm_kokkos(int n, DAT::tdual_int_2d k_sendlist, int iswap_in, DAT::tdual_xfloat_1d& buf, int pbc_flag, int*pbc) {")->Indent();
    cb_impl->Line("d_sendlist = k_sendlist.view<DeviceType>();");
    cb_impl->Line("iswap = iswap_in;");
    cb_impl->Line("v_buf = buf.view<DeviceType>();");
    cb_impl->Line("Kokkos::parallel_for(Kokkos::RangePolicy<DeviceType, TagPackForward>(0,n),*this);");
    for (int i = 0; i < ana_comm.stage_assignment.size(); i++) {
      if (!ana_comm.IsForward(i)) continue;
      cb_impl->RawL("if (stage_peratom == %d) {", i)
          ->Indent()->Line("return n * " + std::to_string(ana_comm.stage_assignment[i].size()) + ";")->End();
    }
    cb_impl->Line("return n;")->End();
    WriteFunctionImpl("KOKKOS_INLINE_FUNCTION\nvoid ");
    cb_impl->Line("operator () (TagPackForward, const int &i) const {")->Indent();
    cb_impl->Line("int j = d_sendlist(iswap, i);");
    for (int i = 0; i < ana_comm.stage_assignment.size(); i++) {
      if (!ana_comm.IsForward(i)) continue;
      cb_impl->RawL("if (stage_peratom == %d) {", i)
          ->Indent();
      cb_impl->Line("int m = i * " + std::to_string(ana_comm.stage_assignment[i].size()) + ";");
      int idx = 0;
      for (auto decl : ana_comm.stage_assignment[i]) {
        if (!decl->IsForward()) continue;
        cb_impl->Line("v_buf[m + " + std::to_string(idx++) + "] = " + GetPerAtomNameDevice(decl) + ";");
      }
      cb_impl->End();
    }
    cb_impl->End();
    WriteFunctionImpl("void ");
    cb_impl->Line("unpack_forward_comm_kokkos(int n, int first_in, DAT::tdual_xfloat_1d &buf) {")->Indent();
    cb_impl->Line("first = first_in;");
    cb_impl->Line("v_buf = buf.view<DeviceType>();");
    cb_impl->Line("Kokkos::parallel_for(Kokkos::RangePolicy<DeviceType, TagUnpackForward>(0,n),*this);")->End();
    WriteFunctionImpl("KOKKOS_INLINE_FUNCTION\nvoid ");
    cb_impl->Line("operator () (TagUnpackForward, const int &i) const {")->Indent();
    for (int i = 0; i < ana_comm.stage_assignment.size(); i++) {
      if (!ana_comm.IsForward(i)) continue;
      cb_impl->RawL("if (stage_peratom == %d) {", i)
          ->Indent();
      cb_impl->Line("int m = i * " + std::to_string(ana_comm.stage_assignment[i].size()) + ";");
      cb_impl->Line("int j = first + i;");
      int idx = 0;
      for (auto decl : ana_comm.stage_assignment[i]) {
        if (!decl->IsForward()) continue;
        cb_impl->Line(GetPerAtomNameDevice(decl) + " = v_buf[m + " + std::to_string(idx++) + "];");
      }
      cb_impl->End();
    }
    cb_impl->End();
  }
  std::string GetPerAtomName(PerAtomActionIRStmt* stmt) override {
    assert(stmt->IsComm());
    if (stmt->IsAdjoint()) return "kk_view_peratom_adjoint_" + stmt->id + ".h_view(j)";
    return "kk_view_peratom_" + stmt->id + ".h_view(j)";
  }
};

}  // namespace gen

}  // namespace potc

#endif  // GEN_KOKKOS_GENERATOR_H_
