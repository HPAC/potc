#include "gen.h"

#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "der.h"
#include "expr.h"
#include "gen_intel_generator.h"
#include "gen_intel_visitor.h"
#include "gen_internal.h"
#include "gen_kokkos_generator.h"
#include "gen_kokkos_visitor.h"
#include "gen_regular_generator.h"
#include "gen_regular_visitor.h"
#include "gen_spline.h"
#include "ir.h"
#include "ir_visitor.h"
#include "ir_visitor_empty.h"

// Write out function
// Pretty print the expressions
// Write out routine to read parameters
// Write out parameters in header file

namespace potc {

namespace gen {

void WriteRegular(
    std::string name, CompoundIRStmt* compute, PotentialDef* def,
    std::vector<std::pair<int, std::unique_ptr<IRExpr>>>* outlined) {
  RegularGenerator gen(name, compute, def, outlined);
  gen.WriteRegular();
  gen.Close();
}

void WriteIntel(
    std::string name, CompoundIRStmt* compute, PotentialDef* def,
    std::vector<std::pair<int, std::unique_ptr<IRExpr>>>* outlined) {
  IntelGenerator gen(name, compute, def, outlined);
  gen.WriteRegular();
  gen.Close();
}

void WriteKokkos(
    std::string name, CompoundIRStmt* compute, PotentialDef* def,
    std::vector<std::pair<int, std::unique_ptr<IRExpr>>>* outlined) {
  KokkosGenerator gen(name, compute, def, outlined);
  gen.WriteRegular();
  gen.Close();
}

}  // namespace gen

}  // namespace potc
