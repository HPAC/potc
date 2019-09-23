#ifndef GEN_SPLINE_H_
#define GEN_SPLINE_H_ GEN_SPLINE_H_

#include <map>
#include <string>
#include <vector>

#include "expr.h"
#include "gen.h"
#include "gen_internal.h"

namespace potc {

namespace gen {

std::vector<std::vector<std::vector<int>>> GetSplineFittingMatrix(
    int order, int dim, const std::vector<std::vector<int>> derivatives);

inline std::string SplineSize(ParameterDecl* param, int pos) {
  return "spline_size_" + std::to_string(pos) + "_" + param->GetName();
}

inline void WriteSplineEvalArgs(CodeBuilder* cb, ParameterDecl* param) {
  for (int i = 0; i < param->GetNumArgs(); i++) {
    if (i != 0) cb->Fragment(", ");
    if (param->GetArgumentType(i) == Type::kAtomType) {
      cb->Fragment("int");
    } else {
      cb->Fragment("double");
    }
    cb->Raw(" arg_%d", i);
  }
  for (int i = 0; i < param->GetNumArgs(); i++) {
    if (param->GetArgumentType(i) == Type::kAtomType) continue;
    cb->Raw(", double *d_%d", i);
  }
}

inline void WriteSplineZero(CodeBuilder* cb_impl, ParameterDecl* param) {
  auto name_str = param->GetName();
  const char* name = name_str.c_str();
  cb_impl->Raw("  spline_allocated_%s = 0;\n", name);
}

// void WriteSplineAlloc(CodeBuilder* cb_impl, ParameterDecl* param) {
//   int num_args = param->GetArguments().size();
//   auto name_str = param->GetName();
//   auto name = name_str.c_str();
//   SplineParameterSpec* spec =
//       static_cast<SplineParameterSpec*>(param->spec.get());
//   if (! spec->is_grid && ! spec->is_integer) {
//     for (int i = 0; i < num_args; i++) {
//       cb_impl->Raw(
//           "  memory->create(spline_limits_%1$d_%2$s, %3$s, "
//           "\"pair:spline:limit:%2$s\");\n",
//           i, name, SplineSize(param, i).c_str());
//     }
//   }
//   cb_impl->Raw("  memory->create(spline_nodes_%s", name);
//   for (int i = 0; i < num_args; i++) {
//     cb_impl->Fragment(", ")->Fragment(SplineSize(param, i));
//   }
//   cb_impl->Raw(", %zu", spec->derivatives.size() + 1);
//   cb_impl->Raw(", \"pair:spline:nodes:%s\");\n", name);
//   cb_impl->Raw("  memory->create(spline_coeffs_%s", name);
//   for (int i = 0; i < num_args; i++) {
//     cb_impl->Fragment(", ")->Fragment(SplineSize(param, i));
//   }
//   cb_impl->Raw(", %d", potc::gen::PowInt(spec->order + 1, num_args));
//   cb_impl->Raw(", \"pair:spline:coeffs:%s\");\n", name);
// }

}  // namespace gen

}  // namespace potc

#endif  // GEN_SPLINE_H_
