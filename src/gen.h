#ifndef GEN_H_
#define GEN_H_ GEN_H_

#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include <vector>

class CompoundIRStmt;
class PotentialDef;
class IRExpr;

namespace potc {

namespace gen {

inline int PowInt(int base, int exp) {
  int ret = 1;
  for (int i = 0; i < exp; i++) {
    ret *= base;
  }
  return ret;
}

void WriteIntel(std::string, CompoundIRStmt* compute, PotentialDef* def,
                std::vector<std::pair<int, std::unique_ptr<IRExpr>>>* outlined);

void WriteRegular(
    std::string, CompoundIRStmt* compute, PotentialDef* def,
    std::vector<std::pair<int, std::unique_ptr<IRExpr>>>* outlined);

void WriteKokkos(
    std::string name, CompoundIRStmt* compute, PotentialDef* def,
    std::vector<std::pair<int, std::unique_ptr<IRExpr>>>* outlined);

}  // namespace gen

}  // namespace potc

#endif  // GEN_H_
