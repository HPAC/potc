// Copyright 2017 Markus Hoehnerbach

#include <unistd.h>
#include <cstdio>
#include <ctime>
#include <map>
#include <memory>
#include <string>
#include "der.h"     // NOLINT
#include "expr.h"    // NOLINT
#include "gen.h"     // NOLINT
#include "ir.h"      // NOLINT
#include "opt.h"     // NOLINT
#include "parser.h"  // NOLINT

namespace {

std::unique_ptr<char[]> slurp_file(const char* filename) {
  char* content = nullptr;
  size_t length;
  FILE* f = fopen(filename, "rb");
  if (f) {
    fseek(f, 0, SEEK_END);
    length = ftell(f);
    fseek(f, 0, SEEK_SET);
    content = new char[length + 1];
    fread(content, 1, length, f);
    content[length] = '\0';
    fclose(f);
  }
  return std::unique_ptr<char[]>(content);
}

// normalize code:
// * no nested CompoundStmts
// * nothing but Assign and Let in If
// * eliminate DeclAcc if a Let could do the job
// * link all IRIdentifiers
// * coalesce multiple accs

double tdiff(std::clock_t from, std::clock_t to) {
  return (to - from) / static_cast<double>(CLOCKS_PER_SEC) * 1000;
}

std::map<char, potc::opt::Pass> RegisterPasses(
    IRIdentifierContext* idc, CompoundIRStmt* program,
    std::vector<std::pair<int, std::unique_ptr<IRExpr>>>* outlined) {
  using namespace potc::opt;
  std::map<char, Pass> passes;
  passes.emplace('d', Pass(idc, DetectDuplicates, program));
  passes.emplace('e', Pass(idc, EliminateUnused, program));
  passes.emplace('f', Pass(idc, FuseLoops, program));
  passes.emplace('F', Pass(idc, FuseConditionals, program));
  passes.emplace('r', Pass(idc, RemoveRedundantDecl, program));
  passes.emplace('a', Pass(idc, ImproveArithmetic, program));
  passes.emplace('i', Pass(idc, FoldInvalid, program));
  passes.emplace('I', Pass(idc, InvariantCodeMotion, program));
  passes.emplace('E', Pass(idc, EliminateUnusedIntegrate, program));
  passes.emplace('S', Pass(idc, ShortenNeighLists, program));
  passes.emplace('T', Pass(idc, Thread, program));
  passes.emplace('V', Pass(idc, Vectorize, program));
  passes.emplace('O', Pass(idc, OutlineParameters, program, outlined));
  passes.emplace('K', Pass(idc, KokkosPushdown, program));
  return passes;
}

std::string MulString(int n, const std::string& s) {
  std::string ret = "";
  for (int i = 0; i < n; i++) {
    ret += s;
  }
  return ret;
}

std::string Filter(std::string src, std::string lookup, bool only_found) {
  std::string ret;
  for (auto from_src : src) {
    bool found = false;
    for (auto from_lookup : lookup) {
      if (from_src == from_lookup) {
        found = true;
      }
    }
    if (found == only_found) {
      ret += from_src;
    }
  }
  return ret;
}

std::string GetPassProgram(bool mode_all, std::string modifier) {
  std::string essential = "edfFri";
  std::string all = essential + "EOSIa";
  std::string allowed;
  if (mode_all) {
    allowed = Filter(all, modifier, false);
  } else {
    allowed = essential + modifier;
  }
  std::string pass_program = MulString(2, MulString(4, "fFederaiI") + "EdE");
  pass_program = "ededeIdedeI" + pass_program + pass_program + "O" +
                 pass_program + "S" + pass_program;
  return Filter(pass_program, allowed, true);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    printf("Usage: %s <potc-file> <pot-name> [<variants>]\n", argv[0]);
    return EXIT_SUCCESS;
  }
  std::clock_t t_start = std::clock();
  auto content = slurp_file(argv[1]);
  std::clock_t t_slurp = std::clock();
  if (!content) {
    printf("Error: Could not read file %s.\n", argv[1]);
    return EXIT_FAILURE;
  }
  auto def = potc::Parse(content.get());
  std::clock_t t_parse = std::clock();
  if (def.get() == nullptr) return -1;
  IRIdentifierContext idc;
  auto program = potc::der::GenerateCode(def.get(), &idc);
  std::clock_t t_gen = std::clock();
  std::vector<std::pair<int, std::unique_ptr<IRExpr>>> outlined;
  std::map<char, potc::opt::Pass> passes =
      RegisterPasses(&idc, program.get(), &outlined);
  bool opt_show_each = false;
  bool opt_check_legal = true;
  for (auto& p : passes) {
    p.second.option_show_each_time = opt_show_each;
    p.second.option_check_legal = opt_check_legal;
  }
  auto gen_name = std::string(argv[2]) + "/gen";
  bool write_reg = false, write_kokkos = false, write_intel = false;
  std::string variants = argc >= 4 ? argv[3] : "rki";
  bool opt = true, mode_all = true;
  for (char c : variants) {
    if (c == 'r') write_reg = true;
    if (c == 'k') write_kokkos = true;
    if (c == 'i') write_intel = true;
    if (c == 'u') opt = false;
    if (c == 'E') mode_all = false;
  }
  auto pass_program = GetPassProgram(mode_all, argc >= 5 ? argv[4] : "");
  printf("opt len: %zu\n", pass_program.size());
  if (getenv("POTC_PASS_LIMIT")) {
    pass_program = pass_program.substr(0, atoi(getenv("POTC_PASS_LIMIT")));
    printf("pass program: %s\n", pass_program.c_str());
  }
  if (opt) {
    for (char p : pass_program) {
      try {
        passes.at(p).Execute();
      } catch (int _) {
        break;
      }
    }
  }
  for (auto& p : passes)
    printf("%c %p %f\n", p.first, p.second.pass_ptr, p.second.msec_time);
  std::clock_t t_opt = std::clock();
  if (write_reg)
    potc::gen::WriteRegular(gen_name, program.get(), def.get(), &outlined);
  if (write_kokkos) {
    passes.at('K').Execute();
    potc::gen::WriteKokkos(gen_name, program.get(), def.get(), &outlined);
  }
  if (write_intel) {
    passes.at('T').Execute();
    passes.at('V').Execute();
    potc::gen::WriteIntel(gen_name, program.get(), def.get(), &outlined);
  }
  std::clock_t t_write = std::clock();
  printf("slurp %f ", tdiff(t_start, t_slurp));
  printf("parse %f der %f opt %f out %f\n", tdiff(t_slurp, t_parse),
         tdiff(t_parse, t_gen), tdiff(t_gen, t_opt), tdiff(t_opt, t_write));
  return 0;
}
