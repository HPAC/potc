// Copyright 2018 Markus Hoehnerbach

#ifndef OPT_H_
#define OPT_H_ OPT_H_

#include <cstdio>
#include <ctime>
#include <memory>
#include <utility>
#include <vector>

class CompoundIRStmt;
class IRIdentifierContext;
class IRExpr;

namespace potc {

namespace opt {

struct PassContext {
  std::vector<std::pair<int, std::unique_ptr<IRExpr>>>* outlined;
  IRIdentifierContext* context;
  CompoundIRStmt* program;
};

void EliminateUnused(CompoundIRStmt* program);
void EliminateUnusedIntegrate(CompoundIRStmt* program);
void DetectDuplicates(CompoundIRStmt* program, IRIdentifierContext* ctx);
void FuseLoops(CompoundIRStmt* program);
void FuseConditionals(CompoundIRStmt* program);
void RemoveRedundantDecl(CompoundIRStmt* program);
void ImproveArithmetic(CompoundIRStmt* program);
void CheckLegal(CompoundIRStmt* program);
void Thread(CompoundIRStmt* program);
void Vectorize(CompoundIRStmt* program);
void ShortenNeighLists(CompoundIRStmt* program, IRIdentifierContext* ctx);
void FoldInvalid(CompoundIRStmt* program);
void InvariantCodeMotion(CompoundIRStmt* program);
void OutlineParameters(PassContext* ctx);
void KokkosPushdown(CompoundIRStmt* program, IRIdentifierContext* ctx);

// Measure time
// Check if legal
// Check if modification occurred
// Has the correct IRIdentifierContext
struct Pass {
  double msec_time;
  bool option_show_each_time;
  bool option_check_legal;
  void (*pass_ptr1)(CompoundIRStmt* program);
  void (*pass_ptr2)(CompoundIRStmt* program, IRIdentifierContext* irc);
  void (*pass_ptr3)(PassContext* ctx);
  void* pass_ptr;
  IRIdentifierContext* context;
  CompoundIRStmt* program;
  std::vector<std::pair<int, std::unique_ptr<IRExpr>>>* outlined;
  Pass(IRIdentifierContext* ctx, void (*ptr)(CompoundIRStmt*),
       CompoundIRStmt* pr)
      : context(ctx),
        pass_ptr1(ptr),
        pass_ptr2(nullptr),
        pass_ptr3(nullptr),
        pass_ptr(reinterpret_cast<void*>(ptr)),
        program(pr),
        option_check_legal(true),
        option_show_each_time(false),
        msec_time(0) {}
  Pass(IRIdentifierContext* ctx,
       void (*ptr)(CompoundIRStmt*, IRIdentifierContext*), CompoundIRStmt* pr)
      : context(ctx),
        pass_ptr1(nullptr),
        pass_ptr2(ptr),
        pass_ptr3(nullptr),
        pass_ptr(reinterpret_cast<void*>(ptr)),
        program(pr),
        option_check_legal(true),
        option_show_each_time(false),
        msec_time(0) {}
  Pass(IRIdentifierContext* ctx, void (*ptr)(PassContext* ctx),
       CompoundIRStmt* pr,
       std::vector<std::pair<int, std::unique_ptr<IRExpr>>>* o)
      : context(ctx),
        outlined(o),
        pass_ptr1(nullptr),
        pass_ptr2(nullptr),
        pass_ptr3(ptr),
        pass_ptr(reinterpret_cast<void*>(ptr)),
        program(pr),
        option_check_legal(true),
        option_show_each_time(false),
        msec_time(0) {}
  double tdiff(std::clock_t from, std::clock_t to) {
    return (to - from) / static_cast<double>(CLOCKS_PER_SEC) * 1000;
  }
  void Execute() {
    std::clock_t t_start = std::clock();
    if (pass_ptr1) pass_ptr1(program);
    if (pass_ptr2) pass_ptr2(program, context);
    PassContext ctx = {outlined, context, program};
    if (pass_ptr3) pass_ptr3(&ctx);
    std::clock_t t_end = std::clock();
    double t = tdiff(t_start, t_end);
    if (option_show_each_time) printf("[PASS TIME] %p %f\n", pass_ptr, t);
    msec_time += t;
    if (option_check_legal) CheckLegal(program);
  }
};

}  // namespace opt
}  // namespace potc

#endif  // OPT_H_
