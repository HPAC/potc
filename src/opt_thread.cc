#include "opt.h"

#include "ir_build.h"
#include "ir_visitor_empty.h"

namespace potc {

namespace opt {

namespace {

// Modify outer loops to go from certain to certain elems
// If needed insert barriers
struct ThreaderVisitor : public EmptyIRStmtVisitor {
  void Visit(CompoundIRStmt* stmt) override {
    for (auto& c : stmt->body) {
      c->Accept(this);
    }
  }
  void Visit(ForIRStmt* stmt) override {
    using namespace potc::ir::build;
    // Check:
    // Outer loop starts at 0, goes to nlocal or nghost
    // Heuristic:
    // - If it only affects energy and for calculation
    //   it can be threaded
    // - Under all other circumstances, it can't
    // Question:
    // Do we transform the loop here and then, or do we
    // wait until code generation.
    // I.e. attach something like
    //  "this loop is to be executed by one thread"
    //  "this loop needs a barrier after it"
    //  "this loop does not need a barrier after it"
    // Maybe:
    //  Have a list of attributes, and each IRNode carries these around
    // Or just have them as part of the node:
    // ForIRExpr.thread_mode = none/single/barrier/full;
    LitIRExpr* initial_expr = stmt->initial->asa<LitIRExpr>();
    assert(initial_expr && initial_expr->value == "0");
    LookupIRExpr* lookup_expr = stmt->top_value->asa<LookupIRExpr>();
    if (lookup_expr && lookup_expr->lookup_kind == LookupIRExpr::kNumLocal) {
      stmt->initial =
          std::make_unique<LookupIRExpr>(LookupIRExpr::kThreadNumLocalFrom);
      stmt->top_value =
          std::make_unique<LookupIRExpr>(LookupIRExpr::kThreadNumLocalTo);
      stmt->thread_mode = IRThreadMode::kFull;
      return;
    }

    if (lookup_expr && lookup_expr->lookup_kind == LookupIRExpr::kNumAll) {
      stmt->initial =
          std::make_unique<LookupIRExpr>(LookupIRExpr::kThreadNumAllFrom);
      stmt->top_value =
          std::make_unique<LookupIRExpr>(LookupIRExpr::kThreadNumAllTo);
      stmt->thread_mode = IRThreadMode::kFull;
    }
  }
};

}  // namespace

void Thread(CompoundIRStmt* program) {
  ThreaderVisitor vis;
  vis.Visit(program);
}

}  // namespace opt

}  // namespace potc
