#include "opt.h"

#include "ir_visitor.h"

namespace potc {

namespace opt {

namespace {

// at each loop
// declacc widen
// accumulate to reduce
// force as vectorize

// give IR flag
// if accumulator is encountered, widen
// if `for` is encountered, chunk and add remainder loop
// below that:
// - if `if` is encountered, generate mask, check it and set it as `current
// mask` for everything below
// - if `assign`/`acc` is encountered, respect current mask
// - if `for` is encountered, handle as normal
// - if `continue` is encountered, update and check current mask,
//   or alternatively update surrounding index and go back to top, if no side
//   effect has occurred
// - if `force accumulate` is encountered, do a conflict check, and do fast/slow
// respectively as needed
// - if `energy accumulate` is encountered, perform reduction on the right

struct VectorizeVisitor : public IRExprVisitor, public IRStmtVisitor {
  bool in_vectorized_region;

  VectorizeVisitor() : in_vectorized_region(false) {}
  void Visit(CompoundIRStmt *) override {
    // can add stmts
  }
  void Visit(LetIRStmt *) override {}
  void Visit(DeclAssignIRStmt *) override {}
  void Visit(AssignIRStmt *) override {}
  void Visit(DeclAccIRStmt *) override {
    // stmt->is_vector = true;
  }
  void Visit(AccIRStmt *) override {}
  void Visit(AccForceIRStmt *) override {
    // do vpconflict code
  }
  void Visit(IfIRStmt *) override {
    //
  }
  void Visit(ForIRStmt *) override {
    if (in_vectorized_region) {
    } else {
      in_vectorized_region = true;

      in_vectorized_region = false;
    }
  }
  void Visit(ContinueIRStmt *) override {}
  void Visit(AccEnergyIRStmt *) override {
    // reduction of RHS
  }
  void Visit(InvalidIRStmt *) override {}
  void Visit(LetComplexFunCallIRStmt *) override {
    // serialize
  }
};

}  // namespace

void Vectorize(CompoundIRStmt *program) {
  // VectorizeVisitor vis;
  // vis.Visit(program);
}

}  // namespace opt

}  // namespace potc
