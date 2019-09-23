// Copyright 2018 Markus Hoehnerbach

#include "opt.h"

#include <map>

#include "ir.h"
#include "ir_visitor_empty.h"

namespace potc {

namespace opt {

namespace {

struct FoldInvalidVisitor : public TraverseIRVisitor {
  void Visit(CompoundIRStmt* stmt) override {
    TraverseIRVisitor::Visit(stmt);
    if (stmt->body.empty()) return;
    if (stmt->body[0]->isa<InvalidIRStmt>()) stmt->body.resize(1);
  }
};

}  // namespace

void FoldInvalid(CompoundIRStmt* program) {
  FoldInvalidVisitor vis;
  program->Accept(&vis);
}

}  // namespace opt

}  // namespace potc
