#include "opt.h"

#include <map>
#include <memory>

#include "ir.h"
#include "ir_visitor_empty.h"

namespace potc {

namespace opt {

struct KokkosPushdownVisitor : public TraverseIRVisitor {
  IRIdentifierContext *idc;
  std::map<IRIdentifier, IRIdentifier> renames;
  bool IsPushdown(const std::unique_ptr<IRStmt> &stmt) {
    if (stmt->isa<LetIRStmt>()) {
      return true;
    } else if (stmt->isa<DeclAccIRStmt>()) {
      return true;
    } else if (stmt->isa<AccEnergyIRStmt>()) {
      return true;
    } else if (stmt->isa<ForIRStmt>()) {
      return false;
    } else if (stmt->isa<PerAtomActionIRStmt>()) {
      return false;
    } else {
      assert(0);
    }
  }
  IRIdentifier Subject(const std::unique_ptr<IRStmt> &stmt) {
    if (stmt->isa<LetIRStmt>()) {
      return stmt->asa<LetIRStmt>()->name;
    } else if (stmt->isa<DeclAccIRStmt>()) {
      return stmt->asa<DeclAccIRStmt>()->name;
    }
    return IRIdentifier::Invalid("No Subject");
  }
  void Pushdown(CompoundIRStmt* stmt) {
    for (int i = 0; i < stmt->body.size(); i++) {
      ForIRStmt* for_stmt = stmt->body[i]->asa<ForIRStmt>();
      if (! for_stmt) continue;
      int at = 0;
      auto& into = for_stmt->body->body;
      for (int j = 0; j < i; j++) {
        if (! IsPushdown(stmt->body[j])) continue;
        into.insert(into.begin() + at, stmt->body[j]->Clone());
        renames[Subject(stmt->body[j])] = idc->Next("pd");
        at += 1;
      }
      for (int j = i + 1; j < stmt->body.size(); j++) {
        if (! IsPushdown(stmt->body[j])) continue;
        into.insert(into.end(), stmt->body[j]->Clone());
        renames[Subject(stmt->body[j])] = idc->Next("pd");
      }
      Visit(for_stmt);
      renames.clear();
    }
    for (int i = 0; i < stmt->body.size(); i++) {
      auto& child = stmt->body[i];
      if (IsPushdown(child)) {
        stmt->body.erase(stmt->body.begin() + i);
        i -= 1;
      }
    }
  }
  void Rename(IRIdentifier *needle) {
    if (renames.count(*needle)) {
      *needle = renames[*needle];
    }
  }
  using TraverseIRVisitor::Visit;
  void Visit(LetIRStmt *stmt) override {
    AcceptExpr(&stmt->value);
    Rename(&stmt->name);
  }
  void Visit(DeclAccIRStmt *stmt) override {
    Rename(&stmt->name);
  }
  void Visit(AccIRStmt *stmt) override {
    AcceptExpr(&stmt->value);
    Rename(&stmt->name);
  }
  void Visit(RefIRExpr *expr) override {
    Rename(&expr->ref);
  }
};

void KokkosPushdown(CompoundIRStmt* program, IRIdentifierContext* idc) {
  KokkosPushdownVisitor vis;
  vis.idc = idc;
  vis.Pushdown(program);
  // all lets are moved into the outer ``for'' stmts
  // all accumulator decls are checked that they accumulate into energy
  // then they are eliminated and replaced by energy accumulation statements inside
}

}

}
