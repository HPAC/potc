// Copyright 2018 Markus Hoehnerbach

#include "opt.h"

#include <algorithm>
#include <map>
#include <set>
#include <vector>

#include "ir.h"
#include "ir_build.h"
#include "ir_visitor_empty.h"

namespace potc {

namespace opt {

namespace {

template <typename TheIRStmt>
struct Occurrence {
  CompoundIRStmt* parent;
  TheIRStmt* stmt;
  bool operator<(const Occurrence& other) const {
    if (other.parent == parent) {
      return stmt->value->ToString() < other.stmt->value->ToString();
    }
    return parent < other.parent;
  }
};

void RemoveFrom(CompoundIRStmt* parent, IRStmt* target) {
  for (int i = 0; i < parent->body.size(); i++) {
    if (parent->body[i].get() == target) {
      parent->body.erase(parent->body.begin() + i);
      return;
    }
  }
}

int FindPositionIn(CompoundIRStmt* parent, IRStmt* target) {
  for (int i = 0; i < parent->body.size(); i++) {
    if (parent->body[i].get() == target) {
      return i;
    }
  }
  assert(0);
}

class RemoveRedundantDeclVisitor : public TraverseIRVisitor {
  std::map<IRIdentifier, std::vector<Occurrence<AccIRStmt>>> acc;
  std::map<IRIdentifier, std::vector<Occurrence<AssignIRStmt>>> ass;
  std::vector<std::vector<DeclAccIRStmt*>> decl_acc;
  std::vector<std::vector<DeclAssignIRStmt*>> decl_ass;
  std::vector<CompoundIRStmt*> parent;
  std::map<IRIdentifier, IRIdentifier> renames;
  bool just_rename;

  void AttemptAccToAdd(CompoundIRStmt* parent, DeclAccIRStmt* dest,
                       std::vector<Occurrence<AccIRStmt>> occs) {
    using namespace potc::ir::build;
    using namespace potc::ir::build::stmt;

    int last = -1;
    // Sort by position in parent
    for (auto& occ : occs) {
      if (occ.parent != parent) return;
      last = std::max(last, FindPositionIn(parent, occ.stmt));
    }
    if (last == -1) {
      assert(occs.size() == 0);
      last = FindPositionIn(parent, dest);
      parent->body[last] = Let(dest->name, Lit("0"));
      return;
    }
    std::unique_ptr<IRExpr> expr = Lit("0");
    for (auto& occ : occs) {
      expr = Add(std::move(expr), std::move(occ.stmt->value));
    }
    IRStmt* overwritten = parent->body[last].get();
    // Find last occ, replace
    parent->body[last] = Let(dest->name, std::move(expr));
    // Remove all others
    RemoveFrom(parent, dest);
    for (auto& occ : occs) {
      if (occ.stmt != overwritten) RemoveFrom(parent, occ.stmt);
    }
  }

  template <typename TheDeclIRStmt, typename TheIRStmt>
  bool Merge(CompoundIRStmt* parent, TheDeclIRStmt* dest, TheDeclIRStmt* src,
             std::vector<Occurrence<TheIRStmt>> dest_occ,
             std::vector<Occurrence<TheIRStmt>> src_occ) {
    // check if same amount of occurences.
    if (src_occ.size() != dest_occ.size()) return false;
    // sort by compoundirstmt
    std::sort(dest_occ.begin(), dest_occ.end());
    std::sort(src_occ.begin(), src_occ.end());
    // iterate through both concurrently, checking both match in parent and expr
    for (int i = 0; i < dest_occ.size(); i++) {
      if (dest_occ[i].parent != src_occ[i].parent) return false;
      if (dest_occ[i].stmt->value->ToString() !=
          src_occ[i].stmt->value->ToString())
        return false;
    }
    // if so, remove decl and ass of j, then add appropriate rename
    renames[src->name] = dest->name;
    RemoveFrom(parent, src);
    for (auto& occ : src_occ) RemoveFrom(occ.parent, occ.stmt);
    return true;
  }

 public:
  RemoveRedundantDeclVisitor() : just_rename(false) {}
  void Visit(CompoundIRStmt* stmt) override {
    if (just_rename) {
      for (auto& child : stmt->body) {
        child->Accept(this);
      }
      return;
    }
    decl_acc.push_back({});
    decl_ass.push_back({});
    parent.push_back(stmt);
    for (auto& child : stmt->body) {
      child->Accept(this);
    }
    for (int i = 0; i < decl_acc.back().size(); i++) {
      for (int j = 0; j < decl_acc.back().size(); j++) {
        if (i == j) continue;
        if (Merge(stmt, decl_acc.back()[i], decl_acc.back()[j],
                  acc[decl_acc.back()[i]->name],
                  acc[decl_acc.back()[j]->name])) {
          decl_acc.back().erase(decl_acc.back().begin() + j);
          j -= 1;
        }
      }
    }
    for (int i = 0; i < decl_ass.back().size(); i++) {
      for (int j = 0; j < decl_ass.back().size(); j++) {
        if (i == j) continue;
        if (Merge(stmt, decl_ass.back()[i], decl_ass.back()[j],
                  ass[decl_ass.back()[i]->name],
                  ass[decl_ass.back()[j]->name])) {
          decl_ass.back().erase(decl_ass.back().begin() + j);
          j -= 1;
        }
      }
    }
    for (auto& decl : decl_acc.back()) {
      AttemptAccToAdd(stmt, decl, acc[decl->name]);
    }
    just_rename = true;
    for (auto& child : stmt->body) {
      child->Accept(this);
    }
    just_rename = false;
    parent.pop_back();
    decl_acc.pop_back();
    decl_ass.pop_back();
  }
  void Visit(DeclAssignIRStmt* stmt) override {
    if (!just_rename) decl_ass.back().push_back(stmt);
  }
  void Visit(AssignIRStmt* stmt) override {
    stmt->value->Accept(this);
    if (!just_rename)
      ass[stmt->name].push_back(Occurrence<AssignIRStmt>{parent.back(), stmt});
  }
  void Visit(DeclAccIRStmt* stmt) override {
    if (!just_rename) decl_acc.back().push_back(stmt);
  }
  void Visit(AccIRStmt* stmt) override {
    stmt->value->Accept(this);
    if (!just_rename)
      acc[stmt->name].push_back(Occurrence<AccIRStmt>{parent.back(), stmt});
  }
  void Visit(RefIRExpr* expr) {
    if (renames.count(expr->ref) > 0) {
      expr->ref = renames[expr->ref];
    }
  }
};

}  // namespace

void RemoveRedundantDecl(CompoundIRStmt* program) {
  RemoveRedundantDeclVisitor vis;
  vis.Visit(program);
  // use local value numberin to identify candidates
  // compare candidates for equality modulo associativity
  // replace usages using translation table
  // traverse code forwards
}
}  // namespace opt
}  // namespace potc
