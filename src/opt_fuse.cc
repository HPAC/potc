// Copyright 2018 Markus Hoehnerbach

#include "opt.h"

#include <map>
#include <set>
#include <vector>

#include "ir.h"
#include "ir_visitor_empty.h"

namespace potc {

namespace opt {

namespace {

const bool DEBUG = false;

int if_count = 0;

struct FindUsedModifiedVisitor : public TraverseIRVisitor {
  std::set<IRIdentifier> modified;   // these are modified in the IfIRStmt
  std::set<IRIdentifier> used;       // these are used in the IfIRStmt
  std::set<IRIdentifier> used_comm;  // these are used in the IfIRStmt

  void Visit(LetIRStmt* stmt) override {
    stmt->value->Accept(this);
    modified.insert(stmt->name);
  }
  void Visit(DeclAssignIRStmt* stmt) override { modified.insert(stmt->name); }
  void Visit(AssignIRStmt* stmt) override {
    stmt->value->Accept(this);
    modified.insert(stmt->name);
    used.insert(stmt->name);
  }
  void Visit(DeclAccIRStmt* stmt) override { modified.insert(stmt->name); }
  void Visit(AccIRStmt* stmt) override {
    stmt->value->Accept(this);
    // TODO(markus) Handle commutativity properly!
    modified.insert(stmt->name);
    used_comm.insert(stmt->name);
  }
  void Visit(LetComplexFunCallIRStmt* stmt) override {
    for (auto& arg : stmt->args) {
      arg->Accept(this);
    }
    modified.insert(stmt->name);
    for (auto& n : stmt->other_names) {
      modified.insert(n);
    }
  }
  void Visit(RefIRExpr* expr) override { used.insert(expr->ref); }

  static const int PERATOM_FLAG = -10;
  static const int PERATOM_ADJOINT_FLAG = -11;
  void Visit(LookupIRExpr* expr) override {
    if (expr->lookup_kind == LookupIRExpr::kPerAtom) {
      used.insert(IRIdentifier(PERATOM_FLAG, expr->id));
    } else if (expr->lookup_kind == LookupIRExpr::kPerAtomAdjoint) {
      used.insert(IRIdentifier(PERATOM_ADJOINT_FLAG, expr->id));
    }
    TraverseIRVisitor::Visit(expr);
  }
  void Visit(PerAtomActionIRStmt* stmt) override {
    if (stmt->kind == PerAtomActionIRStmt::kPerAtomAcc) {
      used_comm.insert(IRIdentifier(PERATOM_FLAG, stmt->id));
      modified.insert(IRIdentifier(PERATOM_FLAG, stmt->id));
    } else if (stmt->kind == PerAtomActionIRStmt::kPerAtomAdjointAcc) {
      used_comm.insert(IRIdentifier(PERATOM_ADJOINT_FLAG, stmt->id));
      modified.insert(IRIdentifier(PERATOM_ADJOINT_FLAG, stmt->id));
    } else if (stmt->kind == PerAtomActionIRStmt::kPerAtomZero) {
      used.insert(IRIdentifier(PERATOM_FLAG, stmt->id));
      modified.insert(IRIdentifier(PERATOM_FLAG, stmt->id));
    } else if (stmt->kind == PerAtomActionIRStmt::kPerAtomAdjointZero) {
      used.insert(IRIdentifier(PERATOM_ADJOINT_FLAG, stmt->id));
      modified.insert(IRIdentifier(PERATOM_ADJOINT_FLAG, stmt->id));
    } else if (stmt->kind == PerAtomActionIRStmt::kPerAtomComm) {
      used.insert(IRIdentifier(PERATOM_FLAG, stmt->id));
      modified.insert(IRIdentifier(PERATOM_FLAG, stmt->id));
    } else if (stmt->kind == PerAtomActionIRStmt::kPerAtomAdjointComm) {
      used.insert(IRIdentifier(PERATOM_ADJOINT_FLAG, stmt->id));
      modified.insert(IRIdentifier(PERATOM_ADJOINT_FLAG, stmt->id));
    } else {
      assert(0);
    }
    TraverseIRVisitor::Visit(stmt);
  }
};

void MoveCodeBefore(CompoundIRStmt* within, IRStmt* before, IRStmt* src) {
  size_t pos_before = -1, pos_src = -1, pos = 0;
  // 1. find each one's position
  for (auto& stmt : within->body) {
    if (stmt.get() == before) pos_before = pos;
    if (stmt.get() == src) pos_src = pos;
    pos += 1;
  }
  // 2. check that currently after
  if (pos_before > pos_src) return;
  // 3. move out at that position
  auto src_tmp = std::move(within->body[pos_src]);
  // 4. erase at that position
  within->body.erase(within->body.begin() + pos_src);
  // 5. insert at new position
  within->body.insert(within->body.begin() + pos_before, std::move(src_tmp));
}

bool MergeIf(CompoundIRStmt* parent, IfIRStmt* dest, IfIRStmt* src) {
  if (dest->cond->ToString() != src->cond->ToString()) return false;
  // static int if_limit = atoi(getenv("IF_LIMIT"));
  // if (! if_limit) return false;
  FindUsedModifiedVisitor src_um;
  src->Accept(&src_um);
  FindUsedModifiedVisitor dest_um;
  dest->Accept(&dest_um);
  int pos_src = -1, pos_dest = -1;
  for (size_t pos = 0; pos < parent->body.size(); pos++) {
    if (parent->body[pos].get() == dest) pos_dest = pos;
    if (parent->body[pos].get() == src) pos_src = pos;
  }
  assert(pos_src != -1);
  assert(pos_dest != -1);
  assert(pos_src != pos_dest);
  std::vector<IRStmt*> src_dep;
  if (pos_src < pos_dest) {
    // source is in front of destination.
    // i.e. the question is: "does any of the code in between depend on any of
    // the data computed in src?
    // accordingly, collect ``used'' from steps in between as ``dest'' used.
    // no dependent code to move, but need to augment dest's
    for (size_t pos = pos_src + 1; pos < pos_dest; pos++) {
      parent->body[pos]->Accept(&dest_um);
    }
  } else if (pos_src > pos_dest) {
    // source comes after the destination.
    // potentially (certainly) need to copy over some code.
    // compute set of these lines into src_dep.
    // proceed in *reverse*, add line if it declares/modifies a variable used
    // in src_um.
    for (size_t pos = pos_src - 1; pos > pos_dest; pos--) {
      FindUsedModifiedVisitor um;
      parent->body[pos]->Accept(&um);
      // if it's ``modified'' is used within our code
      for (auto& mod : um.modified) {
        if (src_um.used.count(mod) > 0 || src_um.used_comm.count(mod) > 0) {
          // add uses into src_um
          for (auto& use : um.used) {
            src_um.used.insert(use);
          }
          for (auto& use : um.used_comm) {
            src_um.used_comm.insert(use);
          }
          for (auto& use : um.modified) {
            src_um.modified.insert(use);
          }
          // add line to the front of src_dep
          src_dep.insert(src_dep.begin(), parent->body[pos].get());
          break;
          // TODO(markus) do we need to add to src_um modified?
        }
      }
    }
  }
  // Extend used and modified to include dependencies from code in between
  // clearly only need to look at the later one
  for (auto& src_mod : src_um.modified) {
    if (dest_um.used.count(src_mod) > 0) return false;
  }
  for (auto& dest_mod : dest_um.modified) {
    if (src_um.used.count(dest_mod) > 0) return false;
  }
  if (DEBUG) {
    for (auto& x : src_um.modified) {
      printf("src mod %s\n", x.ToString().c_str());
    }
    for (auto& x : src_um.used) {
      printf("src use %s\n", x.ToString().c_str());
    }
    for (auto& x : dest_um.modified) {
      printf("dest mod %s\n", x.ToString().c_str());
    }
    for (auto& x : dest_um.used) {
      printf("dest ise %s\n", x.ToString().c_str());
    }
  }
  // If dest before src: find lines between that need to be moved as well
  // If dest after src: no such movement needed
  for (auto dependent_code : src_dep) {
    MoveCodeBefore(parent, dest, dependent_code);
  }
  for (auto& then_code : src->then->body) {
    dest->then->body.push_back(std::move(then_code));
  }
  src->then->body.clear();
  for (auto& other_code : src->otherwise->body) {
    dest->otherwise->body.push_back(std::move(other_code));
  }
  src->otherwise->body.clear();
  for (size_t pos = 0; pos < parent->body.size(); pos++) {
    if (parent->body[pos].get() == src) pos_src = pos;
  }
  parent->body.erase(parent->body.begin() + pos_src);
  if_count += 1;
  // if_limit -= 1;
  return true;
}

bool IsLetStmt(IRStmt* stmt) { return stmt->isa<LetIRStmt>(); }

bool IsContinueStmt(IRStmt* stmt) { return stmt->isa<ContinueIRStmt>(); }

class RenameVisitor : public TraverseIRVisitor {
  const std::map<IRIdentifier, IRIdentifier>& rename;

 public:
  explicit RenameVisitor(const std::map<IRIdentifier, IRIdentifier>& r)
      : rename(r) {}

  void Visit(RefIRExpr* expr) override {
    if (rename.count(expr->ref) > 0) {
      expr->ref = rename.at(expr->ref);
    }
  }
};

// TODO(markus) LAZY SUPREME
bool EqualUnderSubstitution(
    IRExpr* a, IRExpr* b, const std::map<IRIdentifier, IRIdentifier>& a_to_b) {
  auto a_clone = a->Clone();
  RenameVisitor vis(a_to_b);
  a_clone->Accept(&vis);
  return a_clone->ToString() == b->ToString();
}

bool EqualUnderSubstitution(
    const std::unique_ptr<IRExpr>& a, const std::unique_ptr<IRExpr>& b,
    const std::map<IRIdentifier, IRIdentifier>& a_to_b) {
  return EqualUnderSubstitution(a.get(), b.get(), a_to_b);
}

// Candidates are:
// - [X] At the same level
// - [X] Of the same kind
// - [ ] Iterate over the same stuff, are condition over the same stuff
//   - Have the same code leading up to a ``continue'' if it exists
//   - i.e. find last continue within each for, try to find unification between
//   the two
// - [ ] Do not depend directly or indirectly on each other's results
//   - have a list of ``used'' variables for each one
//   - have a list of ``modified'' variables for each one
//   - merge can only take place if each one's used does not contain anyone's
//   modified
// - [ ] Also move any statements that are dependent

bool MergeFor(CompoundIRStmt* parent, ForIRStmt* dest, ForIRStmt* src) {
  // Check if initial, cond and next match under src.index=dest.index
  // replacement.
  // If so, check if they match (unify) up to their last continue statement.
  // Remember that assignment, going from src to dest because we integrate
  // src into dest.

  assert(dest != src);
  assert(dest != NULL);
  assert(src != NULL);

  std::map<IRIdentifier, IRIdentifier> rename_src_to_dest;
  rename_src_to_dest[src->index] = dest->index;

  if (!EqualUnderSubstitution(src->initial, dest->initial,
                              rename_src_to_dest)) {
    if (DEBUG) printf("initial unequal\n");
    return false;
  }
  if (!EqualUnderSubstitution(src->top_value, dest->top_value,
                              rename_src_to_dest)) {
    if (DEBUG) printf("top unequal\n");
    return false;
  }
  if (DEBUG)
    printf("Attempting fusion for src %s and dest %s.\n",
           src->index.ToString().c_str(), dest->index.ToString().c_str());

  // iterate through both in parallel to find the continue range
  int last_continue_pos = -1;
  size_t pos = 0;
  for (; pos < src->body->body.size() && pos < dest->body->body.size(); pos++) {
    bool src_cont = IsContinueStmt(src->body->body[pos].get());
    bool dest_cont = IsContinueStmt(dest->body->body[pos].get());
    if (src_cont && dest_cont) {
      last_continue_pos = pos;
    } else if (src_cont || dest_cont) {
      if (DEBUG) printf("continue mismatch\n");
      return false;
    }
  }
  // check the remainder for further continue statements
  for (size_t remainder = pos; remainder < src->body->body.size();
       remainder++) {
    if (IsContinueStmt(src->body->body[remainder].get())) {
      if (DEBUG) printf("later continue A\n");
      return false;
    }
  }
  for (size_t remainder = pos; remainder < dest->body->body.size();
       remainder++) {
    if (IsContinueStmt(dest->body->body[remainder].get())) {
      if (DEBUG) printf("later continue B\n");
      return false;
    }
  }

  for (int pos = 0; pos < last_continue_pos; pos++) {
    bool src_cont = IsContinueStmt(src->body->body[pos].get());
    bool dest_cont = IsContinueStmt(dest->body->body[pos].get());
    bool src_let = IsLetStmt(src->body->body[pos].get());
    bool dest_let = IsLetStmt(dest->body->body[pos].get());
    if (src_cont && dest_cont) {
      ContinueIRStmt* src_stmt = src->body->body[pos]->asa<ContinueIRStmt>();
      ContinueIRStmt* dest_stmt = dest->body->body[pos]->asa<ContinueIRStmt>();
      if (!EqualUnderSubstitution(src_stmt->cond, dest_stmt->cond,
                                  rename_src_to_dest)) {
        if (DEBUG) printf("before continue A\n");
        return false;
      }
    } else if (src_let && dest_let) {
      LetIRStmt* src_stmt = src->body->body[pos]->asa<LetIRStmt>();
      LetIRStmt* dest_stmt = dest->body->body[pos]->asa<LetIRStmt>();
      if (!EqualUnderSubstitution(src_stmt->value, dest_stmt->value,
                                  rename_src_to_dest)) {
        if (DEBUG) printf("before continue B\n");
        return false;
      }
      rename_src_to_dest[src_stmt->name] = dest_stmt->name;
    } else {
      if (DEBUG) printf("before continue C\n");
      return false;
    }
  }

  FindUsedModifiedVisitor src_um;
  src->Accept(&src_um);
  FindUsedModifiedVisitor dest_um;
  dest->Accept(&dest_um);
  int pos_src = -1, pos_dest = -1;
  for (size_t pos = 0; pos < parent->body.size(); pos++) {
    if (parent->body[pos].get() == dest) pos_dest = pos;
    if (parent->body[pos].get() == src) pos_src = pos;
  }
  assert(pos_src != -1);
  assert(pos_dest != -1);
  assert(pos_src != pos_dest);
  std::vector<IRStmt*> src_dep;
  if (pos_src < pos_dest) {
    // source is in front of destination.
    // i.e. the question is: "does any of the code in between depend on any of
    // the data computed in src?
    // accordingly, collect ``used'' from steps in between as ``dest'' used.
    // no dependent code to move, but need to augment dest's
    for (size_t pos = pos_src + 1; pos < pos_dest; pos++) {
      parent->body[pos]->Accept(&dest_um);
    }
  } else if (pos_src > pos_dest) {
    // source comes after the destination.
    // potentially (certainly) need to copy over some code.
    // compute set of these lines into src_dep.
    // proceed in *reverse*, add line if it declares/modifies a variable used
    // in src_um.
    for (size_t pos = pos_src - 1; pos > pos_dest; pos--) {
      FindUsedModifiedVisitor um;
      parent->body[pos]->Accept(&um);
      // if it uses values we compute in the loop
      for (auto& used : um.used) {
        // if (dest_um.modified.count(used) > 0) {printf("A\n"); return false;}
        // if (dest_um.used_comm.count(used) > 0) {printf("B\n"); return false;}
      }
      // if it's ``modified'' is used within our code
      for (auto& mod : um.modified) {
        if (src_um.used.count(mod) > 0 || src_um.used_comm.count(mod) > 0) {
          // add uses into src_um
          for (auto& use : um.used) {
            src_um.used.insert(use);
          }
          for (auto& use : um.used_comm) {
            src_um.used_comm.insert(use);
          }
          for (auto& mod : um.modified) {
            src_um.modified.insert(mod);
          }
          // add line to the front of src_dep
          src_dep.insert(src_dep.begin(), parent->body[pos].get());
          break;
          // TODO(markus) do we need to add to src_um modified?
        }
      }
    }
  }
  // Extend used and modified to include dependencies from code in between
  // clearly only need to look at the later one
  for (auto& src_mod : src_um.modified) {
    if (dest_um.used.count(src_mod) > 0) {
      if (DEBUG) printf("dependencies A: %s\n", src_mod.ToString().c_str());
      return false;
    }
  }
  for (auto& dest_mod : dest_um.modified) {
    if (src_um.used.count(dest_mod) > 0) {
      if (DEBUG) printf("dependencies B\n");
      return false;
    }
  }
  // If dest before src: find lines between that need to be moved as well
  // If dest after src: no such movement needed
  for (auto dependent_code : src_dep) {
    MoveCodeBefore(parent, dest, dependent_code);
  }
  RenameVisitor rename_visitor(rename_src_to_dest);
  for (size_t pos = last_continue_pos + 1; pos < src->body->body.size();
       pos++) {
    dest->body->body.push_back(std::move(src->body->body[pos]));
    dest->body->body.back()->Accept(&rename_visitor);
  }
  src->body->body.clear();
  for (size_t pos = 0; pos < parent->body.size(); pos++) {
    if (parent->body[pos].get() == src) pos_src = pos;
  }
  parent->body.erase(parent->body.begin() + pos_src);

  return true;
}

struct FuseSyntaxVisitor : public EmptyIRStmtVisitor {
  std::vector<std::vector<ForIRStmt*>> for_stmts;
  std::vector<std::vector<IfIRStmt*>> if_stmts;
  bool fuse_loops = false, fuse_conditionals = false;

 public:
  void Visit(CompoundIRStmt* stmt) override {
    // collect all for/if stmts herein
    if_stmts.push_back({});
    for_stmts.push_back({});
    for (auto& child : stmt->body) {
      child->Accept(this);
    }
    if (!fuse_conditionals) if_stmts.back().clear();
    auto& my_if_records = if_stmts.back();
    for (size_t i = 0; i < my_if_records.size(); i++) {
      for (size_t j = 0; j < my_if_records.size(); j++) {
        if (i == j) continue;
        bool merged = MergeIf(stmt, my_if_records[i], my_if_records[j]);
        if (DEBUG) printf("if merge %zu %zu %d\n", i, j, merged);
        if (merged) {
          my_if_records.erase(my_if_records.begin() + j);
          if (j < i) i -= 1;
          j -= 1;
        }
        if (i >= my_if_records.size()) break;
      }
    }
    if (!fuse_loops) for_stmts.back().clear();
    auto& my_for_records = for_stmts.back();
    // for (int i = 0; i < my_for_records.size() / 2; i++) {
    //  std::swap(my_for_records[i], my_for_records[my_for_records.size() - 1 -
    //  i]);
    //}
    // for (size_t i = 0; i < my_for_records.size(); i++) {
    //  for (size_t j = 0; j < my_for_records.size(); j++) {
    for (int i = my_for_records.size() - 1; i >= 0; i--) {
      for (int j = my_for_records.size() - 1; j >= 0; j--) {
        if (i == j) continue;
        if (DEBUG)
          printf("try for merge dest %s src %s:\n",
                 my_for_records[i]->index.ToString().c_str(),
                 my_for_records[j]->index.ToString().c_str());
        bool merged = MergeFor(stmt, my_for_records[i], my_for_records[j]);
        if (merged && DEBUG) {
          printf("  SUCCESS\n");
        }
        if (merged) {
          // This incremental loop fusion seems to work better
          // if_stmts.pop_back();
          // for_stmts.pop_back();
          // return;
          my_for_records.erase(my_for_records.begin() + j);
          if (j < i) i -= 1;
          j -= 1;
        }
        // if (i >= my_for_records.size()) break;
      }
    }
    if_stmts.pop_back();
    for_stmts.pop_back();
  }
  void Visit(IfIRStmt* stmt) override {
    if_stmts.back().push_back(stmt);
    stmt->then->Accept(this);
    stmt->otherwise->Accept(this);
  }
  void Visit(ForIRStmt* stmt) override {
    for_stmts.back().push_back(stmt);
    stmt->body->Accept(this);
  }
};

}  // namespace

void FuseLoops(CompoundIRStmt* program) {
  FuseSyntaxVisitor vis;
  vis.fuse_loops = true;
  program->Accept(&vis);
}

void FuseConditionals(CompoundIRStmt* program) {
  FuseSyntaxVisitor vis;
  vis.fuse_conditionals = true;
  if (DEBUG) printf("Before: %d\n", if_count);
  program->Accept(&vis);
  if (DEBUG) printf("After: %d\n", if_count);
}

}  // namespace opt
}  // namespace potc
