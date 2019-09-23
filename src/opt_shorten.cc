// Copyright 2018 Markus Hoehnerbach

#include "opt.h"

#include <map>

#include "ir.h"
#include "ir_build.h"
#include "ir_visitor_empty.h"

namespace potc {

namespace opt {

namespace {

// collect all uses of the neighbor list
// find reuse
// if reuse, do the shortening
// need to assume a parameter max cutoff i, j
// -> I kind of need this anyways for the init_one routine...
// introduce a shortening loop at the correct scope (as far out as possible)
// rewrite the other usages
// the goal here is not introduce new global neighlists, but always in the
// tightest possible instances.
// We do not consider as ``worthy'' rewriting goals anything that is shielded by
// a conditional

// what counts as a neighbor list usage?
// We can just count occurences of loops that go up to numneigh
// Collect these based on ``scopes'', and distinguishing based on the indices it
// uses
// I.e. collect recursively
// Analyze in the scope where that index is introduced
// Count uses lexically

// This pass needs to add code, and in particular to introduce additional
// variables
// It might be time to build facilities to easily enable this

// How to handle half neighbor lists when shortening?
// In particular, can we retain the ``half'' thing?
// Need the structure & connection to the original list
// Maybe just build two lists? Twice memory, but so what?

// Also consider if we want to retain the Continue stmt, or somehow structure it
// more within the for stmt...

struct RewriteListUseVisitor : public TraverseIRVisitor {
  IRIdentifier rewrite_from;
  IRIdentifier new_list_name;
  std::unique_ptr<IRExpr> rewrite_to;
  RewriteListUseVisitor(IRIdentifier rf, IRIdentifier nln)
      : rewrite_from(rf), new_list_name(nln) {}
  void AcceptExpr(std::unique_ptr<IRExpr>* expr) {
    rewrite_to.reset();
    (*expr)->Accept(this);
    if (rewrite_to) *expr = std::move(rewrite_to);
  }
  void Visit(LookupIRExpr* expr) override {
    switch (expr->lookup_kind) {
      case LookupIRExpr::kNeighborNum:
      case LookupIRExpr::kNeighborHalfNum: {
        // How to distinguish the lenght? RefListExpr w/o args?
        // what is my invariant for lookupirexpr? totally constant?
        // no: a ref() to the other stuff should be enough
        // TODO(markus): Still unclear how to get half neighlist...
        assert(expr->args.size() == 1);
        auto orig_ref = expr->args[0]->asa<RefIRExpr>();
        if (orig_ref->ref != rewrite_from) return;
        if (expr->lookup_kind == LookupIRExpr::kNeighborNum)
          expr->lookup_kind = LookupIRExpr::kListNum;
        if (expr->lookup_kind == LookupIRExpr::kNeighborHalfNum)
          expr->lookup_kind = LookupIRExpr::kListHalfNum;
        using potc::ir::build::Ref;
        expr->args[0] = Ref(new_list_name);
        break;
      }
      case LookupIRExpr::kNeighborEntry: {
        assert(expr->args.size() == 2);
        auto orig_ref = expr->args[0]->asa<RefIRExpr>();
        if (orig_ref->ref != rewrite_from) return;
        expr->lookup_kind = LookupIRExpr::kListEntry;
        using potc::ir::build::Ref;
        expr->args[0] = Ref(new_list_name);
        break;
      }
    }
  }
};

// list half handling is missing and important
struct FindNeighUsesVisitor : public TraverseIRVisitor {
  std::map<IRIdentifier, int> uses;
  std::map<IRIdentifier, bool> need_half;
  std::map<IRIdentifier, IRIdentifier> list_access;
  int nlist_idx = 0;
  IRIdentifierContext* idc;
  void Visit(ForIRStmt* stmt) override {
    TraverseIRVisitor::Visit(stmt);
    int direct_use = uses[stmt->index];
    int indirect_use = 0;
    if (list_access.count(stmt->index) >
        0) {  // avoid uninitialized IRIdentifier in lookup
      indirect_use = uses[list_access[stmt->index]];
    }
    // printf("[DEBUG] %s %s %d %d\n", stmt->index.ToString().c_str(),
    // list_access[stmt->index].ToString().c_str(), direct_use, indirect_use);
    if (direct_use + indirect_use <= 1) return;
    printf("Should outline the neighlist construction stuff for %s\n",
           stmt->index.ToString().c_str());
    // insert the loop at the beginning, i.e. after the last ``continue'' stmt
    // find last continue stmt
    auto& body = stmt->body;
    int last_continue_pos = body->body.size() - 1;
    for (; last_continue_pos >= 0; last_continue_pos--) {
      if (body->body[last_continue_pos]->isa<ContinueIRStmt>()) break;
    }
    for (; last_continue_pos + 1 < body->body.size(); last_continue_pos++) {
      if (body->body[last_continue_pos + 1]->isa<ForIRStmt>()) break;
      if (body->body[last_continue_pos + 1]->isa<IfIRStmt>()) break;
    }
    // if indirect, make sure to base it off the list_access thing
    IRIdentifier base_id = direct_use ? stmt->index : list_access[stmt->index];
    // insert the decl for the list

    using namespace potc::ir::build;
    using namespace potc::ir::build::stmt;

    IRIdentifier id_list = idc->Next("snlist");
    IRIdentifier id_counter = idc->Next("scounter");

    int insert_pos = 1;
    body->body.insert(body->body.begin() + last_continue_pos + insert_pos++,
                      std::make_unique<DeclListIRStmt>(id_list));
    auto f = std::make_unique<ForIRStmt>(
        id_counter, Lit("0"), std::make_unique<LookupIRExpr>(
                                  LookupIRExpr::kNeighborNum, Ref(base_id)),
        std::make_unique<CompoundIRStmt>());
    // Add cutoff check in here
    // Use the MasterCutoff to do so
    // Or use the most restrictive check that exists?
    // Add the is_half check to the AddList statement
    auto id_atom = idc->Next("satom");
    f->body->body.push_back(Let(
        id_atom, std::make_unique<LookupIRExpr>(LookupIRExpr::kNeighborEntry,
                                                Ref(base_id), Ref(id_counter)),
        kIRDataTypeInt));
    auto dx = idc->Get("dx"), dy = idc->Get("dy"), dz = idc->Get("dz");
    f->body->body.push_back(Let(
        dx,
        Sub(std::make_unique<LookupIRExpr>(LookupIRExpr::kPosX, Ref(base_id)),
            std::make_unique<LookupIRExpr>(LookupIRExpr::kPosX,
                                           Ref(id_atom)))));
    f->body->body.push_back(Let(
        dy,
        Sub(std::make_unique<LookupIRExpr>(LookupIRExpr::kPosY, Ref(base_id)),
            std::make_unique<LookupIRExpr>(LookupIRExpr::kPosY,
                                           Ref(id_atom)))));
    f->body->body.push_back(Let(
        dz,
        Sub(std::make_unique<LookupIRExpr>(LookupIRExpr::kPosZ, Ref(base_id)),
            std::make_unique<LookupIRExpr>(LookupIRExpr::kPosZ,
                                           Ref(id_atom)))));
    f->body->body.push_back(
        Let(idc->Get("rsq"),
            Add(Mul(Ref(dx), Ref(dx)),
                Add(Mul(Ref(dy), Ref(dy)), Mul(Ref(dz), Ref(dz))))));
    f->body->body.push_back(
        Let(idc->Get("ti"),
            std::make_unique<LookupIRExpr>(LookupIRExpr::kType, Ref(base_id)),
            kIRDataTypeInt));
    f->body->body.push_back(
        Let(idc->Get("tj"),
            std::make_unique<LookupIRExpr>(LookupIRExpr::kType, Ref(id_atom)),
            kIRDataTypeInt));
    f->body->body.push_back(
        Let(idc->Get("cut"), std::make_unique<LookupIRExpr>(
                                 LookupIRExpr::kMasterCutoff,
                                 Ref(idc->Get("ti")), Ref(idc->Get("tj")))));
    f->body->body.push_back(std::make_unique<ContinueIRStmt>(
        BinOp(">", Ref(idc->Get("rsq")), Ref(idc->Get("cut")))));
    if (need_half[base_id]) {
      f->body->body.push_back(std::make_unique<AddListIRStmt>(
          id_list, Ref(id_atom),
          BinOp("<", Ref(id_counter),
                std::make_unique<LookupIRExpr>(LookupIRExpr::kNeighborHalfNum,
                                               Ref(base_id)),
                kIRDataTypeInt)));
    } else {
      f->body->body.push_back(
          std::make_unique<AddListIRStmt>(id_list, Ref(id_atom)));
    }
    // insert the loop
    body->body.insert(body->body.begin() + last_continue_pos + insert_pos++,
                      std::move(f));
    // rewrite everything below
    RewriteListUseVisitor vis(base_id, id_list);
    // 3 = next instr + decl + for
    for (int i = last_continue_pos + insert_pos; i < body->body.size(); i++) {
      body->body[i]->Accept(&vis);
    }
  }
  void Visit(IfIRStmt* stmt) override {
    // Do not consider loops under ifs
  }
  void Visit(LookupIRExpr* expr) override {
    if (expr->lookup_kind == LookupIRExpr::kNeighborNum ||
        expr->lookup_kind == LookupIRExpr::kNeighborHalfNum) {
      assert(expr->args.size() == 1);
      auto ref = expr->args[0]->asa<RefIRExpr>();
      assert(ref);
      uses[ref->ref] += 1;
      if (expr->lookup_kind == LookupIRExpr::kNeighborHalfNum)
        need_half[ref->ref] = true;
    }
  }
  void Visit(LetIRStmt* stmt) override {
    auto expr = stmt->value->asa<LookupIRExpr>();
    if (!expr) return;
    if (expr->lookup_kind != LookupIRExpr::kNeighborEntry) return;
    assert(expr->args.size() == 2);
    auto ref = expr->args[1]->asa<RefIRExpr>();
    assert(ref);
    list_access[ref->ref] = stmt->name;
  }
};

}  // namespace

void ShortenNeighLists(CompoundIRStmt* program, IRIdentifierContext* ctx) {
  FindNeighUsesVisitor vis;
  vis.idc = ctx;
  program->Accept(&vis);
}

}  // namespace opt

}  // namespace potc
