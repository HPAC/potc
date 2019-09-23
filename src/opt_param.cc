#include "opt.h"

#include <map>
#include <vector>

#include "ir.h"
#include "ir_visitor_empty.h"

// how to pass back the outlined parameters?

namespace potc {

namespace opt {

namespace {

struct UnravelExprVisitor : public TraverseIRVisitor {
  std::map<IRIdentifier, IRExpr*>* id_names;
  std::unique_ptr<IRExpr> result;
  std::vector<std::unique_ptr<IRExpr>> ty_expr;
  std::map<std::string, IRIdentifier> ty_map;
  int next_ty = 0;
  void AcceptExpr(std::unique_ptr<IRExpr>* expr) override {
    (*expr)->Accept(this);
    while (result) {
      (*expr) = std::move(result);
      (*expr)->Accept(this);
    }
  }
  void Visit(RefIRExpr* expr) override {
    if ((*id_names).count(expr->ref) == 0) {
      printf("Unknown ref: %s.\n", expr->ref.ToString().c_str());
      return;
    }
    result = (*id_names)[expr->ref]->Clone();
  }
  void Visit(LookupIRExpr* expr) override {
    if (expr->lookup_kind != LookupIRExpr::kParam &&
        expr->lookup_kind != LookupIRExpr::kMasterCutoff) {
      assert(0);  // impossible at this stage
      return;
    }
    for (auto& arg : expr->args) {
      auto str = arg->ToString();
      if (ty_map.count(str) == 0) {
        IRIdentifier id(next_ty++, "ty");
        ty_map[str] = id;
        ty_expr.push_back(arg->Clone());
      }
      arg = std::make_unique<RefIRExpr>(ty_map[str]);
    }
  }
};

// candidates exist that are ``maximal'' in size, and
// i.e. need x s.t. ! is_param[parent[x]] but is_param[x]

struct OutlineParametersVisitor : public TraverseIRVisitor {
  std::vector<int> is_param;
  std::map<IRExpr*, int> is_expr_param;
  std::map<IRExpr*, IRExpr*> expr_parent;
  std::map<IRExpr*, std::unique_ptr<IRExpr>*> expr_location;
  std::map<IRIdentifier, int> id_param;
  std::vector<IRExpr*> parents;
  std::map<IRIdentifier, IRExpr*> id_names;

  std::vector<std::pair<int, std::unique_ptr<IRExpr>>> outlined;

  const int PARAM_OUTLINE_THRESHOLD = 7;  // free parameter

  void FindCandidates() {
    std::map<std::string, int> candidates;
    for (auto& entry : is_expr_param) {
      if (entry.second <= PARAM_OUTLINE_THRESHOLD) continue;
      // We handle constant lets at the point where they are referenced
      if (!expr_parent[entry.first]) continue;
      if (is_expr_param[expr_parent[entry.first]]) continue;
      auto unravel_expr = entry.first->Clone();
      UnravelExprVisitor unravel_vis;
      unravel_vis.id_names = &id_names;
      unravel_vis.AcceptExpr(&unravel_expr);
      if (unravel_vis.next_ty == 0) continue;
      auto str = unravel_expr->ToString();
      if (candidates.count(str) == 0) {
        int id = outlined.size();
        candidates[str] = id;
        outlined.push_back({unravel_vis.next_ty, std::move(unravel_expr)});
        printf("Candidate parameter: %d %d %s\n", unravel_vis.next_ty,
               entry.second, str.c_str());
      }
      int id = candidates[str];
      *expr_location[entry.first] = std::make_unique<LookupIRExpr>(
          LookupIRExpr::kOutlinedParam, std::to_string(id),
          std::move(unravel_vis.ty_expr));
      // if (entry.second < 2) continue;
      // printf("Candidate parameter %d: %s\n", entry.second, str.c_str());
    }
  }
  void Combine(int n) {
    int val = 1;
    for (int i = 0; i < n; i++) {
      int sub_val = is_param.back();
      is_param.pop_back();
      if (sub_val == 0 || val == 0) {
        val = 0;
      } else {
        val += sub_val;
      }
    }
    is_param.push_back(val);
  }
  void AcceptExpr(std::unique_ptr<IRExpr>* expr) {
    if (!parents.empty()) expr_parent[(*expr).get()] = parents.back();
    parents.push_back((*expr).get());
    (*expr)->Accept(this);
    is_expr_param[(*expr).get()] = is_param.back();
    expr_location[(*expr).get()] = expr;
    parents.pop_back();
  }
  void Visit(LetIRStmt* stmt) override {
    TraverseIRVisitor::Visit(stmt);
    id_param[stmt->name] = is_param.back();
    id_names[stmt->name] = stmt->value.get();
    is_param.pop_back();
  }
  void Visit(LetComplexFunCallIRStmt* stmt) override {
    TraverseIRVisitor::Visit(stmt);
    Combine(stmt->args.size());
    // id_param[stmt->name] = is_param.back();
    // for (auto& n : stmt->other_names) {
    //  id_param[n] = is_param.back();
    //}
    is_param.pop_back();
  }
  void Visit(UnIRExpr* expr) override {
    TraverseIRVisitor::Visit(expr);
    Combine(1);
  }
  void Visit(BinIRExpr* expr) override {
    TraverseIRVisitor::Visit(expr);
    Combine(2);
  }
  void Visit(LitIRExpr* expr) override { is_param.push_back(1); }
  void Visit(RefIRExpr* expr) override {
    is_param.push_back(id_param[expr->ref]);
  }
  void Visit(FunCallIRExpr* expr) override {
    TraverseIRVisitor::Visit(expr);
    Combine(expr->args.size());
  }
  void Visit(InvalidIRExpr* expr) override { is_param.push_back(0); }
  void Visit(LookupIRExpr* expr) override {
    if (expr->lookup_kind != LookupIRExpr::kParam &&
        expr->lookup_kind != LookupIRExpr::kMasterCutoff) {
      is_param.push_back(0);
      return;
    }
    TraverseIRVisitor::Visit(expr);
    Combine(expr->args.size());
    is_param.back() = 1;
  }
};

}  // namespace

void OutlineParameters(PassContext* ctx) {
  OutlineParametersVisitor vis;
  ctx->program->Accept(&vis);
  vis.FindCandidates();
  assert(ctx->outlined->empty());
  *ctx->outlined = std::move(vis.outlined);
}

}  // namespace opt

}  // namespace potc
