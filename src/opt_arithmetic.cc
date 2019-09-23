#include "opt.h"

#include <map>
#include <sstream>
#include <vector>

#include "ir.h"
#include "ir_build.h"
#include "ir_visitor_empty.h"

namespace potc {

namespace opt {

namespace {

class ImproveArithmeticVisitor : public TraverseIRVisitor {
  std::vector<std::map<IRIdentifier, IRExpr*>> available;
  std::vector<std::map<std::string, int>> count_terms;
  std::vector<std::string> operand;

  std::unique_ptr<IRExpr> result;
  void AcceptExpr(std::unique_ptr<IRExpr>* expr) {
    result.reset();
    (*expr)->Accept(this);
    if (result) (*expr).reset(result.release());
  }
  IRExpr* UnRef(IRExpr* expr) {
    while (expr->isa<RefIRExpr>()) {
      RefIRExpr* ref = static_cast<RefIRExpr*>(expr);
      for (int i = available.size() - 1; i >= 0; i--) {
        if (available[i].count(ref->ref) > 0) {
          expr = available[i][ref->ref];
          break;
        }
      }
      if (ref == expr) break;
    }
    return expr;
  }
  IRExpr* UnRef(const std::unique_ptr<IRExpr>& expr) {
    return UnRef(expr.get());
  }

  void AddFactors(
      std::map<std::string, std::unique_ptr<IRExpr>>* unique_factors,
      std::map<std::string, int>* factor_counts, IRExpr* root, int increment) {
    std::vector<IRExpr*> todo;
    todo.push_back(root);
    while (!todo.empty()) {
      IRExpr* expr = todo.back();
      todo.pop_back();
      BinIRExpr* e = expr->asa<BinIRExpr>();
      if (e && e->op == "*") {
        todo.push_back(e->left.get());
        todo.push_back(e->right.get());
        continue;
      }
      std::string str = expr->ToString();
      if (unique_factors->count(str) == 0) {
        (*unique_factors)[str] = expr->Clone();
      }
      (*factor_counts)[str] += increment;
    }
  }
  std::unique_ptr<IRExpr> BuildTree(
      const std::map<std::string, std::unique_ptr<IRExpr>>& unique_factors,
      const std::map<std::string, int>& factor_counts, int increment) {
    using namespace potc::ir::build;

    std::vector<std::unique_ptr<IRExpr>> result;
    for (auto& elem : factor_counts) {
      int value = elem.second * increment;
      for (int i = 0; i < value; i++) {
        result.push_back(unique_factors.at(elem.first)->Clone());
      }
    }
    if (result.empty()) return Lit("1");
    while (result.size() > 1) {
      std::vector<std::unique_ptr<IRExpr>> next_result;
      for (int i = 0; i < result.size(); i += 2) {
        if (i + 1 < result.size()) {
          next_result.push_back(
              BinOp("*", std::move(result[i]), std::move(result[i + 1])));
        } else {
          next_result.push_back(std::move(result[i]));
        }
      }
      result = std::move(next_result);
    }
    assert(result.size() == 1);
    return std::move(result[0]);
  }

 public:
  ImproveArithmeticVisitor() {}
  void Visit(CompoundIRStmt* stmt) override {
    available.push_back({});
    for (auto& child : stmt->body) child->Accept(this);
    available.pop_back();
  }
  void Visit(LetIRStmt* stmt) override {
    AcceptExpr(&stmt->value);
    available.back()[stmt->name] = stmt->value.get();
  }
  void Visit(BinIRExpr* expr) override {
    using namespace potc::ir::build;

    AcceptExpr(&expr->left);
    AcceptExpr(&expr->right);
    // Rewrite arithmetic here
    if (expr->op == "+" && UnRef(expr->left)->ToString() == "0") {
      result = std::move(expr->right);
      return;
    }
    if (expr->op == "+" && UnRef(expr->right)->ToString() == "0") {
      result = std::move(expr->left);
      return;
    }
    if (expr->op == "-" && UnRef(expr->right)->ToString() == "0") {
      result = std::move(expr->left);
      return;
    }
    // if (expr->op == "-" && UnRef(expr->left)->ToString() == "0") {
    //  result = Neg(expr->right);
    //  return;
    //}
    if (expr->op == "*" && UnRef(expr->left)->ToString() == "1") {
      result = std::move(expr->right);
      return;
    }
    if (expr->op == "*" && UnRef(expr->right)->ToString() == "1") {
      result = std::move(expr->left);
      return;
    }
    if (expr->op == "*" && UnRef(expr->right)->ToString() == "0") {
      result = Lit("0");
      return;
    }
    if (expr->op == "/" && UnRef(expr->right)->ToString() == "1") {
      result = std::move(expr->left);
      return;
    }
    auto ref_l = UnRef(expr->left)->asa<FunCallIRExpr>();
    auto ref_r = UnRef(expr->right)->asa<FunCallIRExpr>();
    if (expr->op == "*" && ref_l && ref_r && ref_l->name == "sqrt" &&
        ref_r->name == "sqrt" &&
        ref_l->args[0]->ToString() == ref_r->args[0]->ToString()) {
      result = ref_l->args[0]->Clone();
      return;
    }
    if (UnRef(expr->left.get())->isa<LitIRExpr>() &&
        UnRef(expr->right.get())->isa<LitIRExpr>()) {
      LitIRExpr* l = UnRef(expr->left.get())->asa<LitIRExpr>();
      LitIRExpr* r = UnRef(expr->right.get())->asa<LitIRExpr>();
      int value_l, value_r;
      char rest;
      std::stringstream ss_l(l->value);
      ss_l >> value_l;
      if (ss_l.fail() || ss_l.get(rest)) return;
      std::stringstream ss_r(r->value);
      ss_r >> value_r;
      if (ss_r.fail() || ss_r.get(rest)) return;
      // TODO(markus) Float support & overflow checks
      int rewrite_value;
      if (expr->op == "+") {
        rewrite_value = value_l + value_r;
      }
      if (expr->op == "-") {
        rewrite_value = value_l - value_r;
      }
      if (expr->op == "*") {
        rewrite_value = value_l * value_r;
      }
      if (expr->op == "/") {
        if (value_l % value_r != 0) return;
        rewrite_value = value_l / value_r;
      }
      std::stringstream ss;
      ss << rewrite_value;
      result = Lit(ss.str());
      return;
    }
    {
      std::unique_ptr<IRExpr>* left = &expr->left;
      UnIRExpr* l = left->get()->asa<UnIRExpr>();
      bool left_neg = false;
      if (l && l->op == "-") {
        left_neg = true;
        left = &l->inner;
      }
      std::unique_ptr<IRExpr>* right = &expr->right;
      UnIRExpr* r = right->get()->asa<UnIRExpr>();
      bool right_neg = false;
      if (r && r->op == "-") {
        right_neg = true;
        right = &r->inner;
      }
      if (expr->op == "*" || expr->op == "/") {
        if (left_neg && right_neg) {
          result = BinOp(expr->op, *left, *right);
          return;
        } else if (left_neg || right_neg) {
          result = Neg(BinOp(expr->op, *left, *right));
          return;
        }
      }
      if (expr->op == "+") {
        if (left_neg && right_neg) {
          result = Neg(Add(*left, *right));
          return;
        } else if (left_neg) {
          result = Sub(*right, *left);
          return;
        } else if (right_neg) {
          result = Sub(*left, *right);
          return;
        }
      }
      if (expr->op == "-") {
        if (left_neg && right_neg) {
          result = Sub(*right, *left);
          return;
        } else if (left_neg) {
          result = Neg(Add(*left, *right));
          return;
        } else if (right_neg) {
          result = Add(*left, *right);
          return;
        }
      }
    }
    {
      BinIRExpr* l = expr->left->asa<BinIRExpr>();
      BinIRExpr* r = expr->right->asa<BinIRExpr>();
      if (expr->op == "*") {
        // (a * (b / c)) -> (a * b ) / c
        if (l && l->op == "/") {
          result =
              BinOp("/", BinOp("*", std::move(expr->right), std::move(l->left)),
                    std::move(l->right));
          return;
        }
        // ((a / b) * c) -> (a * c) / b
        if (r && r->op == "/") {
          result =
              BinOp("/", BinOp("*", std::move(expr->left), std::move(r->left)),
                    std::move(r->right));
          return;
        }

        std::map<std::string, std::unique_ptr<IRExpr>> unique_factors;
        std::map<std::string, int> factor_counts;
        AddFactors(&unique_factors, &factor_counts, expr->left.get(), 1);
        AddFactors(&unique_factors, &factor_counts, expr->right.get(), 1);
        result = BuildTree(unique_factors, factor_counts, 1);
        return;
      }
      if (expr->op == "/") {
        // ((a / b) / c -> (a * c) / b
        if (l && l->op == "/") {
          result =
              BinOp("/", std::move(l->left),
                    BinOp("*", std::move(expr->right), std::move(l->right)));
          return;
        }
        // (a / (b / c)) -> (a * c) / b
        if (r && r->op == "/") {
          result =
              BinOp("/", BinOp("*", std::move(expr->left), std::move(r->right)),
                    std::move(r->left));
          return;
        }

        std::map<std::string, std::unique_ptr<IRExpr>> unique_factors;
        std::map<std::string, int> factor_counts;
        AddFactors(&unique_factors, &factor_counts, expr->left.get(), 1);
        AddFactors(&unique_factors, &factor_counts, expr->right.get(), -1);
        expr->left = BuildTree(unique_factors, factor_counts, 1);
        expr->right = BuildTree(unique_factors, factor_counts, -1);
        return;
      }
    }
    // How to detect if two identical entries exist in the same binop tree?
    // stack of operands: If same don't push, don't pop, parallel frame for
    // values as strings
    // Rewrite("x/x", "1"); // Needs to UnRef both completely
    // Consider another level of depth
    // TODO(markus)
  }
  void Visit(UnIRExpr* expr) override {
    AcceptExpr(&expr->inner);
    if (expr->op == "-") {
      UnIRExpr* inner = expr->inner.get()->asa<UnIRExpr>();
      if (inner && inner->op == "-") {
        result = std::move(inner->inner);
        return;
      }
    }
  }
  void Visit(FunCallIRExpr* expr) override {
    for (auto& arg : expr->args) {
      AcceptExpr(&arg);
    }
    if (expr->name == "pow") {
      using namespace potc::ir::build;

      // TODO(markus) If Sub(x, Lit("1")), then insert the if formulation
      LitIRExpr* lit = UnRef(expr->args[1])->asa<LitIRExpr>();
      if (lit && lit->value == "2") {
        FunCallIRExpr* call = UnRef(expr->args[0])->asa<FunCallIRExpr>();
        if (call && call->name == "sqrt") {
          result = call->args[0]->Clone();
          return;
        }
        result = Mul(expr->args[0], expr->args[0]);
        return;
      } else if (lit) {
        int value;
        char rest;
        std::stringstream ss(lit->value);
        ss >> value;
        if (ss.fail() || ss.get(rest)) return;
        result = Lit("1");
        for (int i = 0; i < value; i++) {
          result = Mul(std::move(result), expr->args[0]);
        }
      }
      // If IsInteger and positive
      // If IsInteger and negative
      // If IsInteger and multiple of half

      BinIRExpr* bin = UnRef(expr->args[1])->asa<BinIRExpr>();
      LitIRExpr* lit_sub = bin ? UnRef(bin->right)->asa<LitIRExpr>() : nullptr;
      if (lit_sub && lit_sub->value == "1" && bin->op == "-") {
        // InsertBefore(Choose(BinOp("==", expr->args[0], "0.0"), Lit("1"),
        // Div(Pow(expr->args[0], bin->args[0]), expr->args[0])));
        // TODO(markus) insert the if and stuff here
      }
      LitIRExpr* lit_left = bin ? UnRef(bin->left)->asa<LitIRExpr>() : nullptr;
      // pow(a, 0-x) -> 1 / pow(a, x) what if a == 0?
      // TODO(markus): Is this actually safe? It also increases the flop
      // intensity
      // works if pow(a, x) is also in scope
      // if we had perfect CSE, it would be easier to detect, i.e. only apply if
      // pow(ref, -ref).
      // if (lit_left && bin->op == "-" && lit_left->value == "0") {
      //   auto temp = expr->Clone();
      //   temp->asa<FunCallIRExpr>()->args[1] = bin->left->Clone();
      //   result = Div(Lit("1"), temp);
      //   return;
      // }
      // UnIRExpr* un = UnRef(expr->args[1])->asa<UnIRExpr>();
      // if (un && un->op == "-") {
      //   auto temp = expr->Clone();
      //   temp->asa<FunCallIRExpr>()->args[1] = un->inner->Clone();
      //   result = Div(Lit("1"), temp);
      //   return;
      // }
    }
    if (expr->name == "cos") {
      FunCallIRExpr* fun = UnRef(expr->args[0])->asa<FunCallIRExpr>();
      if (fun && fun->name == "acos") {
        result = fun->args[0]->Clone();
        return;
      }
    }
    if (expr->name == "sin") {
      FunCallIRExpr* fun = UnRef(expr->args[0])->asa<FunCallIRExpr>();
      if (fun && fun->name == "acos") {
        using namespace potc::ir::build;
        result = Sqrt(
            Sub(Lit("1"), Mul(fun->args[0]->Clone(), fun->args[0]->Clone())));
        return;
      }
    }
    // Rewrite cos and sin and pow here
    // Rewrite("cos(acos(x))", "x");
    // Rewrite("sin(acos(x))", "sqrt(1-x*x)");
    // Rewrite("pow(x, 2)", "x*x");
  }
};

}  // namespace

void ImproveArithmetic(CompoundIRStmt* program) {
  ImproveArithmeticVisitor vis;
  vis.Visit(program);
}

}  // namespace opt
}  // namespace potc
