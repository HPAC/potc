#include "gtest/gtest.h"
#include "ir_build.h"
#include "opt.h"

namespace {

TEST(MainTest, SetupWorks) { EXPECT_EQ(0, 0); }

TEST(MainTest, SingleUseInline) {
  using namespace potc::ir::build;
  using namespace potc::ir::build::stmt;
  CompoundIRStmt stmt;
  IRIdentifier a(0, "");
  IRIdentifier b(1, "");
  stmt.body.push_back(Let(a, Lit("1")));
  stmt.body.push_back(Let(b, Ref(a)));
  stmt.body.push_back(std::make_unique<AccEnergyIRStmt>(Add(Ref(b), Ref(b))));
  potc::opt::EliminateUnusedIntegrate(&stmt);
  potc::opt::EliminateUnusedIntegrate(&stmt);
  potc::opt::EliminateUnusedIntegrate(&stmt);
  EXPECT_EQ(stmt.body.size(), 2);
}
}  // namespace
