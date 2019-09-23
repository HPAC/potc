#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "gen_spline.cc"  // NOLINT

using namespace potc::gen;

TEST(SplineTest, SetupWorks) { EXPECT_EQ(0, 0); }

TEST(SplineTest, PowInt) {
  EXPECT_EQ(potc::gen::PowInt(2, 0), 1);
  EXPECT_EQ(potc::gen::PowInt(2, 1), 2);
  EXPECT_EQ(potc::gen::PowInt(2, 2), 4);
}

TEST(SplineTest, SplineIterator) {
  EXPECT_EQ(SplineIterator(1, 1).size(), 2);
  EXPECT_EQ(SplineIterator(1, 2).size(), 3);
  EXPECT_EQ(SplineIterator(2, 1).size(), 4);
  EXPECT_EQ(SplineIterator(2, 2).size(), 9);
}

TEST(SplineTest, SplinePos) {
  EXPECT_EQ(SplinePos(1, 1, {0}), 0);
  EXPECT_EQ(SplinePos(1, 1, {1}), 1);
  EXPECT_EQ(SplinePos(2, 1, {0, 0}), 0);
  EXPECT_EQ(SplinePos(2, 1, {1, 0}), 1);
  EXPECT_EQ(SplinePos(2, 1, {0, 1}), 2);
  EXPECT_EQ(SplinePos(2, 1, {1, 1}), 3);
}

TEST(SplineTest, Hypercube) {
  EXPECT_EQ(Hypercube(1).size(), 2);
  EXPECT_EQ(Hypercube(2).size(), 4);
  EXPECT_EQ(Hypercube(3).size(), 8);
  EXPECT_EQ(Hypercube(4).size(), 16);
}

TEST(SplineTest, DeriveAt) {
  EXPECT_EQ(DerivativeAt(1, 3, {1, 1, 1, 1}, {}, {0}),
            (std::vector<int>{1, 0, 0, 0}));
  EXPECT_EQ(DerivativeAt(1, 3, {1, 1, 1, 1}, {0}, {0}),
            (std::vector<int>{0, 1, 0, 0}));
  EXPECT_EQ(DerivativeAt(1, 3, {1, 1, 1, 1}, {}, {1}),
            (std::vector<int>{1, 1, 1, 1}));
  EXPECT_EQ(DerivativeAt(1, 3, {1, 1, 1, 1}, {0}, {1}),
            (std::vector<int>{0, 1, 2, 3}));
}

TEST(SplineTest, Choices) {
  EXPECT_THAT(Choices(2, 0),
              ::testing::ContainerEq(std::vector<std::vector<int>>{{}}));
  EXPECT_THAT(Choices(2, 1),
              ::testing::ContainerEq(std::vector<std::vector<int>>{{0}, {1}}));
  EXPECT_THAT(Choices(2, 2),
              ::testing::ContainerEq(std::vector<std::vector<int>>{{1, 0}}));
  EXPECT_THAT(Choices(3, 2),
              ::testing::ContainerEq(
                  std::vector<std::vector<int>>{{1, 0}, {2, 0}, {2, 1}}));
}

TEST(SplineTest, Derivatives) {
  EXPECT_THAT(Derivatives(1, 3),
              ::testing::ContainerEq(std::vector<std::vector<int>>{{}, {0}}));
  EXPECT_THAT(
      Derivatives(1, 5),
      ::testing::ContainerEq(std::vector<std::vector<int>>{{}, {0}, {0, 0}}));
  EXPECT_THAT(Derivatives(2, 3),
              ::testing::ContainerEq(
                  std::vector<std::vector<int>>{{}, {0}, {1}, {0, 1}}));
  EXPECT_THAT(
      Derivatives(2, 5),
      ::testing::ContainerEq(std::vector<std::vector<int>>{{},
                                                           {0},
                                                           {1},
                                                           {0, 0},
                                                           {0, 1},
                                                           {1, 1},
                                                           {0, 0, 1},
                                                           {0, 1, 1},
                                                           {0, 0, 1, 1}}));
  EXPECT_THAT(Derivatives(3, 3),
              ::testing::ContainerEq(std::vector<std::vector<int>>{
                  {}, {0}, {1}, {2}, {0, 1}, {0, 2}, {1, 2}, {0, 1, 2}}));
}

TEST(SplineTest, RatGCD) {
  EXPECT_EQ(RatGCD(2, 6), 2);
  EXPECT_EQ(RatGCD(10, 6), 2);
  EXPECT_EQ(RatGCD(20, 24), 4);
}

TEST(SplineTest, RatNorm) {
  EXPECT_EQ(RatNorm({0, 2}).num, 0);
  EXPECT_EQ(RatNorm({0, 2}).den, 1);
  EXPECT_EQ(RatNorm({1, -2}).num, -1);
  EXPECT_EQ(RatNorm({1, -2}).den, 2);
  EXPECT_EQ(RatNorm({2, -4}).num, -1);
  EXPECT_EQ(RatNorm({2, -4}).den, 2);
}

TEST(SplineTest, RatAddMul) {
  EXPECT_EQ(RatAddMul({1, 2}, {-2, 3}, {1, 2}).num, 1);
  EXPECT_EQ(RatAddMul({1, 2}, {-2, 3}, {1, 2}).den, 6);
}

TEST(SplineTest, GaussJ) {
  EXPECT_EQ(GaussJ({1, 0, 0, 1}, 2)[0], std::vector<int>({1, 0, 0, 1}));
  EXPECT_EQ(GaussJ({1, 0, 0, 1}, 2)[1], std::vector<int>({1, 1, 1, 1}));
  EXPECT_EQ(GaussJ({1, 1, 0, 1}, 2)[0], std::vector<int>({1, -1, 0, 1}));
  EXPECT_EQ(GaussJ({1, 1, 0, 1}, 2)[1], std::vector<int>({1, 1, 1, 1}));
}
