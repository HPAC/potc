#include <vector>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "gen_internal.h"

using namespace potc::gen;

struct FormatTest : public ::testing::Test {
  virtual ~FormatTest() {}
  virtual void TearDown() {}

  std::map<std::string, std::string> args;
  std::vector<std::map<std::string, std::string>> loop_args;

  virtual void SetUp() {
    args["hello"] = "world";
    args["nope"] = "";
    for (int i = 0; i < 3; i++) {
      loop_args.push_back({});
      loop_args[i]["i"] = std::to_string(i);
      loop_args[i]["even"] = i % 2 == 0 ? "yes" : "";
    }
  }
};

TEST_F(FormatTest, SetupWorks) { EXPECT_EQ(0, 0); }

TEST_F(FormatTest, ReplaceWorks) {
  EXPECT_EQ(FormatTemplate("hello @hello@!", args, loop_args), "hello world!");
}

TEST_F(FormatTest, IfWorks) {
  EXPECT_EQ(FormatTemplate("@ifhello@hello @hello@@/@!", args, loop_args),
            "hello world!");
  EXPECT_EQ(FormatTemplate("@ifnope@hello @hello@@/@!", args, loop_args), "!");
}

TEST_F(FormatTest, LoopWorks) {
  EXPECT_EQ(FormatTemplate("@loop@@i@ @/@!", args, loop_args), "0 1 2 !");
}

TEST_F(FormatTest, IfEqWorks) {
  EXPECT_EQ(FormatTemplate("@ifhello=hello@hello @hello@@/@!", args, loop_args),
            "hello world!");
  EXPECT_EQ(FormatTemplate("@ifhello!nope@hello @hello@@/@!", args, loop_args),
            "hello world!");
  EXPECT_EQ(FormatTemplate("@ifnope=nope@hello @hello@@/@!", args, loop_args),
            "hello world!");
  EXPECT_EQ(FormatTemplate("@ifhello!hello@hello @hello@@/@!", args, loop_args),
            "!");
  EXPECT_EQ(FormatTemplate("@ifnope!nope@hello @hello@@/@!", args, loop_args),
            "!");
}

TEST_F(FormatTest, NamedLoopWorks) {
  EXPECT_EQ(FormatTemplate("@loopi@@ii@ @/@!", args, loop_args), "0 1 2 !");
}

TEST_F(FormatTest, NestedIfWorks) {
  EXPECT_EQ(FormatTemplate("@ifhello@@ifhello@!@/@!@/@!", args, loop_args),
            "!!!");
  EXPECT_EQ(FormatTemplate("@ifhello@@ifnope@!@/@!@/@!", args, loop_args),
            "!!");
  EXPECT_EQ(FormatTemplate("@ifnope@@ifhello@!@/@!@/@!", args, loop_args), "!");
  EXPECT_EQ(FormatTemplate("@ifnope@@ifnope@!@/@!@/@!", args, loop_args), "!");
}

TEST_F(FormatTest, NestedLoopWorks) {
  EXPECT_EQ(FormatTemplate("@loopi@@loopj@@ii@@ji@ @/@@/@!", args, loop_args),
            "00 01 02 10 11 12 20 21 22 !");
}

TEST_F(FormatTest, NestedLoopIfWorks) {
  EXPECT_EQ(FormatTemplate("@loopi@@ifieven@@ii@ @/@@/@!", args, loop_args),
            "0 2 !");
}

TEST_F(FormatTest, NestedLoopAndIfNeqWorks) {
  EXPECT_EQ(FormatTemplate("@loopi@@loopj@@ifii!ji@@ii@@ji@ @/@@/@@/@!", args,
                           loop_args),
            "01 02 10 12 20 21 !");
}
