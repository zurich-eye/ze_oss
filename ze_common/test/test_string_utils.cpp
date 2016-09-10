// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#include <string>
#include <vector>

#include <ze/common/string_utils.hpp>
#include <ze/common/test_entrypoint.hpp>


TEST(StringUtilsTest, testTrim)
{
  std::string a = " abc ";
  a = ze::trimString(a);
  EXPECT_EQ(a, "abc");
}

TEST(StringUtilsTest, testSplit)
{
  std::string a = "one,two,three";
  std::vector<std::string> vec = ze::splitString(a, ',');
  EXPECT_EQ(vec.size(), 3u);
  EXPECT_EQ(vec[0], "one");
  EXPECT_EQ(vec[1], "two");
  EXPECT_EQ(vec[2], "three");
}

TEST(StringUtilsTest, testEnsureSlash)
{
  std::string a = "a";
  a = ze::ensureLeftSlash(a);
  EXPECT_EQ(a, "/a");

  std::string b = "/b";
  b = ze::ensureLeftSlash(b);
  EXPECT_EQ(b, "/b");
}

ZE_UNITTEST_ENTRYPOINT
