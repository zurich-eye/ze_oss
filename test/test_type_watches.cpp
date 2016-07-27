#include <iostream>
#include <string>

#include <ze/common/test_entrypoint.h>
#include <ze/pangolin/type_watches.hpp>

#define ASSERT_OPERATOR(TYPE, LHS, RHS, OP)       \
{                                                 \
  PrimitiveTypeWrapperImpl<TYPE> wrappedLhs(LHS, false); \
  PrimitiveTypeWrapperImpl<TYPE> wrappedRhs(RHS, false); \
  TYPE plainLhs = LHS;                            \
  TYPE plainRhs = RHS;                            \
  TYPE result = plainLhs OP plainRhs;             \
  EXPECT_EQ(wrappedLhs OP wrappedRhs, result);    \
  EXPECT_EQ(wrappedLhs OP plainRhs, result);      \
  EXPECT_EQ(plainLhs OP wrappedRhs, result);      \
}

#define ASSERT_OPERATOR_TYPE_INT(TYPE)      \
  ASSERT_OPERATOR(TYPE, 10, 5, +);        \
  ASSERT_OPERATOR(TYPE, 10, 5, -);        \
  ASSERT_OPERATOR(TYPE, 10, 5, *);        \
  ASSERT_OPERATOR(TYPE, 10, 5, /);        \
  ASSERT_OPERATOR(TYPE, 10, 5, %);        \
  ASSERT_OPERATOR(TYPE, 5, 10, %);        \
  ASSERT_OPERATOR(TYPE, 10, 5, &);        \
  ASSERT_OPERATOR(TYPE, 10, 5, |);        \
  ASSERT_OPERATOR(TYPE, 10, 5, ^);        \
  ASSERT_OPERATOR(TYPE, 10, 5, |);

#define ASSERT_OPERATOR_TYPE_DOUBLE(TYPE)      \
  ASSERT_OPERATOR(TYPE, 10, 5, +);           \
  ASSERT_OPERATOR(TYPE, 10, 5, -);           \
  ASSERT_OPERATOR(TYPE, 10, 5, *);           \
  ASSERT_OPERATOR(TYPE, 10, 5, /);

TEST(TypeWatchesTest, testAllTypes)
{
  using namespace ze;

  ASSERT_OPERATOR_TYPE_DOUBLE(double);
  ASSERT_OPERATOR_TYPE_DOUBLE(float);
  ASSERT_OPERATOR_TYPE_DOUBLE(long double);

  ASSERT_OPERATOR_TYPE_INT(int);
  ASSERT_OPERATOR_TYPE_INT(unsigned);
  ASSERT_OPERATOR_TYPE_INT(short);
  ASSERT_OPERATOR_TYPE_INT(char);
  ASSERT_OPERATOR_TYPE_INT(unsigned long long);
}

ZE_UNITTEST_ENTRYPOINT
