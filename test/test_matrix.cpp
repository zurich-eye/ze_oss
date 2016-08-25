// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#include <cmath>

#include <ze/common/test_entrypoint.h>
#include <ze/common/matrix.h>
#include <ze/common/types.h>

TEST(MatrixTests, testVectorSlice)
{
  using namespace ze;

  VectorX M(5);
  M << 1, 2, 3, 4, 6;
  std::vector<uint32_t> indices { 0, 2, 3 };

  M = getVectorElements(M, indices);

  VectorX A_expected(3);
  A_expected << 1, 3, 4;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(M, A_expected));
}

TEST(MatrixTests, testColumnSlice)
{
  using namespace ze;

  Matrix2X M(2, 5);
  M << 1, 2, 3, 4, 6,
       7, 8, 9, 10, 11;
  std::vector<uint32_t> indices { 0, 2, 3 };

  M = getMatrixCols(M, indices);

  Matrix2X A_expected(2,3);
  A_expected << 1, 3, 4, 7, 9, 10;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(M, A_expected));
}

ZE_UNITTEST_ENTRYPOINT
