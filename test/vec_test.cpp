#include <gtest/gtest.h>

// system includes
#include <assert.h>
#include <cstdint>
#include <iostream>

#include <imp/core/pixel.hpp>

template <typename T>
void testNormalize()
{
  T tmp(1);
  T vec = tmp * 2;
  auto normalized = imp::normalize(vec);
  ASSERT_FLOAT_EQ(1.f, imp::length(normalized));
}

TEST(IMPCoreTestSuite,vecTest)
{
  testNormalize<imp::Vec8uC2>();
  testNormalize<imp::Vec8uC3>();
  testNormalize<imp::Vec8uC4>();

  testNormalize<imp::Vec16uC2>();
  testNormalize<imp::Vec16uC3>();
  testNormalize<imp::Vec16uC4>();

  testNormalize<imp::Vec32sC2>();
  testNormalize<imp::Vec32sC3>();
  testNormalize<imp::Vec32sC4>();

  testNormalize<imp::Vec32fC2>();
  testNormalize<imp::Vec32fC3>();
  testNormalize<imp::Vec32fC4>();
}
