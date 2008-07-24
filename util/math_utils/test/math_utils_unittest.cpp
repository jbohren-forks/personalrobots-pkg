#include "math_utils/MathExpression.h"
#include "math_utils/math_utils.h"
#include <gtest/gtest.h>

#define TEST_EXPRESSION(a) EXPECT_EQ((a), meval::EvaluateMathExpression(#a))

TEST(MathExpressions, OperatorRecognition){
  EXPECT_TRUE(meval::ContainsOperators("+"));
  EXPECT_TRUE(meval::ContainsOperators("-"));
  EXPECT_TRUE(meval::ContainsOperators("/"));
  EXPECT_TRUE(meval::ContainsOperators("*"));
  EXPECT_FALSE(meval::ContainsOperators("1234567890qwertyuiop[]asdfghjkl;'zxcvbnm,._=?8"));
}

TEST(MathExpressions, BasicOperations){
  EXPECT_EQ(5, meval::EvaluateMathExpression("2+3"));
  EXPECT_EQ(5, meval::EvaluateMathExpression("2 + 3"));
  EXPECT_EQ(10, meval::EvaluateMathExpression("20/2"));
  EXPECT_EQ(-4, meval::EvaluateMathExpression("6 - 10"));
  EXPECT_EQ(24, meval::EvaluateMathExpression("6 * 4"));
}

TEST(MathExpressions, ComplexOperations){
  TEST_EXPRESSION(((3 + 4) / 2.0) + 10);
  TEST_EXPRESSION(7 * (1 + 2 + 3 - 2 + 3.4) / 12.7);
  TEST_EXPRESSION((1 + 2 + 3) - (8.0 / 10));
}

TEST(MathExpression, UnaryMinus){
  TEST_EXPRESSION(-5);
}

TEST(MathExpressions, BadInput){
  //TODO - figure out what good error behavior is and test for it properly
  //EXPECT_EQ(0, meval::EvaluateMathExpression("4.1.3 - 4.1"));
  //EXPECT_EQ(0, meval::EvaluateMathExpression("4.1.3"));
}

TEST(math_utils, BasicOperations){
  EXPECT_EQ(MIN(10, 20), 10);
  EXPECT_EQ(MAX(10, 20), 20);
  EXPECT_EQ(CLAMP(-10, 10, 20), 10);
  EXPECT_EQ(CLAMP(15, 10, 20), 15);
  EXPECT_EQ(CLAMP(25, 10, 20), 20);
}

TEST(math_utils, BoundaryCases){
  EXPECT_EQ(MAX(10, 10), 10);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
