#ifndef MATH_EXPRESSION_
#define MATH_EXPRESSION_

#include <string>

/* callback for evaluating a variable */
typedef double (*ExpressionVariableFn)(void *, std::string&);

double EvaluateMathExpression(const char        *expression, ExpressionVariableFn var = NULL, void *data = NULL);
double EvaluateMathExpression(const std::string &expression, ExpressionVariableFn var = NULL, void *data = NULL);

#endif
