/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \author Ioan Sucan */

#include "math_expr/MathExpression.h"
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <deque>
#include <cstdlib>

bool meval::ContainsOperators(const char *expression)
{
  return ContainsOperators(std::string(expression));
}

bool meval::ContainsOperators(const std::string &expression)
{
  return expression.find_first_of("+-*/") != std::string::npos;
}

double meval::EvaluateMathExpression(const char *expression, ExpressionVariableFn var, void *data)
{
  return EvaluateMathExpression(std::string(expression), var, data);
}

double meval::EvaluateMathExpression(const std::string &expression, ExpressionVariableFn var, void *data)
{
  std::string exp = expression;
  while (!exp.empty())
  {
    std::string::size_type pos = exp.find_first_of("\n\t ");
    if (pos != std::string::npos)
      exp.erase(pos, 1);
    else
      break;
  }

  /* remove brackets, if needed */
  while (exp.size() > 0 && exp[0] == '(' && exp[exp.size() - 1] == ')')
  {
    int depth = 1;
    bool done = false;
    for (unsigned int i = 1 ; i < exp.size() - 1; ++i)
    {
      if (exp[i] == '(')
        depth++;
      if (exp[i] == ')')
        depth--;
      if (depth == 0)
      {
        done = true;
        break;
      }
    }
    if (done)
      break;
    else
    {
      exp.erase(exp.size() - 1);
      exp.erase(0, 1);
    }
  }

  /* find possible operations */
  int depth = 0;
  std::deque<unsigned int> ops;
  for (unsigned int i = 0 ; i < exp.size() ; ++i)
  {
    if (depth == 0 && (exp[i] == '+' || exp[i] == '-'))
      ops.push_front(i);
    if (depth == 0 && (exp[i] == '*' || exp[i] == '/'))
      ops.push_back(i);
    if (exp[i] == '(')
      depth++;
    if (exp[i] == ')')
      depth--;
  }

  if (ops.empty())
  {
    if (!exp.empty())
    {
      bool variable = false;
      for (unsigned int i = 0 ; i < exp.size() ; ++i)
      {
        if (!((exp[i] <= '9' && exp[i] >= '0') || exp[i] == '+' || exp[i] == '-' || exp[i] == '.'))
        {
          variable = true;
          break;
        }
      }

      if (variable)
      {
        if (var)
          return var(data, exp);
        return NAN;
      }
      else
      {
        return atof(exp.c_str());
      }
    }
  }
  else
  {
    unsigned int pos = ops[0];
    std::string exp1 = exp.substr(0, pos);
    std::string exp2 = exp.substr(pos + 1);
    double val1, val2;
    val1 = EvaluateMathExpression(exp1, var, data);
    val2 = EvaluateMathExpression(exp2, var, data);

    // Hack: handles unary minus
    if (exp1.size() == 0)
      val1 = 0.0;

    switch (exp[pos])
    {
    case '+':
      return val1 + val2;
    case '-':
      return val1 - val2;
    case '*':
      return val1 * val2;
    case '/':
      return val1 / val2;
    default:
      return NAN;
    }
  }

  return NAN;
}
