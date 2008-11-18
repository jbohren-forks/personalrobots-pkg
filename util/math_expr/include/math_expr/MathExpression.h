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

/** \Author Ioan Sucan */

#ifndef MATH_EXPRESSION_
#define MATH_EXPRESSION_

#include <string>

/** @htmlinclude ../../manifest.html

    Evaluate simple mathematical expressions */

namespace meval
{

/** \brief Callback for evaluating a variable */
typedef double (*ExpressionVariableFn)(void *, std::string&);

/** Given a mathematical expression in string format, compute
 * what this expression evaluates to.  The expression can be
 * arbitrarily parenthesised, but can only include the +, -, *, /
 * mathematical operators. In addition to floating point constants,
 * this function allows the use of named constants, if a callback is
 * provided to evaluate those named constants. A data pointer for the

 * callback is provided for convenience.
 *
 * Returns nan on error.
 */
double EvaluateMathExpression(const char *expression, ExpressionVariableFn var = NULL, void *data = NULL);
double EvaluateMathExpression(const std::string &expression, ExpressionVariableFn var = NULL, void *data = NULL);

/** Returns true if the expression contains any known mathematical operators */
bool   ContainsOperators(const char        *expression);
bool   ContainsOperators(const std::string &expression);

}

#endif

