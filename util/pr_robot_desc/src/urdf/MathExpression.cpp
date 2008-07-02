#include "urdf/MathExpression.h"
#include <cstring>
#include <cmath>
#include <deque>

double EvaluateMathExpression(const char  *expression, ExpressionVariableFn var, void *data)
{
    return EvaluateMathExpression(std::string(expression), var, data);
}

double EvaluateMathExpression(const std::string &expression, ExpressionVariableFn var, void *data)
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
		if (!((exp[i] <= '9' && exp[i] >= '0') || exp[i] == '+' || exp[i] == '-' || exp[i] == '.'))
		{
		    variable = true;
		    break;
		}
	    if (variable)
	    {
		if (var)
		    return var(data, exp);		
	    }
	    else
		return atof(exp.c_str());
	}
    }
    else
    {
	unsigned int pos = ops[0];
	std::string exp1 = exp.substr(0, pos);
	std::string exp2 = exp.substr(pos + 1);
	switch (exp[pos])
	{
	case '+':
	    return EvaluateMathExpression(exp1, var, data) + EvaluateMathExpression(exp2, var, data);
	case '-':
	    return EvaluateMathExpression(exp1, var, data) - EvaluateMathExpression(exp2, var, data);
	case '*':
	    return EvaluateMathExpression(exp1, var, data) * EvaluateMathExpression(exp2, var, data);
	case '/':
	    return EvaluateMathExpression(exp1, var, data) / EvaluateMathExpression(exp2, var, data);
	}
    }
        
    return 0.0;
}
