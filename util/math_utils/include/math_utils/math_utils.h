#ifndef MATH_UTIL_H
#define MATH_UTIL_H

/** Minimum between two doubles */
static inline double MIN(double a, double b)
{
    return a < b ? a : b;
}

/** Maximum between two doubles */
static inline double MAX(double a, double b)
{
    return a > b ? a : b;
}

/** Clamp value a between b and c */
static inline double CLAMP(double a, double b, double c)
{
    return MIN(MAX(b, a), c);
}

#endif
