#pragma once

#include <float.h>
#include <math.h>

#define SIGN(x) abs(x) > DBL_EPSILON ? ((x) > 0 ? 1.0 : -1.0) : 0.0
#define DBL_ZERO(x) abs(x) < DBL_EPSILON
#define DBL_EQUAL(x, y) abs((x) - (y)) < DBL_EPSILON

int square_equation(const double a, const double b, const double c, double& x1, double& x2);

int square_equation_non_negative(const double a, const double b, const double c, double& x1, double& x2);