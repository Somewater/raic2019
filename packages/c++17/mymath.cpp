#include "mymath.h"

int square_equation(const double a, const double b, const double c, double& x1, double& x2) {
  double D = b * b - 4 * a * c;
  if (DBL_ZERO(D)) {
    x1 = -b / (2 * a);
    return 1;
  } else if (D > 0) {
    double sqrtD = sqrt(D);
    x1 = (-b + sqrtD) / (2 * a);
    x2 = (-b - sqrtD) / (2 * a);
    return 2;
  } else {
    return 0;
  }
}

int square_equation_non_negative(const double a, const double b, const double c, double& x1, double& x2) {
  double D = b * b - 4 * a * c;
  if (DBL_ZERO(D)) {
    x1 = -b / (2 * a);
    if (x1 >= 0) {
      return 1;
    } else {
      return 0;
    }
  } else if (D > 0) {
    double sqrtD = sqrt(D);
    x1 = (-b + sqrtD) / (2 * a);
    x2 = (-b - sqrtD) / (2 * a);
    if (x1 >= 0 && x2 >= 0) {
      return 2;
    } else if (x1 >=0) {
      return 1;
    } else if (x2 > 0) {
      x1 = x2;
      return 1;
    } else {
      return 0;
    }
  } else {
    return 0;
  }
}