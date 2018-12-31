//
// Created by Pavel Naydenov on 30/12/2018.
//

#include <iostream>
#include "vector3d.h"

using namespace std;

ostream& operator<<(ostream& stream, const Vector3D& vector) {
  return stream << '(' << vector.x << ',' << vector.y << ',' << vector.z << ')';
}