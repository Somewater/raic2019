#pragma once
#ifndef MYSTRATEGY_VECTOR3D_H
#define MYSTRATEGY_VECTOR3D_H

#include "math.h"
#include <iostream>

using namespace std;

struct Vector3D {
  double x;
  double y;
  double z;

  Vector3D() : x(0), y(0), z(0) {}

  Vector3D(double x, double y, double z) : x(x), y(y), z(z) {}

  inline
  void set(double x_, double y_, double z_) { x = x_; y = y_; z = z_; }

  inline
  double len() {
    if (_len < 0) _len = sqrt(x*x + y*y + z*z);
    return _len;
  }

  inline
  double distance_to(const double x_, const double y_, const double z_) const { return sqrt((x-x_)*(x-x_) + (y-y_)*(y-y_) + (z-z_)*(z-z_)); }

  inline
  double distance_to(const Vector3D& p) const { return distance_to(p.x, p.y, p.z); }

  //Vector3D operator+(Vector3D p) { return { x+p.x, z+p.z, y+p.y };}
  Vector3D add(const Vector3D& other) const { return Vector3D(x + other.x, y + other.y, z + other.z); }
  Vector3D sub(const Vector3D& other) const { return Vector3D(x - other.x, y - other.y, z - other.z); }
  Vector3D mul(const double value) const { return Vector3D(x * value, y * value, z * value); }

  //Vector3D operator*(double val) { return { x*val, z*val, y*val }; }

  //void operator*=(double val) { x *= val; z *= val; y *= val; }


  inline
  double dot(const Vector3D& other) const {
    return x * other.x + y * other.y + z * other.z;
  }

  inline
  Vector3D clamp(const double lenght) {
    if (len() <= lenght) {
      return {x, y, z};
    } else {
      // return normalize().mul(lenght);
      double m = lenght / len();
      return Vector3D(m * x, m * y, m * z);
    }
  }

  inline
  Vector3D normalize() {
    if (x == 0 and y == 0 and z == 0) {
      return Vector3D(x ,y ,z);
    } else {
      double l = this->len();
      return Vector3D(x / l, y / l, z / l);
    }
  }

  inline
  Vector3D plane() const {
    return Vector3D(x, 0, z);
  }

  // this.add(other.mul(value))
  inline
  Vector3D addMul(const Vector3D& other, const double value) const {
    return Vector3D(x + other.x * value, y + other.y * value, z + other.z * value);
  }

  // this.sub(other.mul(value))
  inline
  Vector3D subMul(const Vector3D& other, const double value) const {
    return Vector3D(x - other.x * value, y - other.y * value, z - other.z * value);
  }

private:
  double _len = -1;
};

ostream& operator<<(ostream& stream, const Vector3D& vector);

#endif //MYSTRATEGY_VECTOR3D_H
