#pragma once
#ifndef MYSTRATEGY_VECTOR3D_H
#define MYSTRATEGY_VECTOR3D_H

#include "math.h"

struct Vector3D {
  double x;
  double y;
  double z;

  Vector3D() : x(0), y(0), z(0) {}

  Vector3D(double x, double z, double y) : x(x), y(y), z(z) {}

  void set(double x_, double z_, double y_) { x = x_; z = z_; y = y_; }

  double len() {
    if (_len < 0) _len = sqrt(x*x + y*y + z*z);
    return _len;
  }

  double lenTo(double x_, double z_, double y_) { return sqrt((x-x_)*(x-x_) + (y-y_)*(y-y_) + (z-z_)*(z-z_)); }

  double lenTo(Vector3D p) { return lenTo(p.x, p.z, p.y); }

  //Vector3D operator+(Vector3D p) { return { x+p.x, z+p.z, y+p.y };}
  Vector3D add(Vector3D other) { return {x + other.x, y + other.y, z + other.z}; }
  Vector3D sub(Vector3D other) { return {x - other.x, y - other.y, z - other.z}; }
  Vector3D mul(double value) { return {x * value, y * value, z * value}; }

  //Vector3D operator*(double val) { return { x*val, z*val, y*val }; }

  //void operator*=(double val) { x *= val; z *= val; y *= val; }


  double dot(Vector3D other) {
    return x * other.x + y * other.y + z * other.z;
  }

  Vector3D min(double value) {
    return Vector3D((value < x ? value : x),
                    (value < y ? value : y),
                    (value < z ? value : z));
  }

  Vector3D normalize() {
    if (x == 0 and y == 0 and z == 0) {
      return Vector3D(x ,y ,z);
    } else {
      double l = this->len();
      return Vector3D(x / l, y / l, z / l);
    }
  }

private:
  double _len = -1;
};

#endif //MYSTRATEGY_VECTOR3D_H
