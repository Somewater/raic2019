#pragma once
#include "vector3d.h"
#include "model/Arena.h"

using namespace std;
using namespace model;

#ifndef MYSTRATEGY_PREDEFINED_H
#define MYSTRATEGY_PREDEFINED_H

struct Dan {
  double distance;
  Vector3D normal;
  Dan(double distance, const Vector3D& vector) : distance(distance), normal(vector) {}
};

Vector3D clamp_vector(Vector3D& vector, const double lenght);

Dan min_dan(const Dan& v1, const Dan& v2);

Dan dan_to_plane(const Vector3D& point, const Vector3D& point_on_plane, const Vector3D& plane_normal);

Dan dan_to_sphere_inner(const Vector3D& point, const Vector3D& sphere_center, const double sphere_radius);

Dan dan_to_sphere_outer(const Vector3D& point, const Vector3D& sphere_center, const double sphere_radius);

double clamp_float(const double value, const double minumum, const double maximum);

Dan dan_to_arena_quarter(const Arena& arena, const Vector3D& point);

Dan dan_to_arena(const Arena& arena, const Vector3D& point0);

#endif //MYSTRATEGY_PREDEFINED_H
