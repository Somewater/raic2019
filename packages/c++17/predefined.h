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
  Dan(double distance, Vector3D vector) : distance(distance), normal(vector) {}
};

Vector3D clamp_vector(Vector3D vector, double lenght);

Dan min_dan(Dan v1, Dan v2);

Dan dan_to_plane(Vector3D point, Vector3D point_on_plane, Vector3D plane_normal);

Dan dan_to_sphere_inner(Vector3D point, Vector3D sphere_center, double sphere_radius);

Dan dan_to_sphere_outer(Vector3D point, Vector3D sphere_center, double sphere_radius);

double clamp_float(double value, double minumum, double maximum);

Dan dan_to_arena_quarter(const Arena& arena, Vector3D point);

Dan dan_to_arena(const Arena& arena, Vector3D point0);

#endif //MYSTRATEGY_PREDEFINED_H
