#include "vector3d.h"
#include "predefined.h"

using namespace std;

Vector3D clamp_vector(Vector3D vector, double lenght) {
  if (vector.len() <= lenght) {
    return vector;
  } else {
    return vector.normalize().mul(lenght);
  }
}

Dan min_dan(Dan v1, Dan v2) {
  if (v1.distance < v2.distance) {
    return v1;
  } else {
    return v2;
  }
}

Dan dan_to_plane(Vector3D point, Vector3D point_on_plane, Vector3D plane_normal) {
  return {(point.sub(point_on_plane)).dot(plane_normal), plane_normal};
}

Dan dan_to_sphere_inner(Vector3D point, Vector3D sphere_center, double sphere_radius) {
  return Dan(sphere_radius - (point.sub(sphere_center)).len(), (sphere_center.sub(point)).normalize());
}

Dan dan_to_sphere_outer(Vector3D point, Vector3D sphere_center, double sphere_radius) {
  return Dan((point.sub(sphere_center)).len() - sphere_radius, (point.sub(sphere_center)).normalize());
}

double clamp_float(double value, double minumum, double maximum) {
  return fmax(fmin(value, maximum), minumum);
}

Dan dan_to_arena_quarter(const Arena& arena, Vector3D point) {
  Dan dan = dan_to_plane(point, Vector3D(0, 0, 0), Vector3D(0, 1, 0));

  // Side x
  dan = min_dan(dan, dan_to_plane(point, Vector3D(arena.width / 2, 0, 0), Vector3D(-1, 0, 0)));

  // Side z (goal)
  dan = min_dan(dan, dan_to_plane(
          point,
          Vector3D(0, 0, (arena.depth / 2) + arena.goal_depth),
          Vector3D(0, 0, -1)));

  // Side z
  Vector3D v = Vector3D(point.x, point.y, 0).sub(
      Vector3D(
          (arena.goal_width / 2) - arena.goal_top_radius,
          arena.goal_height - arena.goal_top_radius,
          0));
  if (point.x >= (arena.goal_width / 2) + arena.goal_side_radius \
          or point.y >= arena.goal_height + arena.goal_side_radius \
          or (v.x > 0 and v.y > 0 and v.len() >= arena.goal_top_radius + arena.goal_side_radius)) {
    dan = min_dan(dan, dan_to_plane(point, Vector3D(0, 0, arena.depth / 2), Vector3D(0, 0, -1)));
  }

  // Side x & ceiling (goal)
  if (point.z >= (arena.depth / 2) + arena.goal_side_radius) {
    // x
    dan = min_dan(dan, dan_to_plane(
        point,
        Vector3D(arena.goal_width / 2, 0, 0),
        Vector3D(-1, 0, 0)));
    // y
    dan = min_dan(dan, dan_to_plane(point, Vector3D(0, arena.goal_height, 0), Vector3D(0, -1, 0)));
  }

  // Goal back corners
  //assert arena.bottom_radius == arena.goal_top_radius
  if (point.z > (arena.depth / 2) + arena.goal_depth - arena.bottom_radius) {
    dan = min_dan(dan, dan_to_sphere_inner(
        point,
        Vector3D(
            clamp_float(
                point.x,
                arena.bottom_radius - (arena.goal_width / 2),
                (arena.goal_width / 2) - arena.bottom_radius
            ),
            clamp_float(
                point.y,
                arena.bottom_radius,
                arena.goal_height - arena.goal_top_radius
            ),
            (arena.depth / 2) + arena.goal_depth - arena.bottom_radius),
        arena.bottom_radius));
  }

  // Corner
  if (point.x > (arena.width / 2) - arena.corner_radius and point.z > (arena.depth / 2) - arena.corner_radius) {
    dan = min_dan(dan, dan_to_sphere_inner(
        point,
        Vector3D(
            (arena.width / 2) - arena.corner_radius,
            point.y,
            (arena.depth / 2) - arena.corner_radius
        ),
        arena.corner_radius));
  }

  // Goal outer corner
  if (point.z < (arena.depth / 2) + arena.goal_side_radius) {
    // Side x
    if (point.x < (arena.goal_width / 2) + arena.goal_side_radius) {
      dan = min_dan(dan, dan_to_sphere_outer(
          point,
          Vector3D(
              (arena.goal_width / 2) + arena.goal_side_radius,
              point.y,
              (arena.depth / 2) + arena.goal_side_radius
          ),
          arena.goal_side_radius));
    }

    // Ceiling
    if (point.y < arena.goal_height + arena.goal_side_radius) {
      dan = min_dan(dan, dan_to_sphere_outer(
          point,
          Vector3D(
              point.x,
              arena.goal_height + arena.goal_side_radius,
              (arena.depth / 2) + arena.goal_side_radius
          ),
          arena.goal_side_radius));
    }
    // Top corner
    Vector3D o = Vector3D(
        (arena.goal_width / 2) - arena.goal_top_radius,
        arena.goal_height - arena.goal_top_radius,
        0);
    Vector3D v = point.sub(o);
    if (v.x > 0 and v.y > 0) {
      o = o.add(v.normalize().mul((arena.goal_top_radius + arena.goal_side_radius)));
      dan = min_dan(dan, dan_to_sphere_outer(
          point,
          Vector3D(o.x, o.y, (arena.depth / 2) + arena.goal_side_radius),
          arena.goal_side_radius));
    }
  }

  // Goal inside top corners
  if (point.z > (arena.depth / 2) + arena.goal_side_radius and point.y > arena.goal_height - arena.goal_top_radius) {
    // Side x
    if (point.x > (arena.goal_width / 2) - arena.goal_top_radius) {
      dan = min_dan(dan, dan_to_sphere_inner(
          point,
          Vector3D(
              (arena.goal_width / 2) - arena.goal_top_radius,
              arena.goal_height - arena.goal_top_radius,
              point.z
          ),
          arena.goal_top_radius));
    }
    // Side z
    if (point.z > (arena.depth / 2) + arena.goal_depth - arena.goal_top_radius) {
      dan = min_dan(dan, dan_to_sphere_inner(
          point,
          Vector3D(
              point.x,
              arena.goal_height - arena.goal_top_radius,
              (arena.depth / 2) + arena.goal_depth - arena.goal_top_radius
          ),
          arena.goal_top_radius));
    }
  }

  // Bottom corners
  if (point.y < arena.bottom_radius) {
    // Side x
    if (point.x > (arena.width / 2) - arena.bottom_radius) {
      dan = min_dan(dan, dan_to_sphere_inner(
          point,
          Vector3D(
              (arena.width / 2) - arena.bottom_radius,
              arena.bottom_radius,
              point.z
          ),
          arena.bottom_radius));
    }

    // Side z
    if (point.z > (arena.depth / 2) - arena.bottom_radius and
        point.x >= (arena.goal_width / 2) + arena.goal_side_radius) {
      dan = min_dan(dan, dan_to_sphere_inner(
          point,
          Vector3D(
              point.x,
              arena.bottom_radius,
              (arena.depth / 2) - arena.bottom_radius
          ),
          arena.bottom_radius));
    }

    // Side z (goal)
    if (point.z > (arena.depth / 2) + arena.goal_depth - arena.bottom_radius) {
      dan = min_dan(dan, dan_to_sphere_inner(
          point,
          Vector3D(
              point.x,
              arena.bottom_radius,
              (arena.depth / 2) + arena.goal_depth - arena.bottom_radius
          ),
          arena.bottom_radius));
    }

    // Goal outer corner
    Vector3D o = Vector3D(
        (arena.goal_width / 2) + arena.goal_side_radius,
        (arena.depth / 2) + arena.goal_side_radius,
        0);
    Vector3D v = Vector3D(point.x, point.z, 0).sub(o);
    if (v.x < 0 and v.y < 0 and v.len() < arena.goal_side_radius + arena.bottom_radius) {
      o = o.add(v.normalize().mul(arena.goal_side_radius + arena.bottom_radius));
      dan = min_dan(dan, dan_to_sphere_inner(
          point,
          Vector3D(o.x, arena.bottom_radius, o.y),
          arena.bottom_radius));
    }

    // Side x (goal)
    if (point.z >= (arena.depth / 2) + arena.goal_side_radius and
        point.x > (arena.goal_width / 2) - arena.bottom_radius) {
      dan = min_dan(dan, dan_to_sphere_inner(
          point,
          Vector3D(
              (arena.goal_width / 2) - arena.bottom_radius,
              arena.bottom_radius,
              point.z
          ),
          arena.bottom_radius));
    }

    // Corner
    if (point.x > (arena.width / 2) - arena.corner_radius and point.z > (arena.depth / 2) - arena.corner_radius) {
      Vector3D corner_o = Vector3D(
          (arena.width / 2) - arena.corner_radius,
          (arena.depth / 2) - arena.corner_radius,
          0
      );
      Vector3D n = Vector3D(point.x, point.z, 0).sub(corner_o);
      double dist = n.len();
      if (dist > arena.corner_radius - arena.bottom_radius) {
        n = n.mul(1 / dist);
        Vector3D o2 = corner_o.add(n.mul(arena.corner_radius - arena.bottom_radius));
        dan = min_dan(dan, dan_to_sphere_inner(
            point,
            Vector3D(o2.x, arena.bottom_radius, o2.y),
            arena.bottom_radius));
      }
    }
  }

  // Ceiling corners
  if (point.y > arena.height - arena.top_radius) {
    // Side x
    if (point.x > (arena.width / 2) - arena.top_radius) {
      dan = min_dan(dan, dan_to_sphere_inner(
          point,
          Vector3D(
              (arena.width / 2) - arena.top_radius,
              arena.height - arena.top_radius,
              point.z
          ),
          arena.top_radius));
    }

    // Side z
    if (point.z > (arena.depth / 2) - arena.top_radius) {
      dan = min_dan(dan, dan_to_sphere_inner(
          point,
          Vector3D(
              point.x,
              arena.height - arena.top_radius,
              (arena.depth / 2) - arena.top_radius
          ),
          arena.top_radius));
    }

    // Corner
    if (point.x > (arena.width / 2) - arena.corner_radius and point.z > (arena.depth / 2) - arena.corner_radius) {
      Vector3D corner_o = Vector3D(
          (arena.width / 2) - arena.corner_radius,
          (arena.depth / 2) - arena.corner_radius,
          0
      );
      Vector3D dv = Vector3D(point.x, point.z, 0).sub(corner_o);
      if (dv.len() > arena.corner_radius - arena.top_radius) {
        Vector3D n = dv.normalize();
        Vector3D o2 = corner_o.add(n.mul(arena.corner_radius - arena.top_radius));
        dan = min_dan(dan, dan_to_sphere_inner(
            point,
            Vector3D(o2.x, arena.height - arena.top_radius, o2.y),
            arena.top_radius));
      }
    }
  }

  return dan;
}

Dan dan_to_arena(const Arena& arena, Vector3D point) {
  bool negate_x = point.x < 0;
  bool negate_z = point.z < 0;
  if (negate_x) {
      point.x = -point.x;
  }
  if (negate_z) {
    point.z = -point.z;
  }
  Dan dan = dan_to_arena_quarter(arena, point);
  Vector3D result_normal = dan.normal;
  double result_distance = dan.distance;
  if (negate_x) {
    result_normal.x = -result_normal.x;
  }
  if (negate_z) {
    result_normal.z = -result_normal.z;
  }
  return Dan(result_distance, result_normal);
}