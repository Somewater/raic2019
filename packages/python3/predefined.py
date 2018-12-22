from typing import Tuple

from model import *
from vector import *

def dan_to_plane(point: Vector3D, point_on_plane: Vector3D, plane_normal: Vector3D) -> Tuple[float, Vector3D]:
    return (point - point_on_plane).dot(plane_normal), plane_normal

def dan_to_sphere_inner(point: Vector3D, sphere_center: Vector3D, sphere_radius: float) -> Tuple[float, Vector3D]:
    return sphere_radius - (point - sphere_center).len(), (sphere_center - point).normalize()

def dan_to_sphere_outer(point: Vector3D, sphere_center: Vector3D, sphere_radius: float):
    return (point - sphere_center).len() - sphere_radius, (point - sphere_center).normalize()

def clamp_float(value: float, minumum: float, maximum: float) -> float:
    return max(min(value, maximum), minumum)

def clamp_vector(vector: Vector3D, length: float) -> Vector3D:
    if vector.len() <= length:
        return vector
    return vector.normalize() * length

def min_dan(v1: Tuple[float, Vector3D], v2: Tuple[float, Vector3D]) -> Tuple[float, Vector3D]:
    if v1[0] < v2[0]:
        return v1
    else:
        return v2

def dan_to_arena_quarter(arena: Arena, point: Vector3D) -> Tuple[float, Vector3D]:
    # Ground
    dan = dan_to_plane(point, Vector3D(0, 0, 0), Vector3D(0, 1, 0))

    # Ceiling
    dan = min_dan(dan, dan_to_plane(point, Vector3D(0, arena.height, 0), Vector3D(0, -1, 0)))

    # Side x
    dan = min_dan(dan, dan_to_plane(point, Vector3D(arena.width / 2, 0, 0), Vector3D(-1, 0, 0)))

    # Side z (goal)
    dan = min_dan(dan, dan_to_plane(
            point,
            Vector3D(0, 0, (arena.depth / 2) + arena.goal_depth),
            Vector3D(0, 0, -1)))

    # Side z
    v = Vector3D(point.x, point.y, 0) - \
        Vector3D(
            (arena.goal_width / 2) - arena.goal_top_radius,
            arena.goal_height - arena.goal_top_radius,
            0)
    if point.x >= (arena.goal_width / 2) + arena.goal_side_radius \
            or point.y >= arena.goal_height + arena.goal_side_radius \
            or (v.x > 0 and v.y > 0 and v.len() >= arena.goal_top_radius + arena.goal_side_radius):
        dan = min_dan(dan, dan_to_plane(point, Vector3D(0, 0, arena.depth / 2), Vector3D(0, 0, -1)))

    # Side x & ceiling (goal)
    if point.z >= (arena.depth / 2) + arena.goal_side_radius:
        # x
        dan = min_dan(dan, dan_to_plane(
                point,
                Vector3D(arena.goal_width / 2, 0, 0),
                Vector3D(-1, 0, 0)))
        # y
        dan = min_dan(dan, dan_to_plane(point, Vector3D(0, arena.goal_height, 0), Vector3D(0, -1, 0)))

    # Goal back corners
    assert arena.bottom_radius == arena.goal_top_radius
    if point.z > (arena.depth / 2) + arena.goal_depth - arena.bottom_radius:
        dan = min_dan(dan, dan_to_sphere_inner(
                point,
                Vector3D(
                    clamp_float(
                        point.x,
                        arena.bottom_radius - (arena.goal_width / 2),
                        (arena.goal_width / 2) - arena.bottom_radius,
                    ),
                    clamp_float(
                        point.y,
                        arena.bottom_radius,
                        arena.goal_height - arena.goal_top_radius,
                    ),
                    (arena.depth / 2) + arena.goal_depth - arena.bottom_radius),
                arena.bottom_radius))

    # Corner
    if point.x > (arena.width / 2) - arena.corner_radius \
            and point.z > (arena.depth / 2) - arena.corner_radius:
        dan = min_dan(dan, dan_to_sphere_inner(
                point,
                Vector3D(
                    (arena.width / 2) - arena.corner_radius,
                    point.y,
                    (arena.depth / 2) - arena.corner_radius
                ),
                arena.corner_radius))

    # Goal outer corner
    if point.z < (arena.depth / 2) + arena.goal_side_radius:
        # Side x
        if point.x < (arena.goal_width / 2) + arena.goal_side_radius:
            dan = min_dan(dan, dan_to_sphere_outer(
                    point,
                    Vector3D(
                        (arena.goal_width / 2) + arena.goal_side_radius,
                        point.y,
                        (arena.depth / 2) + arena.goal_side_radius
                    ),
                    arena.goal_side_radius))
        # Ceiling
        if point.y < arena.goal_height + arena.goal_side_radius:
            dan = min_dan(dan, dan_to_sphere_outer(
                    point,
                    Vector3D(
                        point.x,
                        arena.goal_height + arena.goal_side_radius,
                        (arena.depth / 2) + arena.goal_side_radius
                    ),
                    arena.goal_side_radius))
        # Top corner
        o = Vector3D(
            (arena.goal_width / 2) - arena.goal_top_radius,
            arena.goal_height - arena.goal_top_radius,
            0)
        v: Vector3D = point - o
        if v.x > 0 and v.y > 0:
            o = o + v.normalize() * (arena.goal_top_radius + arena.goal_side_radius)
            dan = min_dan(dan, dan_to_sphere_outer(
                    point,
                    Vector3D(o.x, o.y, (arena.depth / 2) + arena.goal_side_radius),
                    arena.goal_side_radius))

    # Goal inside top corners
    if point.z > (arena.depth / 2) + arena.goal_side_radius \
            and point.y > arena.goal_height - arena.goal_top_radius:
        # Side x
        if point.x > (arena.goal_width / 2) - arena.goal_top_radius:
            dan = min_dan(dan, dan_to_sphere_inner(
                    point,
                    Vector3D(
                        (arena.goal_width / 2) - arena.goal_top_radius,
                        arena.goal_height - arena.goal_top_radius,
                        point.z
                    ),
                    arena.goal_top_radius))
        # Side z
        if point.z > (arena.depth / 2) + arena.goal_depth - arena.goal_top_radius:
            dan = min_dan(dan, dan_to_sphere_inner(
                    point,
                    Vector3D(
                        point.x,
                        arena.goal_height - arena.goal_top_radius,
                        (arena.depth / 2) + arena.goal_depth - arena.goal_top_radius
                    ),
                    arena.goal_top_radius))

    # Bottom corners
    if point.y < arena.bottom_radius:
        # Side x
        if point.x > (arena.width / 2) - arena.bottom_radius:
            dan = min_dan(dan, dan_to_sphere_inner(
                    point,
                    Vector3D(
                        (arena.width / 2) - arena.bottom_radius,
                        arena.bottom_radius,
                        point.z
                    ),
                    arena.bottom_radius))
        # Side z
        if point.z > (arena.depth / 2) - arena.bottom_radius \
                and point.x >= (arena.goal_width / 2) + arena.goal_side_radius:
            dan = min_dan(dan, dan_to_sphere_inner(
                    point,
                    Vector3D(
                        point.x,
                        arena.bottom_radius,
                        (arena.depth / 2) - arena.bottom_radius
                    ),
                    arena.bottom_radius))
        # Side z (goal)
        if point.z > (arena.depth / 2) + arena.goal_depth - arena.bottom_radius:
            dan = min_dan(dan, dan_to_sphere_inner(
                    point,
                    Vector3D(
                        point.x,
                        arena.bottom_radius,
                        (arena.depth / 2) + arena.goal_depth - arena.bottom_radius
                    ),
                    arena.bottom_radius))
        # Goal outer corner
        o = Vector3D(
            (arena.goal_width / 2) + arena.goal_side_radius,
            0,
            (arena.depth / 2) + arena.goal_side_radius)
        v = Vector3D(point.x, 0, point.z) - o
        if v.x < 0 and v.y < 0 \
                and v.len() < arena.goal_side_radius + arena.bottom_radius:
            o = o + v.normalize() * (arena.goal_side_radius + arena.bottom_radius)
            dan = min_dan(dan, dan_to_sphere_inner(
                    point,
                    Vector3D(o.x, arena.bottom_radius, o.y),
                    arena.bottom_radius))
        # Side x (goal)
        if point.z >= (arena.depth / 2) + arena.goal_side_radius \
                and point.x > (arena.goal_width / 2) - arena.bottom_radius:
            dan = min_dan(dan, dan_to_sphere_inner(
                    point,
                    Vector3D(
                        (arena.goal_width / 2) - arena.bottom_radius,
                        arena.bottom_radius,
                        point.z
                    ),
                    arena.bottom_radius))
        # Corner
        if point.x > (arena.width / 2) - arena.corner_radius \
                and point.z > (arena.depth / 2) - arena.corner_radius:
            corner_o = Vector3D(
                (arena.width / 2) - arena.corner_radius,
                0,
                (arena.depth / 2) - arena.corner_radius
            )
            n = Vector3D(point.x, 0, point.z) - corner_o
            dist = n.len()
            if dist > arena.corner_radius - arena.bottom_radius:
                n = n * (1/dist)
                o2 = corner_o + n * (arena.corner_radius - arena.bottom_radius)
                dan = min_dan(dan, dan_to_sphere_inner(
                        point,
                        Vector3D(o2.x, arena.bottom_radius, o2.y),
                        arena.bottom_radius))

    # Ceiling corners
    if point.y > arena.height - arena.top_radius:
        # Side x
        if point.x > (arena.width / 2) - arena.top_radius:
            dan = min_dan(dan, dan_to_sphere_inner(
                    point,
                    Vector3D(
                        (arena.width / 2) - arena.top_radius,
                        arena.height - arena.top_radius,
                        point.z,
                    ),
                    arena.top_radius))
        # Side z
        if point.z > (arena.depth / 2) - arena.top_radius:
            dan = min_dan(dan, dan_to_sphere_inner(
                    point,
                    Vector3D(
                        point.x,
                        arena.height - arena.top_radius,
                        (arena.depth / 2) - arena.top_radius,
                    ),
                    arena.top_radius))

        # Corner
        if point.x > (arena.width / 2) - arena.corner_radius \
                and point.z > (arena.depth / 2) - arena.corner_radius:
            corner_o = Vector3D(
                (arena.width / 2) - arena.corner_radius,
                0,
                (arena.depth / 2) - arena.corner_radius
            )
            dv = Vector3D(point.x, 0, point.z) - corner_o
            if dv.len() > arena.corner_radius - arena.top_radius:
                n = dv.normalize()
                o2 = corner_o + n * (arena.corner_radius - arena.top_radius)
                dan = min_dan(dan, dan_to_sphere_inner(
                        point,
                        Vector3D(o2.x, arena.height - arena.top_radius, o2.y),
                        arena.top_radius))

    return dan

def dan_to_arena(arena: Arena, point: Vector3D):
    negate_x = point.x < 0
    negate_z = point.z < 0
    if negate_x:
        point.x = -point.x
    if negate_z:
        point.z = -point.z
    result_distance, result_normal = dan_to_arena_quarter(arena, point)
    if negate_x:
        result_normal.x = -result_normal.x
    if negate_z:
        result_normal.z = -result_normal.z
    return (result_distance, result_normal)