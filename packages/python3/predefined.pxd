from vector cimport Vector3D

cdef class ArenaStruct:
    cdef float width, height, depth, bottom_radius, top_radius, corner_radius, goal_top_radius, goal_width, goal_height, goal_depth, goal_side_radius

cdef class Dan:
    cdef Vector3D normal
    cdef float distance

cdef Dan dan_to_arena_quarter(ArenaStruct arena, Vector3D point)