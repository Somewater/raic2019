cdef class Vector3D:
    cdef float x, y, z

    cpdef float len(self)

    cdef Vector3D sub(self, Vector3D other_sub)

    cdef Vector3D add(self, Vector3D other_add)

    cdef Vector3D mul(self, float num_mul)

    cpdef Vector3D normalize(self)

    cpdef float dot(self, Vector3D other)

    cpdef Vector3D min(self, float value)

    cpdef Vector3D max(self, float value)

