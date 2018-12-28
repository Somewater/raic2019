cdef class Vector3D:
    def __cinit__(self, float x, float y, float z):
        self.x = x
        self.y = y
        self.z = z
        self._len = -2.0

    cpdef float len(self):
        if self._len < -1.0:
            self._len = ((self.x * self.x) + (self.y * self.y) + (self.z * self.z))**0.5
        return self._len

    cdef Vector3D sub(self, Vector3D other_sub):
        return Vector3D(self.x - other_sub.x, self.y - other_sub.y, self.z - other_sub.z)

    #def __sub__(self, Vector3D other_sub):
    #    return Vector3D(self.get_x() - other_sub.get_x(), self.get_y() - other_sub.get_y(), self.get_z() - other_sub.get_z())

    cdef Vector3D add(self, Vector3D other_add):
        return Vector3D(self.x + other_add.x, self.y + other_add.y, self.z + other_add.z)

    #def __add__(self, Vector3D other_add):
    #    return Vector3D(self.get_x() + other_add.get_x(), self.get_y() + other_add.get_y(), self.get_z() + other_add.get_z())

    cdef Vector3D mul(self, float num_mul):
        return Vector3D(self.x * num_mul, self.y * num_mul, self.z * num_mul)

    #def __mul__(self, float other_mul):
    #    return Vector3D(self.get_x() * other_mul, self.get_y() * other_mul, self.get_z() * other_mul)

    cpdef Vector3D normalize(self):
        if self.len() == 0:
            return self
        else:
            return Vector3D(self.x/self.len(), self.y/self.len(), self.z/self.len())

    cpdef float dot(self, Vector3D other):
        return self.x * other.x + self.y * other.y + self.z * other.z

    cpdef Vector3D min(self, float value):
        return Vector3D(min(self.x, value), min(self.y, value), min(self.z, value))

    cpdef Vector3D max(self, float value):
        return Vector3D(max(self.x, value), max(self.y, value), max(self.z, value))

    def get_x(self):
        return self.x

    def set_x(self, value):
        self.x = value

    def get_y(self):
        return self.y

    def set_y(self, value):
        self.y = value

    def get_z(self):
        return self.z

    def set_z(self, value):
        self.z = value

    def __repr__(self):
        return "(%f, %f, %f)" % (self.x, self.y, self.z)

    def __str__(self):
        return self.__repr__()

