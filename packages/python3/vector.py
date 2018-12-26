class Vector3D:
    __slots__ = 'x', 'y', 'z'

    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x: float = x
        self.y: float = y
        self.z: float = z

    def len(self) -> float:
        return ((self.x * self.x) + (self.y * self.y) + (self.z * self.z))**0.5

    def __sub__(self, other: 'Vector3D') -> 'Vector3D':
        return Vector3D(self.x - other.x, self.y - other.y, self.z - other.z)

    def __add__(self, other: 'Vector3D') -> 'Vector3D':
        return Vector3D(self.x + other.x, self.y + other.y, self.z + other.z)

    def __mul__(self, num: float) -> 'Vector3D':
        return Vector3D(self.x * num, self.y * num, self.z * num)

    def normalize(self) -> 'Vector3D':
        if self.len() == 0:
            return self
        else:
            return Vector3D(self.x/self.len(), self.y/self.len(), self.z/self.len())

    def dot(self, other: 'Vector3D') -> float:
        return self.x * other.x + self.y * other.y + self.z * other.z

    def min(self, value: float) -> 'Vector3D':
        return Vector3D(min(self.x, value), min(self.y, value), min(self.z, value))

    def max(self, value: float) -> 'Vector3D':
        return Vector3D(max(self.x, value), max(self.y, value), max(self.z, value))

