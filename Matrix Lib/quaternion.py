from vector import Vector
from matrix import Matrix

class Quaternion(Vector):
    def __init__(self, w, x, y, z):
        super().__init__(w, x, y, z)


    @property
    def w(self):
        return self[0]

    @property
    def x(self):
        return self[1]

    @property
    def y(self):
        return self[2]

    @property
    def z(self):
        return self[3]

    def conjugate(self):
        return Quaternion(self.w, -self.x, -self.y, -self.z)

    def inverse(self):
        mod_sq = self.modulus ** 2
        if mod_sq == 0:
            raise ValueError('Cannot invert a zero quaternion.')
        conjugate = self.conjugate()
        return (1 / mod_sq) * conjugate

    def epsilon(self):
        qw = self.w
        qy = self.y
        qx = self.x
        qz = self.z

        return Matrix.from_list([   [-qx, -qy, -qz],
                                    [qw, -qz, qy],
                                    [qz, qw, -qx],
                                    [-qy, qx, qw]])
    def __add__(self, other):
        if isinstance(other, Matrix) and other.shape== self.shape:
            return Quaternion(self.w + other[0], self.x + other[1], self.y + other[2], self.z + other[3])
        raise TypeError(f'Not adding quaternion to the correct dimensional matrix, which is {other.shape}')

    def __subtract__(self, other):
        return self.__add__(-1 * other)
    
    def __mul__(self, other):
        if isinstance(other, (float, int)):
            ans = Quaternion(*(self[i] * other for i in range(4)))
            return ans
        raise NotImplementedError('Quaternion multiplication with scalar only implemented in this library')
    
    def attitude_matrix(self):
        w, x, y, z = self.w, self.x, self.y, self.z
        return Matrix.from_list([
            [1 - 2*y**2 - 2*z**2, 2*x*y + 2*w*z, 2*x*z - 2*w*y],
            [2*x*y - 2*w*z, 1 - 2*x**2 - 2*z**2, 2*y*z + 2*w*x],
            [2*x*z + 2*w*y, 2*y*z - 2*w*x, 1 - 2*x**2 - 2*y**2]
        ])

    
# Usage example
q = Quaternion(1, 0, 1, 0)
v = Vector(1, 0, 0)

