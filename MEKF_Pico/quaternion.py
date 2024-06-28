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
        elif isinstance(other, Quaternion):
            w = self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z
            x = self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y
            y = self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x
            z = self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w
            return Quaternion(w, x, y, z)
        raise NotImplementedError('Quaternion multiplication with scalar only implemented in this library')
    
    def attitude_matrix(self):
        w, x, y, z = self.w, self.x, self.y, self.z
        return Matrix.from_list([
            [1 - 2*y**2 - 2*z**2, 2*x*y + 2*w*z, 2*x*z - 2*w*y],
            [2*x*y - 2*w*z, 1 - 2*x**2 - 2*z**2, 2*y*z + 2*w*x],
            [2*x*z + 2*w*y, 2*y*z - 2*w*x, 1 - 2*x**2 - 2*y**2]
        ])

def quaternion_from_attitude_matrix(M):
    # Ensure M is a 3x3 matrix
    assert ((isinstance(M, Matrix) and M.shape == (3,3)), "Input must be a 3x3 matrix")

    tr = M[0][0] + M[1][1] + M[2][2]

    if tr > 0:
        # Case 1: Trace is positive
        S = (tr + 1.0)**0.5 * 2  # S = 4 * qw
        qw = 0.25 * S
        qx = (-M[2][1] + M[1][2]) / S
        qy = (-M[0][2] + M[2][0]) / S
        qz = (-M[1][0] + M[0][1]) / S
    elif M[0][0] > M[1][1] and M[0][0] > M[2][2]:
        # Case 2: M[0][0] is the largest diagonal element
        S = (1.0 + M[0][0] - M[1][1] - M[2][2])**0.5 * 2  # S = 4 * qx
        qw = (- M[2][1] + M[1][2]) / S
        qx = 0.25 * S
        qy = (M[0][1] + M[1][0]) / S
        qz = (M[0][2] + M[2][0]) / S
    elif M[1][1] > M[2][2]:
        # Case 3: M[1][1] is the largest diagonal element
        S = (1.0 + M[1][1] - M[0][0] - M[2][2])**0.5 * 2  # S = 4 * qy
        qw = (- M[0][2] + M[2][0]) / S
        qx = (M[0][1] + M[1][0]) / S
        qy = 0.25 * S
        qz = (M[1][2] + M[2][1]) / S
    else:
        # Case 4: M[2][2] is the largest diagonal element
        S = (1.0 + M[2][2] - M[0][0] - M[1][1])**0.5 * 2  # S = 4 * qz
        qw = (- M[1][0] + M[0][1]) / S
        qx = (M[0][2] + M[2][0]) / S
        qy = (M[1][2] + M[2][1]) / S
        qz = 0.25 * S

    q = Quaternion(qw, qx, qy, qz)
    q.normalize()
    return q

# Usage example
if __name__ == '__main__':
    q = Quaternion(1, 0, 1, 0)
    v = Vector(1, 0, 0)

