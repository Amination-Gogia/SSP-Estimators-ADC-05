from vector import Vector
from matrix import Matrix

class Quaternion(Vector):
    """Scalar component is the first component"""
    def __init__(self, q1, q2, q3, q4):
        super().__init__(q1, q2, q3, q4)
    
    def __add__(self, other):
        if not isinstance(other, Quaternion):
            raise ValueError(f'Cannot add quaternion with type {type(other)}')
        return Quaternion(*[self[i] + other[i] for i in range(4)])

    def __sub__(self, other):
        if not isinstance(other, Quaternion):
            raise ValueError(f'Cannot subtract quaternion with type {type(other)}')
        return Quaternion(*[self[i] - other[i] for i in range(4)])

    def __mul__(self, other):
        if isinstance(other, Quaternion):
            # Quaternion multiplication
            w1, x1, y1, z1 = self[0], self[1], self[2], self[3]
            w2, x2, y2, z2 = other[0], other[1], other[2], other[3]
            return Quaternion(
                w1*w2 - x1*x2 - y1*y2 - z1*z2,
                w1*x2 + x1*w2 + y1*z2 - z1*y2,
                w1*y2 - x1*z2 + y1*w2 + z1*x2,
                w1*z2 + x1*y2 - y1*x2 + z1*w2
            )
        elif isinstance(other, (int, float)):
            # Scalar multiplication
            return Quaternion(*[self[i] * other for i in range(4)])
        else:
            raise TypeError(f'Unsupported operand type(s) for *: {type(self)} and {type(other)}')

    def __rmul__(self, other):
        if isinstance(other, Matrix):
            temp = other * self
            components = [temp.matrix[i][0] for i in range(4)]
            return Quaternion(*components)
        elif isinstance(other, (int, float)):
            return self * other
        else:
            raise TypeError(f'Unsupported operand type(s) for *: {type(other)} and {type(self)}')

    @property
    def conjugate(self):
        return Quaternion(self[0], -self[1], -self[2], -self[3])

    @property
    def norm(self):
        return (sum(self[i] ** 2 for i in range(4))) ** 0.5

    def inverse(self):
        norm_sq = self.norm ** 2
        if norm_sq == 0:
            raise ValueError("Cannot invert a quaternion with zero norm.")
        return (1 / norm_sq) * self.conjugate

    def __truediv__(self, other):
        if isinstance(other, Quaternion):
            return self * other.inverse()
        elif isinstance(other, (int, float)):
            return Quaternion(*[self[i] / other for i in range(4)])
        else:
            raise TypeError(f'Unsupported operand type(s) for /: {type(self)} and {type(other)}')
