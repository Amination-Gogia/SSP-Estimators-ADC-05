from matrix import Matrix

class Vector(Matrix):
    def __init__(self, *args):
        length = len(args)
        super().__init__(length, 1)
        for i in range(length):
            self.matrix[i][0] = args[i]
    
    def __getitem__(self, idx):
        return super().__getitem__(idx)[0]
    
    def __setitem__(self, idx, value):
        self.matrix[idx][0] = value

    @property
    def modulus(self):
        sum = 0
        for i in range(self.shape[0]):
            sum += self[i] ** 2
        return sum ** 0.5
    
    def __len__(self):
        return self.shape[0]
    
    def __add__(self, other):
        if not isinstance(other, Vector) or self.shape != other.shape:
            raise ValueError(f'Cannot add vectors of length {self.shape[0]} and {other.shape[0]}')
        return Vector(*[self[i] + other[i] for i in range(self.shape[0])])

    def __sub__(self, other):
        if not isinstance(other, Vector) or self.shape != other.shape:
            raise ValueError(f'Cannot subtract vectors of length {self.shape[0]} and {other.shape[0]}')
        return Vector(*[self[i] - other[i] for i in range(self.shape[0])])

    def __mul__(self, other):
        # if isinstance(other, Vector):
        #     if self.shape != other.shape:
        #         raise ValueError(f'Cannot multiply vectors of length {self.shape[0]} and {other.shape[0]}')
        #     return sum(self[i] * other[i] for i in range(self.shape[0]))
        if isinstance(other, (int, float)):
            return Vector(*[self[i] * other for i in range(self.shape[0])])
        else:
            try:
                return self * other
            except:
                raise TypeError(f'Unsupported operand type(s) for *: {type(self)} and {type(other)}')

    def cross_product(self, other):
        if self.shape[0] != 3 or other.shape[0] != 3:
            raise ValueError('Cross product is only defined for 3-dimensional vectors.')
        a1, a2, a3 = self[0], self[1], self[2]
        b1, b2, b3 = other[0], other[1], other[2]
        return Vector(
            a2 * b3 - a3 * b2,
            a3 * b1 - a1 * b3,
            a1 * b2 - a2 * b1
        )

    @property
    def unit_vector(self):
        mod = self.modulus
        if mod == 0:
            raise ValueError('Cannot normalize a zero vector.')
        return Vector(*[self[i] / mod for i in range(self.shape[0])])
    
    def normalize(self):
        self = self.unit_vector
    
    @property
    def cross_pdt_matrix(self):
        """Defined only for 3 dimensional vectors"""
        if len(self) != 3:
            raise TypeError(f'Cross Product Matrix defined only for 3 Dimensional Vectors, the vector {self}, is {len(self)} dimensional')
        x = self[0]
        y = self[1]
        z = self[2]

        return Matrix.from_list([[0, -z, y], 
                                 [z, 0, -x], 
                                 [-y, x, 0]])
    def __rmul__(self, other):
        if isinstance(other, Matrix):
            if self.shape[0] != other.shape[1]:
                raise ValueError(f'Cannot multiply matrix of shape {other.shape} with vector of length {self.shape[0]}')
            
            # Perform matrix-vector multiplication
            result_vector = Vector(*[0] * other.shape[0])  # Initialize result vector
            for i in range(other.shape[0]):  # Loop over rows of matrix
                for j in range(other.shape[1]):  # Loop over columns of matrix
                    result_vector[i] += other[i][j] * self[j]  # Multiply and accumulate
            
            return result_vector
        
        elif isinstance(other, (int, float)):
            return self * other
        
        else:
            raise TypeError(f'Unsupported operand type(s) for *: {type(self)} and {type(other)}')
        
I = Matrix.identity(4)
v = Vector(1, 2, 3, 4)
# print(type(2 * v))

v2 = 2 * I * v
print(type(v2))