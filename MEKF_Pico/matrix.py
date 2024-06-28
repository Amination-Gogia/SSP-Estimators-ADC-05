class Matrix:
    def __init__(self, rows, cols):
        self.matrix = [[0 for i in range(cols)] for i in range(rows)]
        self.shape = (rows, cols)

    def __repr__(self):
        temp_string = ''
        temp_string += str(self.matrix[0])
        for i in range(1, self.shape[0]):
            temp_string += '\n'
            temp_string += str(self.matrix[i])
        return temp_string

    def __add__(self, other):
        if isinstance(other, Matrix) and self.shape != other.shape:
            raise ValueError(f'Cannot add matrices of order {self.shape} and {other.shape}')
        try:
            sum = Matrix(self.shape[0], self.shape[1])
            for i in range(self.shape[0]):
                for j in range(self.shape[1]):
                    sum.matrix[i][j] = self.matrix[i][j] + other.matrix[i][j]
        except AttributeError as e:
            raise TypeError(f'Not adding matrix to matrix, {e}')
        else:
            return sum

    def __sub__(self, other):
        if isinstance(other, Matrix) and self.shape != other.shape:
            raise ValueError(f'Cannot subtract matrices of order {self.shape} and {other.shape}')
        try:
            diff = Matrix(self.shape[0], self.shape[1])
            for i in range(self.shape[0]):
                for j in range(self.shape[1]):
                    diff.matrix[i][j] = self.matrix[i][j] - other.matrix[i][j]
        except AttributeError as e:
            raise TypeError(f'Not subtracting matrix from matrix, {e}')
        else:
            return diff

    def __mul__(self, other):
        if isinstance(other, Matrix):
            if self.shape[1] != other.shape[0]:
                raise ValueError(f'Cannot multiply matrices of order {self.shape} and {other.shape}')
            product = Matrix(self.shape[0], other.shape[1])
            for i in range(self.shape[0]):
                for j in range(other.shape[1]):
                    for k in range(self.shape[1]):
                        product.matrix[i][j] += self.matrix[i][k] * other.matrix[k][j]
            return product
        elif isinstance(other, (int, float)):
            product = Matrix(self.shape[0], self.shape[1])
            for i in range(self.shape[0]):
                for j in range(self.shape[1]):
                    product.matrix[i][j] = self.matrix[i][j] * other
            return product
        else:
            raise TypeError(f'Unsupported operand type(s) for *: {type(self)} and {type(other)}')

    def __rmul__(self, other):
        return self.__mul__(other)

    def transpose(self):
        transposed = Matrix(self.shape[1], self.shape[0])
        for i in range(self.shape[0]):
            for j in range(self.shape[1]):
                transposed.matrix[j][i] = self.matrix[i][j]
        return transposed

    def __getitem__(self, idx):
        return self.matrix[idx]

    def __setitem__(self, idx, value):
        self.matrix[idx] = value

    def __eq__(self, other):
        if isinstance(other, Matrix) and self.shape == other.shape:
            for i in range(self.shape[0]):
                for j in range(self.shape[1]):
                    if self.matrix[i][j] != other.matrix[i][j]:
                        return False
            return True
        return False

    @classmethod
    def from_list(cls, list_of_lists):
        """Initialise with row vectors"""
        rows = len(list_of_lists)
        cols = len(list_of_lists[0])
        matrix_instance = cls(rows, cols)
        for i in range(rows):
            for j in range(cols):
                matrix_instance.matrix[i][j] = list_of_lists[i][j]
        return matrix_instance

    @classmethod
    def identity(cls, size):
        identity_matrix = cls(size, size)
        for i in range(size):
            identity_matrix.matrix[i][i] = 1
        return identity_matrix

    @classmethod
    def zeros(cls, r, c):
        zero_matrix = cls(r, c)
        for i in range(r):
            for j in range(c):
                zero_matrix.matrix[i][j] = 0
        return zero_matrix
    @classmethod
    def diagonal(cls, *args):
        m = len(args)
        temp = cls(m,m)
        for i in range(m):
            temp.matrix[i][i] = args[i]
        return temp

    def is_square(self):
        return self.shape[0] == self.shape[1]

    def determinant(self):
        if not self.is_square():
            raise ValueError("Determinant is only defined for square matrices.")
        return self._compute_determinant(self.matrix)

    def _compute_determinant(self, mat):
        if len(mat) == 1:
            return mat[0][0]
        if len(mat) == 2:
            return mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0]
        det = 0
        for c in range(len(mat)):
            det += ((-1) ** c) * mat[0][c] * self._compute_determinant(self._get_minor(mat, 0, c))
        return det

    def _get_minor(self, mat, i, j):
        return [row[:j] + row[j+1:] for row in (mat[:i] + mat[i+1:])]

    def inverse(self):
        if not self.is_square():
            raise ValueError("Inverse is only defined for square matrices.")
        det = self.determinant()
        if det == 0:
            raise ValueError("Matrix is not invertible (determinant is zero).")
        return self._compute_inverse()

    def _compute_inverse(self):
        n = self.shape[0]
        matrix_of_minors = [[self._compute_determinant(self._get_minor(self.matrix, i, j)) for j in range(n)] for i in range(n)]
        cofactors = [[matrix_of_minors[i][j] * ((-1) ** (i + j)) for j in range(n)] for i in range(n)]
        adjugate = Matrix.from_list(cofactors).transpose()
        inverse_matrix = (1 / self.determinant()) * adjugate
        return inverse_matrix




    
    

    
# v = Vector(3,2,1,5)
# v[3] = 10
# print(v.modulus)