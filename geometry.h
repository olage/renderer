#ifndef __GEOMETRY_H__
#define __GEOMETRY_H__

#include <cassert>
#include <iostream>
#include <cmath>
#include <array>

template <size_t Dims, typename T>
class VectorTemplate {
public:
  VectorTemplate() {
    for (int i = 0; i < Dims; ++i) {
      data_[i] = T();
    }
  }

  template <typename U>
  VectorTemplate(std::initializer_list<U> l) {
    std::copy(l.begin(), l.end(), data_.begin());
  }

  T& operator[] (const size_t idx) {
    return data_[idx];
  }

  const T& operator[] (const size_t idx) const {
    return data_[idx];
  }

  T Norm() {
    T result = 0;
    for (size_t i = 0; i < Dims; ++i) {
      result += data_[i] * data_[i];
    }
    return std::sqrt(result);
  }

  VectorTemplate<3, T> Normalize() {
    return (*this) / Norm();
  }

private:
  //T data_[Dims];
  std::array<T, Dims> data_;
};

template <typename T>
class VectorTemplate<2, T> {
public:
  VectorTemplate()
    : x(), y()
  {}

  VectorTemplate(T x, T y)
    : x(x), y(y)
  {}

  T& operator[] (const size_t idx) {
    switch (idx) {
    case 0: return x;
    case 1: return y;
    }
  }

  const T& operator[] (const size_t idx) const {
    switch (idx) {
    case 0: return x;
    case 1: return y;
    }
  }

  T x;
  T y;
};

template <typename T>
class VectorTemplate<3, T> {
public:
  VectorTemplate()
    : x(), y(), z()
  {}

  template <typename U>
  VectorTemplate(U x, U y, U z)
    : x(x), y(y), z(z)
  {}

  T& operator[] (const size_t idx) {
    switch (idx) {
    case 0: return x;
    case 1: return y;
    case 2: return z;
    }
  }

  const T& operator[] (const size_t idx) const {
    switch (idx) {
    case 0: return x;
    case 1: return y;
    case 2: return z;
    }
  }

  T Norm() {
    return std::sqrt(x*x + y*y + z*z);
  }

  VectorTemplate<3, T> Normalize() {
    return (*this) / Norm();
  }

  T x;
  T y;
  T z;
};

template <size_t Dims, typename T>
VectorTemplate<Dims, T> operator-(const VectorTemplate<Dims, T>& lhs, const VectorTemplate<Dims, T>& rhs) {
  VectorTemplate<Dims, T> result;
  for (size_t i = 0; i < Dims; ++i) {
    result[i] = lhs[i] - rhs[i];
  }
  return result;
}

template <size_t Dims, typename T>
VectorTemplate<Dims + 1, T> ToHomogeneous(const VectorTemplate<Dims, T>& vector) {
  VectorTemplate<Dims + 1, T> result;
  for (size_t i = 0; i < Dims; ++i) {
    result[i] = vector[i];
  }
  result[Dims] = 1.f;
  return result;
}

template <size_t Dims, typename T>
VectorTemplate<Dims - 1, T> FromHomogeneous(const VectorTemplate<Dims, T>& vector) {
  VectorTemplate<Dims - 1, T> result;
  for (size_t i = 0; i < Dims - 1; ++i) {
    result[i] = vector[i] / vector[Dims - 1];
  }
  return result;
}

template <size_t Dims, typename T>
T Min(const VectorTemplate<Dims, T>& vector) {
  T result = vector[0];
  for (size_t i = 1; i < Dims; ++i) {
    result = std::min(result, vector[i]);
  }
  return result;
}

template <size_t Dims, typename T>
T Max(const VectorTemplate<Dims, T>& vector) {
  T result = vector[0];
  for (size_t i = 1; i < Dims; ++i) {
    result = std::max(result, vector[i]);
  }
  return result;
}

template <typename T>
VectorTemplate<3, T> CrossProduction(const VectorTemplate<3, T>& v1, const VectorTemplate<3, T>& v2) {
  return{ v1[1]*v2[2] - v1[2]*v2[1], v1[2]*v2[0] - v1[0]*v2[2], v1[0]*v2[1] - v1[1]*v2[0] };
}

template <size_t N, typename T>
T DotProduction(const VectorTemplate<N, T> &v1, const VectorTemplate<N, T> &v2) {
  T result = 0;
  for (size_t i = 0; i < N; ++i) {
    result += v1[i] * v2[i];
  }
  return result;
}

template <size_t Dims, typename T>
std::ostream& operator<<(std::ostream& out, const VectorTemplate<Dims, T>& vector) {
  out << "[";
  for (size_t i = 0; i < Dims; ++i) {
    if (i) {
      out << ", ";
    }
    out << vector[i];
  }
  out << "]";
  return out;
}

template <size_t Dims, typename T>
std::istream& operator>>(std::istream& in, VectorTemplate<Dims, T>& vector) {
  for (size_t i = 0; i < Dims; ++i) {
    in >> vector[i];
  }
  return in;
}

template <size_t RowDims, size_t ColDims, typename T>
class MatrixTemplate {
public:
  using RowType = VectorTemplate<ColDims, T>;
  using ColType = VectorTemplate<RowDims, T>;

  RowType& operator[] (const size_t idx) {
    return rows_[idx];
  }

  const RowType& operator[] (const size_t idx) const {
    assert(idx < RowDims);
    return rows_[idx];
  }

  RowType& GetRow(const size_t idx) {
    assert(idx < RowDims);
    return (*this)[idx];
  }

  const RowType& GetRow(const size_t idx) const {
    assert(idx < RowDims);
    return (*this)[idx];
  }

  ColType GetCol(const size_t idx) const {
    assert(idx < ColDims);
    ColType result;
    for (size_t i = 0; i < result.size(); ++i) {
      result[i] = rows_[i][idx];
    }
    return result;
  }

  void SetCol(const size_t idx, const ColType& col) {
    assert(idx < ColDims);
    for (size_t i = 0; i < RowDims; ++i) {
      rows_[i][idx] = col[i];
    }
  }

  static MatrixTemplate<RowDims, ColDims, T> Identity() {
    MatrixTemplate<RowDims, ColDims, T> result;
    for (int i = 0; i < std::min(RowDims, ColDims); ++i) {
      result[i][i] = 1;
    }
    return result;
  }

  MatrixTemplate<ColDims, RowDims, T> Transpose() {
    MatrixTemplate<ColDims, RowDims, T> result;
    for (int i = 0; i < RowDims; ++i) {
      for (int j = 0; j < ColDims; ++j) {
        result[j][i] = rows_[i][j];
      }
    }
    return result;
  }

  T Det() const {
    T result = 0;
    for (size_t i = std::min(ColDims, RowDims); i--; result += rows_[0][i] * Cofactor(0, i));
    return result;
  }

  MatrixTemplate<RowDims - 1, ColDims - 1, T> GetMinor(size_t row, size_t col) const {
    MatrixTemplate<RowDims - 1, ColDims - 1, T> result;
    for (size_t i = 0; i < RowDims; ++i) {
      for (size_t j = 0; j < ColDims; ++j) {
        result[i][j] = rows_[i < row ? i : i + 1][j < col ? j : j + 1];
      }
    }
    return result;
  }

  T Cofactor(size_t row, size_t col) const {
    return GetMinor(row, col).Det()*((row + col) % 2 ? -1 : 1);
  }

  MatrixTemplate<RowDims, ColDims, T> Adjugate() const {
    MatrixTemplate<RowDims, ColDims, T> result;
    for (size_t i = 0; i < RowDims; ++i) {
      for (size_t j = 0; j < ColDims; ++j) {
        result[i][j] = Cofactor(i, j);
      }
    }
    return result;
  }

  MatrixTemplate<RowDims, ColDims, T> InvertTranspose() {
    MatrixTemplate<RowDims, ColDims, T> result = Adjugate();
    T tmp = result[0] * rows_[0];
    return result / tmp;
  }

  MatrixTemplate<RowDims, ColDims, T>  Invert() {
    return InvertTranspose().Transpose();
  }

private:
  VectorTemplate<ColDims, T> rows_[RowDims];
};

template <size_t RowDims, size_t ColDims, typename T>
VectorTemplate<RowDims, T> operator*(const MatrixTemplate<RowDims, ColDims, T>& lhs, const VectorTemplate<ColDims, T>& rhs) {
  VectorTemplate<RowDims, T> result;
  for (size_t i = 0; i < RowDims; ++i) {
    result[i] = DotProduction(lhs[i], rhs);
  }
  return result;
}

template <size_t R1, size_t C1, size_t C2, typename T>
MatrixTemplate<R1, C2, T> operator*(const MatrixTemplate<R1, C1, T>& lhs, const MatrixTemplate<C1, C2, T>& rhs) {
  MatrixTemplate<R1, C2, T> result;
  for (size_t i = 0; i < R1; ++i) {
    for (size_t j = 0; j < C2; ++j) {
      for (size_t k = 0; k < C1; ++k) {
        result[i][j] += lhs[i][k] * rhs[k][j];
      }
    }
  }
  return result;
}

template <size_t Dims, typename T>
VectorTemplate<Dims, T> operator/(VectorTemplate<Dims, T> lhs, T rhs) {
  for (size_t i = 0; i < Dims; ++i) {
    lhs[i] /= rhs;
  }
  return lhs;
}

template<size_t RowDims, size_t ColDims, typename T>
MatrixTemplate<RowDims, ColDims, T> operator/(MatrixTemplate<RowDims, ColDims, T> lhs, const T& rhs) {
  for (size_t i = 0; i < RowDims; ++i) {
    lhs[i] = lhs[i] / rhs;
  }
  return lhs;
}

template <size_t Dims, typename T>
VectorTemplate<Dims, int> Round(const VectorTemplate<Dims, T>& vector) {
  VectorTemplate<Dims, int> result;
  
  for (size_t i = 0; i < Dims; ++i) {    
      result[i] = (int)std::round(vector[i]);
  }
  return result;
}

template <size_t RowDims, size_t ColDims, typename T>
MatrixTemplate<RowDims, ColDims, int> Round(const MatrixTemplate<RowDims, ColDims, T>& matrix) {
  MatrixTemplate<RowDims, ColDims, int> result;
  for (size_t i = 0; i < RowDims; ++i) {
    result[i] = Round(matrix[i]);
  }
  return result;
}

template <size_t RowDims, size_t ColDims, typename T>
std::ostream& operator<<(std::ostream& out, const MatrixTemplate<RowDims, ColDims, T>& matrix) {
  out << "[";
  for (size_t i = 0; i < RowDims; ++i) {
    out << matrix[i] << std::endl;
  }
  out << "]" << std::endl;

  return out;
}

typedef VectorTemplate<2, float> Vector2f;
typedef VectorTemplate<2, int>   Vector2i;
typedef VectorTemplate<3, float> Vector3f;
typedef VectorTemplate<3, int>   Vector3i;
typedef VectorTemplate<4, float> Vector4f;
typedef MatrixTemplate<4, 4, float> Matrix44f;
typedef MatrixTemplate<4, 3, float> Matrix43f;
typedef MatrixTemplate<3, 3, float> Matrix33f;
typedef MatrixTemplate<2, 3, float> Matrix23f;
typedef MatrixTemplate<4, 3, int> Matrix43i;

Vector3f Barycentric(const Vector2i& A, const Vector2i& B, const Vector2i& C, const Vector2i& P);

#endif //__GEOMETRY_H__
