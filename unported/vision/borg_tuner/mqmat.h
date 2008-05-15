#ifndef MQMAT_H
#define MQMAT_H

#include <cstring>
#include <cassert>
#include <cstdio>
#include <iostream>
#include <iomanip>

namespace mqmath
{

template<size_t rows, size_t cols, typename T>
class mqmat;

template<size_t rows, size_t cols, typename T> 
std::ostream &operator<<(std::ostream &, const mqmat<rows, cols, T> &);

template<size_t rows, size_t cols = 1, typename T = double>
class mqmat
{
  friend std::ostream &operator<< <rows, cols, T>
    (std::ostream &, const mqmat<rows, cols, T> &);
public:
  T *d;
  int pop_row, pop_col; // used for populating with <<
  mqmat() : pop_row(0), pop_col(0)
  {
    d = new T[rows * cols];
    memset(d, 0, rows * cols * sizeof(T));
  }
  mqmat(const mqmat &copy) : 
    pop_row(copy.pop_row),
    pop_col(copy.pop_col)
  {
    printf("copy constructor, %d by %d\n", rows, cols);
    d = new T[rows * cols];
    memcpy(d, copy.d, rows * cols * sizeof(T));
  }

  ~mqmat()
  {
    delete[] d;
  }

  inline const size_t num_rows() const { return rows; }
  inline const size_t num_cols() const { return cols; }
  void populate(int i, int j) { pop_row = i; pop_col = j; }
  T at(int i, int j) const { return d[i*cols + j]; }

  mqmat &operator |(T val)
  {
    if (pop_row < (int)rows && pop_col < (int)cols)
    {
      d[pop_row * cols + pop_col] = val;
      if (++pop_col >= cols)
      {
        pop_col = 0;
        pop_row++;
      }
    }
    return *this;
  }

  template<size_t rrows, size_t rcols, typename rT>
  mqmat<rows, rcols, T> operator *(const mqmat<rrows, rcols, rT> &rhs)
  {
    mqmat<rows, rcols, T> temp;
    for (int i = 0; i < rows; i++)
      for (int j = 0; j < rcols; j++)
        for (int k = 0; k < cols; k++)
          temp.d[i*rcols+j] += d[i*cols+k] * rhs.d[k*rcols+j];
    return temp;
  }
};

template<size_t rows, size_t cols, typename T> 
std::ostream &operator<<(std::ostream &ostr, const mqmat<rows, cols, T> &mat)
{
  ostr << std::setprecision(4);
  for (int i = 0; i < rows; i++)
  {
    for (int j = 0; j < cols; j++)
    {
      ostr << std::setw(11) << std::left << mat.d[i*cols +j] << " ";
    }
    ostr << "\n";
  }
  return ostr;
}

}

#endif

