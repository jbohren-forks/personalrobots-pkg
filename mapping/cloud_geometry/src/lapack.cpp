/*
 * Copyright (c) 2008 Radu Bogdan Rusu <rusu -=- cs.tum.edu>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

/** \author Radu Bogdan Rusu */

#include "cloud_geometry/lapack.h"

namespace cloud_geometry
{
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Compute the 3 eigen values and eigenvectors for a 3x3 covariance matrix
    * \param covariance_matrix a 3x3 covariance matrix in eigen2::matrix3d format
    * \param eigen_values the resulted eigenvalues in eigen2::vector3d
    * \param eigen_vectors a 3x3 matrix in eigen2::matrix3d format, containing each eigenvector on a new line
    */
  bool
    eigen_cov (Eigen::Matrix3d covariance_matrix, Eigen::Vector3d &eigen_values, Eigen::Matrix3d &eigen_vectors)
  {
    char jobz = 'V';    // 'V':  Compute eigenvalues and eigenvectors
    char uplo = 'U';    // 'U':  Upper triangle of A is stored

    int n = 3, lda = 3, info = -1;
    int lwork = 3 * 3 - 1;

    double work[lwork];
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        eigen_vectors (i, j) = covariance_matrix (i, j);

    dsyev_ (&jobz, &uplo, &n, eigen_vectors.data (), &lda, eigen_values.data (), work, &lwork, &info);

    return (info == 0);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Perform Singular Value Decomposition (SVD) on a given matrix. Returns U, S, V.
    * \param h a 3x3 matrix in eigen2::matrix3d format that is to be decomposed
    * \param u the resulted U matrix
    * \param s the resulted S diagonal matrix as a eigen2::vector3d
    * \param v the resulted V matrix
    */
  bool
    svd (Eigen::Matrix3d h, Eigen::Matrix3d &u, Eigen::Vector3d &s, Eigen::Matrix3d &v)
  {
    char jobu  = 'A';    // 'A':  all M columns of U are returned in array U
    char jobvt = 'A';    // 'A':  all N rows of V**T are returned in the array VT

    int m = 3, n = 3, lda = 3, ldu = 3, ldvt = 3, info = -1;
    int lwork = 5 * 3;

    double work[lwork];
    Eigen::Matrix3d B;
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        B (i, j) = h (j, i);

    dgesvd_ (&jobu, &jobvt, &m, &n, B.data (), &lda, s.data (), u.data (), &ldu, v.data (), &ldvt, work, &lwork, &info);

//    U = transpose (U, m, m);
    return (info == 0);
  }

}

