// This file is part of LeebA, a lightweight C++ template library
// for convex optimization and control.
//
// Copyright (C) 2008 Timothee Hunter <tjhunter@stanford.edu>
//
// LeebA is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// LeebA is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License  for 
// more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with LeebA in the file COPYING.LESSER at the root of the 
// LeebA package. If not, see <http://www.gnu.org/licenses/>.

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/QR>
#include <Eigen/SVD>
#include <Eigen/Cholesky>
#include <vector>
#include <boost/mpl/assert.hpp>

/** @author Timothy Hunter tjhunter@willowgarage.com
  * @brief LQR algorithmic solver for steady state control in discrete and continuous systems
*/



namespace LQR
{

/** @brief templated structure around the lqr algorithm
  * @param AType Type of the state matrix
  * @param BType Type of the input matrix
  * @param QType Type of the state cost matrix
  * @param RType Type of the input cost matrix
  */
template<typename AType, typename BType, typename QType, typename RType>
struct LQRDP
{
  enum
  {
    NumStates = AType::ColsAtCompileTime,
    NumCommands = BType::ColsAtCompileTime
  };
  
  // The scalar type used (double, float, complex, int if you really want)
  typedef typename AType::Scalar Scalar;
  
  // The type of the gains matrix
  typedef Eigen::Matrix<Scalar, NumCommands,NumStates> KType;

  //NOTE: this code is still not very mature and can be optimized:
  //FIXME: move computation of K out of the loop
  //FIXME: use a better inverter (Cholesky decomposition)
  //FIXME: factorize inversion and various pieces that get transposed around
  /** @brief Given a system dynamics (A,B) of a discrete system, a final cost Qf, and trajectory gains Q and R (for the state and the command, respectively), this function computes a new gains matrix
    * This function computes iteratively the solution of the steady-state regulator.
    * We want to solve the problem in the limit:
    * Minimize J(U)
    * Subject to: x_n+1 = A x_n + B u_n for all n
    * With J(U) = sum((x_t)^T Q x_t + (u_t)^T R u_t) + x(N)^T Qf x(N)
    * An equivalent problem is the above problem with the additional condition u(t) = K x(t) where the matrix K becomes the new unknown.
    * In the limit, the optimal gains matrix verifies the Algebraic Riccati Equation:
    *   P_ss = Q + A^T P_ss A - A^T P_ss B ( R + B^T P_ss B ) ^-1 B^T P_ss A
    *   K_ss = -(R + B^T P_ss B)^-1 B^T P_ss A
    * The algorithm used uses this equation to compute a converging sequence of P_ss until the precision threshold is reached.
    * @note This algorithm assumes the pair (A,B) is controllable and the pair (A,Q) is observable.
    * @param A 
    * @param B
    * @param Q the state cost matrix. Assumed to be positive, semi-definite
    * @param Qf the final state cost matrix. Assumed to be positive, semi-definite
    * @param R the inputs cost matrix
    * @param precision 
    * @param K 
    */
  static void run(const AType & A,
    const BType & B,
    const QType & Q,
    const QType & Qf,
    const RType & R,
    Scalar precision,
    KType & K)
  {
    const int n=A.rows();
    QType P = Qf;
    Scalar error=Scalar(1000);
    int steps=0;
    while(error>precision*precision*n*n)
    {
      steps++;
      const QType & nextP = Q+A.transpose()*P*A
        -A.transpose()*P*B*(R+B.transpose()*P*B).inverse()*B.transpose()*P*A;
      error = (P-nextP).lazy().cwise().square().sum();
      
      K = -(R+B.transpose()*P*B).inverse()*B.transpose()*P*A;
      P = nextP;
    }
  }
  
  // Continuous version
  // This version is the one mostly used in practice and is optimized for speed and stability
  // TODO: evaluate Cholesky inversion for R
  /** @brief Solves the steady-state continuous LQR problem
    * Given a linear system in a continuous space:
    *    xdot=Ax+Bu
    * We want to solve an approximation of the infinite horizon problem:
    * minimize J(U)
    * subject to xdot = Ax+Bu, x(0)=x_0
    * with:
    *   \f$ J(U)=\int_{0}^{T}x(s)^{T}Qx(s)+u(s)^{T}Ru(s)ds+x(T)^{T}Q_{f}x(T) \f$
    * It can be shown that the optimal control is again linear to the state. In the case of the steady-state regulator, it can be computed by solving the algebraic Riccati equation:
    *  \f$ A^{T}P_{ss}+P_{ss}PBR^{-1}B^{T}P+Q=0 \f$
    * Then \f$ u(t)=K_{ss}x(t) \f$
    *    \f$ K_{ss}=-R^{-1}B^{T}P_{ss}\f$
    * @note The current implementation relies on the discrete solver by solving a discretized version of the problem. The parameter h is used to control the discretization step. Parameters below 0.2 are prone to fail (i.e. the computed gains matrix is not Hurwitz). In any case, one should always check if the computed gains matrix is Hurwitz by using the isConverging function from the util functions. If it did not converge, try a bigger step.
    *
    */
  static void runContinuous(const AType & A,
    const BType & B,
    const QType & Q,
    const RType & R,
    KType & K)
  {

    // The algorithm is straightforward: we create the Hamiltonian matrix,
    // we identify the n eigen vectors corresponding to the stable eigen values
    // from which we extract P, unique PSD solution of the ARE
    // All of this is easily done using Eigen

    // We statically determine the size of the Hamiltonian matrix
    static const int N=AType::RowsAtCompileTime;
    static const int N2 = (N==Eigen::Dynamic?Eigen::Dynamic:2*N);
    typedef typename AType::Scalar Scalar;
    typedef Eigen::Matrix<Scalar,N2,N2> Hamiltonian;
    // The interesting eigen values
    typedef Eigen::Matrix<Scalar,N2,N> SemiHam;
    // We need to evaluate the transpose of B, so we create a separate type for it.
    typedef Eigen::Matrix<Scalar,BType::ColsAtCompileTime,BType::RowsAtCompileTime> BTType;
    const int n=A.rows();
    
    const BTType & Bt = B.transpose();
    // Computes R^-1 B' (used in 2 places)
    //Using SVD inversion for best stability.
    BTType RinvBt(Bt.rows(),Bt.cols());
    Eigen::SVD<RType> svd(R);
    svd.solve(Bt,&RinvBt);

    // Creates the Hamiltonian
    // 2 lines...
    Hamiltonian H(2*n,2*n);
    H<<A,-B*RinvBt,
      -Q,-A.transpose();

    Eigen::EigenSolver<Hamiltonian> es(H);
    const Hamiltonian & realEigenVecs = es.pseudoEigenvectors();
    const Hamiltonian & realEigenVals = es.pseudoEigenvalueMatrix();

    //Now we have to work a bit to isolate n real eigenvectors corresponding to the n negative eigen values
    SemiHam S(2*n,n);
    int j=0;
    for(int i=0;i<2*n;++i)
      if(realEigenVals(i,i)<0)
      {
        S.col(j)=realEigenVecs.col(i);
        j++;
      }

    const AType &X=S.corner(Eigen::TopLeft,n,n);
    const AType &Y=S.corner(Eigen::BottomLeft,n,n);
    
    // Inverting X by LU decomposition
    const AType &P=Y*X.inverse();

    K=-RinvBt*P;
    
#if DEBUG
    // Prints all the steps to easily debug in Matlab/Octave
    
    // Matlab-friendly format
    Eigen::IOFormat fmt(8, Eigen::AlignCols, ", ", ";\n", "", "", "[", "]");
    cout<<"A = "<<A.format(fmt)<<endl;
    cout<<"B = "<<B.format(fmt)<<endl;
    cout<<"Q = "<<Q.format(fmt)<<endl;
    cout<<"R = "<<R.format(fmt)<<endl;
    
    cout<<"RinvBt = "<<RinvBt.format(fmt)<<endl;
    
    cout<<"H = "<<H.format(fmt)<<endl;
    
    cout<<"REVal = "<<es.pseudoEigenvalueMatrix().format(fmt)<<"\n";
    cout<<"REVec = "<<es.pseudoEigenvectors().format(fmt)<<"\n";
    
    cout<<"EVDiff = "<<(H*realEigenVecs-realEigenVecs*realEigenVals).eval().format(fmt)<<endl;
    
    cout<<"S = "<<S.format(fmt)<<endl;
    
    cout<<"P = "<<P.format(fmt)<<endl;
    
    cout<<"K = "<<K.format(fmt)<<endl;
#endif
  }

};

} //namespace