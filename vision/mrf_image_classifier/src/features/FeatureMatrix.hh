#ifndef __FEAT_MAT_H__
#define __FEAT_MAT_H__

#include <vector>
#include <algorithm>
#include <exception>
#include <ext/hash_set>
#include <ext/hash_map>

#include "util/Dvec.hh"

#include "util/uBLASUtils.hh"

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/operation.hpp>

//using namespace std;

//typedef __gnu_cxx::hash_set<int> intSet;
typedef __gnu_cxx::hash_map<int,int> Int2IntMap;

/*
class EmptyMatrix : public exception {
  virtual const char* what() const throw() {
    return "Empty matrix";
  }
} EmptyMatrixException;
*/

/**
   @brief Abstract base class for matrices used to store features
   @tparam elt The type of the matrix elements
 */
template <class elt>
class FeatureMatrix {
public:

  FeatureMatrix(int nr, int nc) :
    nRows(nr),
    nCols(nc) {
  };

  virtual ~FeatureMatrix() { };

  //  virtual int firstIndex() const = 0;

  int rows() const { return nRows; }
  int cols() const { return nCols; }

  template <class DVEC>
  void addRowToVec(DVEC& vec, double scalar, int row) const;

  void addRowToVec(Dvec& vec, double scalar, int row) const {
    addRowToVec<Dvec>(vec, scalar, row);
  }

  void addRowToVec(DvecView& vec, 
		   double scalar, int row) const {
    addRowToVec<DvecView>(vec, scalar, row);
  }
    
  // right-multiply the feature matrix by a weight vector
  virtual void multiplyWeights(Dvec& wvec, Dvec &vecOut) const = 0;
  virtual void multiplyWeights(DvecViewConst& wvec, 
			       Dvec &vecOut) const = 0;

  // return a new matrix with a subset of rows
  virtual FeatureMatrix* getRowSet(const std::vector<int>& rows) const = 0;

  // should be 0-indexed
  virtual elt get(int row, int col) = 0;
  virtual void set(int row, int col, elt val) = 0;

  //  virtual FeatureMatrix* copy() = 0;

  virtual void finalize() = 0;

  virtual void assertFinite() const = 0;

private:
  int nRows, nCols;
};

// FIXME: set bandwidth in constructor
template <class elt>
class SparseFeatureMatrix : public FeatureMatrix<elt> {
/*
public:
  typedef typename SparseGeMatrix< CRS<elt> >::const_iterator const_iterator;

  SparseFeatureMatrix(int ncols) :
    FeatureMatrix<elt>(0,ncols),
    fmat(NULL),
    finalized(true),
    originalCopy(true)
  {
  }
  
  SparseFeatureMatrix(int nobs, int fdim, 
		      vector<elt> ii, vector<elt> jj, vector<elt> vals) :
    FeatureMatrix<elt>(nobs,fdim), 
    fmat(new SparseGeMatrix< CRS<elt> >(nobs, fdim)),
    finalized(true),
    originalCopy(true)
  {
    for (int it = 0; it < ii.size(); it++) {
      (*fmat)(ii[it] + 1, jj[it] + 1) = vals[it];
    }
    fmat->finalize();
  }

  SparseFeatureMatrix(int nobs, int fdim) :
    FeatureMatrix<elt>(nobs,fdim),
    fmat(new SparseGeMatrix< CRS<elt> >(nobs,fdim)),
    finalized(false),
    originalCopy(true)
  {
  }

  SparseFeatureMatrix<elt>* copy() {
    SparseFeatureMatrix<elt>* newmat = new SparseFeatureMatrix(*this);
    newmat->finalize();
    return newmat;
  }

  SparseFeatureMatrix(const SparseFeatureMatrix& copyme) :
    FeatureMatrix<elt>(copyme.rows(), copyme.cols()),
    fmat(copyme.fmat),
    originalCopy(false)
  {    
  }

  ~SparseFeatureMatrix() {
    if (fmat == NULL && originalCopy == true) delete fmat;
  }

  int firstIndex() const { return 1; }
  
  // FIXME: if null, should probably return empty iterator
  const_iterator begin() const { 
    assert(fmat != NULL);
    return fmat->begin(); 
  }

  const_iterator end() const { 
    assert(fmat != NULL);
    return fmat->end(); 
  }

  // rows are 0-indexed
  SparseFeatureMatrix<elt>* getRowSet(const vector<int>& rows) const {
    if ((fmat == NULL) || (rows.size() == 0)) {
      //      throw "Empty matrix requested!";
      return new SparseFeatureMatrix(this->cols());
    }
    
    SparseFeatureMatrix<elt>* newMat = 
      new SparseFeatureMatrix(rows.size(), this->cols());

    // make a map from old row indices to new row indices
    Int2IntMap old2NewRows;
    int newRow = 1;
    for (vector<int>::const_iterator it = rows.begin(); it != rows.end(); it++) {
      old2NewRows[*it + 1] = newRow;
      newRow++;
    }

    // iterate over sparse matrix, copying elements in row set
    for (const_iterator it = begin(); it != end(); ++it) {
      pair<pair<int,int>, elt> keyval = *it;
      pair<int,int> rowcol = keyval.first;

      // check to see if this row is in the set
      if (old2NewRows.find(rowcol.first) == old2NewRows.end())
	continue;

      (*newMat->fmat)(old2NewRows[rowcol.first], rowcol.second) = keyval.second;
    }

    newMat->finalize();

    return newMat;
  }

  void multiplyWeights(Dvec& wvec, Dvec& vecOut) const {
    if (fmat == NULL) return;
    vecOut = (*fmat) * wvec;
  }

  void multiplyWeights(const Dvec::ConstView& wvec, Dvec& vecOut) const {
    if (fmat == NULL) return;
    vecOut = (*fmat) * wvec;
  }

  void finalize() { 
    if (fmat == NULL || finalized == true) return;
    fmat->finalize(); finalized = true; 
  }
  
  Dvec* getRow(int row) const {
    Dvec *vec = new Dvec0(this->getCols());
    for (int col = 1; col <= this->getCols(); col++)
      (*vec)(col) = fmat(row,col);
    return vec;
  };

  Dvec* getCol(int col) const {
    Dvec *vec = new Dvec0(this->getRows());
    for (int row = 1; row <= this->getRows(); row++)
      (*vec)(col) = fmat(row,col);
    return vec;
  };

  // row is 0-indexed
  template <class DVEC>
  void addRowToVec(DVEC& vec, double scalar, int row) const {
    int ind0 = vec.firstIndex();
    int cols = this->cols();
    for (int col = 0; col < cols; col++) 
      vec(col+ind0) += scalar * (*fmat)(row + 1, col + 1);
  }

  void addRowToVec(Dvec& vec, double scalar, int row) const {
    addRowToVec<Dvec>(vec, scalar, row);
  }

  void addRowToVec(Dvec::View& vec, double scalar, int row) const {
    addRowToVec<Dvec::View>(vec, scalar, row);
  }

  //  SparseGeMatrix< CRS<elt> > *getMat() { return &fmat; }
  
  elt get(int row, int col) { 
    assert(finalized == true);
    assert(fmat != NULL);
    return (*fmat)(row+1, col+1); 
  }

  void set(int row, int col, elt val) { 
    assert(finalized == false);
    assert(fmat != NULL);

    if (!originalCopy) {
      fmat = new SparseGeMatrix< CRS<elt> >(*fmat);
      originalCopy = true;
    }

    (*fmat)(row+1, col+1) = val; 
  }

  int nnz() { return fmat->numNonZeros(); }

private:
  SparseGeMatrix< CRS<elt> >* fmat;
  bool finalized;
  bool originalCopy;
*/
};
  
// 0-indexed matrix with dense storage
template <class elt> 
class DenseFeatureMatrix : public FeatureMatrix<elt> {
public:
  // Creates a 0 x fdim matrix
  DenseFeatureMatrix(int fdim) :
    FeatureMatrix<elt>(0, fdim),
    fmat(NULL),
    originalCopy(true)
  {
  };

  // Creates a nobs x fdim matrix
  DenseFeatureMatrix(int nobs, int fdim) :
    FeatureMatrix<elt>(nobs, fdim),
    fmat(new boost::numeric::ublas::matrix<double>(nobs,fdim)),
    originalCopy(true)
  {
    fmat->clear();
    //    fmat(_(0,nobs-1), _(0,fdim-1)) {
  };
  
  /*
  // makes a shallow copy (deep copy made on write)
  DenseFeatureMatrix(const SparseFeatureMatrix<elt>& smat) :
    FeatureMatrix<elt>(smat.rows(), smat.cols()),
    fmat(new GeMatrix<FullStorage<elt, ColMajor> > 
	 (smat.rows(), smat.cols(), 0, 0)),
    originalCopy(true)
  {
    typename SparseFeatureMatrix<elt>::const_iterator itEnd = smat.end();
    for (typename SparseFeatureMatrix<elt>::const_iterator it = smat.begin();
	 it != itEnd;
	 ++it) {
      pair<pair<int,int>, elt> keyval = *it;
      pair<int,int> rowcol = keyval.first;
      (*fmat)(rowcol.first - smat.firstIndex(), 
	      rowcol.second - smat.firstIndex()) = keyval.second;	
    }
  }
  */
  
  // make shallow copies, then deep copy on write
  DenseFeatureMatrix(const DenseFeatureMatrix& copyme) :
    FeatureMatrix<elt>(copyme.rows(), copyme.cols()),
    fmat(copyme.fmat),
    originalCopy(false)
  {    
  }
	 
  ~DenseFeatureMatrix() {
    if (fmat != NULL && originalCopy == true) delete fmat;
  }

  DenseFeatureMatrix<elt>* copy() const {
    DenseFeatureMatrix<elt>* fcopy = new DenseFeatureMatrix(*this);
    return fcopy;
  }

  int firstIndex() const { return 0; }

  // vecOut MUST NOT share memory with wvec
  void multiplyWeights(Dvec& wvec, Dvec& vecOut) const {
    if (fmat == NULL || this->rows() == 0)
      return;

    //    vecOut = (*fmat) * wvec;
    noalias(vecOut) = prod(*fmat, wvec);
  }

  void multiplyWeights(DvecViewConst& wvec, Dvec& vecOut) const {
    if (fmat == NULL) return;

    //    vecOut = (*fmat) * wvec;
    using namespace boost::numeric;

    noalias(vecOut) = prod(*fmat, wvec);
  }

  DenseFeatureMatrix<elt>* getRowSet(const std::vector<int>& rows) const {
    if (fmat == NULL || rows.size() == 0) {
      //      throw "Empty matrix requested!";
      return new DenseFeatureMatrix(this->cols());
    }
    
    DenseFeatureMatrix<elt>* newMat = 
      new DenseFeatureMatrix(rows.size(), this->cols());

    boost::numeric::ublas::matrix<double>* newfmat = newMat->fmat;

    int newRow = 0;
    for (std::vector<int>::const_iterator it = rows.begin(); 
	 it != rows.end();
	 it++) {
      int oldRow = *it;
      //      (*newfmat)(newRow,_) = (*fmat)(oldRow,_);
      row(*newfmat, newRow) = row(*fmat, oldRow);
      newRow++;
    }

    return newMat;
  }

  template <class DVEC>
  void addRowToVec(DVEC& vec, double scalar, int row) const {
    assert(fmat != NULL);

    //    vec += scalar * (*fmat)(row,_);
    vec += scalar * boost::numeric::ublas::row(*fmat, row);
  }

  elt get(int row, int col) {
    assert(fmat != NULL);
    return (*fmat)(row, col);
  }

  void set(int row, int col, elt val) {
    assert(fmat != NULL);

    if (!originalCopy) {
      fmat = new boost::numeric::ublas::matrix<double>(*fmat);
      originalCopy = true;
    }

    (*fmat)(row, col) = val;
  }

  void assertFinite() const {
    if (fmat == NULL) return;

    for (int ii = 0; ii < (int)fmat->size1(); ii++) {
      for (int jj = 0; jj < (int)fmat->size2(); jj++) {
	assert(finite((*fmat)(ii, jj)));
      }
    }
  }


  // stub for compatibility with sparse matrices
  void finalize() { return; }

  //  GeMatrix<FullStorage<elt, ColMajor> >* getMat() { return fmat; }

  /*
  void setRow(int row, vector<elt> data) {
    assert(fmat != NULL);

    typename vector<elt>::iterator dend = data.end();
    int col = 0;
    for (typename vector<elt>::iterator dit = data.begin();
	 dit != dend; 
	 dit++) {
      (*fmat)(row, col) = *dit;
      col++;
    }
  }

  Dvec *getRow(int row) const {
    if (fmat == NULL) return NULL;

    // this syntax necessary because of template weirdness
    Dvec *vec = new Dvec0(this->getCols()); 
    *vec = (*fmat)(row,_);
    return vec;
  }

  Dvec *getCol(int col) const {
    if (fmat == NULL) return NULL;

    Dvec *vec = new Dvec0(this->getRows());
    *vec = (*fmat)(col,_);
    return vec;
  }
    
  // FIXME: is this 1-indexed?  does it matter?
  Dvec::View* getRowView(int row) {
    if (fmat == NULL) return NULL;

    return new Dvec::View((*fmat)(row,_));
  }

  Dvec::View* getColView(int col) {
    if (fmat == NULL) return NULL;

    return new Dvec::View((*fmat)(_,col));
  }

  // FIXME: is this 1-indexed?  does it matter?
  Dvec::ConstView* getRowViewConst(int row) const {
    if (fmat == NULL) return NULL;

    return new Dvec::ConstView((*fmat)(row,_));
  }

  Dvec::ConstView* getColViewConst(int col) const {
    if (fmat == NULL) return NULL;

    return new Dvec::ConstView((*fmat)(_,col));
  }
  */

  friend std::ostream& operator<<(std::ostream& output, 
				  const DenseFeatureMatrix<double>& fmat);
    
private:
  boost::numeric::ublas::matrix<double> *fmat;
  bool originalCopy; 
};

std::ostream& operator<<(std::ostream& sout, 
			 const DenseFeatureMatrix<double>& fmat);

#endif
