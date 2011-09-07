#ifndef __NEW_MAT_VECTOR_H__
#define __NEW_MAT_VECTOR_H__

/// Row vector.
class RowVector : public Matrix
{
   GeneralMatrix* Image() const;                // copy of matrix
public:
   RowVector() { nrows_val = 1; }
   ~RowVector() {}
   RowVector(ArrayLengthSpecifier n) : Matrix(1,n.Value()) {}
   RowVector(const BaseMatrix&);
   RowVector(const RowVector& gm) : Matrix() { GetMatrix(&gm); }
   void operator=(const BaseMatrix&);
   void operator=(Real f) { GeneralMatrix::operator=(f); }
   void operator=(const RowVector& m) { Eq(m); }
   Real& operator()(int);                       // access element
   Real& element(int);                          // access element
   Real operator()(int) const;                  // access element
   Real element(int) const;                     // access element
#ifdef SETUP_C_SUBSCRIPTS
   Real& operator[](int m) { return store[m]; }
   const Real& operator[](int m) const { return store[m]; }
   // following for Numerical Recipes in C++
   RowVector(Real a, int n) : Matrix(a, 1, n) {}
   RowVector(const Real* a, int n) : Matrix(a, 1, n) {}
#endif
   MatrixType type() const;
   void GetCol(MatrixRowCol&);
   void GetCol(MatrixColX&);
   void NextCol(MatrixRowCol&);
   void NextCol(MatrixColX&);
   void RestoreCol(MatrixRowCol&) {}
   void RestoreCol(MatrixColX& c);
   GeneralMatrix* Transpose(TransposedMatrix*, MatrixType);
   void resize(int);                       // change dimensions
   void ReSize(int m) { resize(m); }
   void resize_keep(int);
   void resize_keep(int,int);
   void resize(int,int);                   // in case access is matrix
   void ReSize(int m,int n) { resize(m, n); }
   void resize(const GeneralMatrix& A);
   void ReSize(const GeneralMatrix& A) { resize(A); }
   Real* nric() const
      { CheckStore(); return store-1; }         // for use by NRIC
   void cleanup();                              // to clear store
   void MiniCleanUp()
      { store = 0; storage = 0; nrows_val = 1; ncols_val = 0; tag_val = -1; }
   // friend ReturnMatrix GetMatrixRow(Matrix& A, int row);
   void operator+=(const Matrix& M) { PlusEqual(M); }
   void operator-=(const Matrix& M) { MinusEqual(M); }
   void operator+=(Real f) { GeneralMatrix::Add(f); }
   void operator-=(Real f) { GeneralMatrix::Add(-f); }
   void swap(RowVector& gm)
      { GeneralMatrix::swap((GeneralMatrix&)gm); }
   NEW_DELETE(RowVector)
};

/// Column vector.
class ColumnVector : public Matrix
{
   GeneralMatrix* Image() const;                // copy of matrix
public:
   ColumnVector() { ncols_val = 1; }
   ~ColumnVector() {}
   ColumnVector(ArrayLengthSpecifier n) : Matrix(n.Value(),1) {}
   ColumnVector(const BaseMatrix&);
   ColumnVector(const ColumnVector& gm) : Matrix() { GetMatrix(&gm); }
   void operator=(const BaseMatrix&);
   void operator=(Real f) { GeneralMatrix::operator=(f); }
   void operator=(const ColumnVector& m) { Eq(m); }
   Real& operator()(int);                       // access element
   Real& element(int);                          // access element
   Real operator()(int) const;                  // access element
   Real element(int) const;                     // access element
#ifdef SETUP_C_SUBSCRIPTS
   Real& operator[](int m) { return store[m]; }
   const Real& operator[](int m) const { return store[m]; }
   // following for Numerical Recipes in C++
   ColumnVector(Real a, int m) : Matrix(a, m, 1) {}
   ColumnVector(const Real* a, int m) : Matrix(a, m, 1) {}
#endif
   MatrixType type() const;
   GeneralMatrix* Transpose(TransposedMatrix*, MatrixType);
   void resize(int);                       // change dimensions
   void ReSize(int m) { resize(m); }
   void resize_keep(int);
   void resize_keep(int,int);
   void resize(int,int);                   // in case access is matrix
   void ReSize(int m,int n) { resize(m, n); }
   void resize(const GeneralMatrix& A);
   void ReSize(const GeneralMatrix& A) { resize(A); }
   Real* nric() const
      { CheckStore(); return store-1; }         // for use by NRIC
   void cleanup();                              // to clear store
   void MiniCleanUp()
      { store = 0; storage = 0; nrows_val = 0; ncols_val = 1; tag_val = -1; }
//   ReturnMatrix Reverse() const;                // reverse order of elements
   void operator+=(const Matrix& M) { PlusEqual(M); }
   void operator-=(const Matrix& M) { MinusEqual(M); }
   void operator+=(Real f) { GeneralMatrix::Add(f); }
   void operator-=(Real f) { GeneralMatrix::Add(-f); }
   void swap(ColumnVector& gm)
      { GeneralMatrix::swap((GeneralMatrix&)gm); }
   NEW_DELETE(ColumnVector)
};

#endif

