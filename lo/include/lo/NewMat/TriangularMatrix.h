#ifndef __NEW_MAT_TRIANGULAR_MATRIX_H__
#define __NEW_MAT_TRIANGULAR_MATRIX_H__

/// Upper triangular matrix.
class UpperTriangularMatrix : public GeneralMatrix
{
   GeneralMatrix* Image() const;                // copy of matrix
public:
   UpperTriangularMatrix() {}
   ~UpperTriangularMatrix() {}
   UpperTriangularMatrix(ArrayLengthSpecifier);
   void operator=(const BaseMatrix&);
   void operator=(const UpperTriangularMatrix& m) { Eq(m); }
   UpperTriangularMatrix(const BaseMatrix&);
   UpperTriangularMatrix(const UpperTriangularMatrix& gm)
      : GeneralMatrix() { GetMatrix(&gm); }
   void operator=(Real f) { GeneralMatrix::operator=(f); }
   Real& operator()(int, int);                  // access element
   Real& element(int, int);                     // access element
   Real operator()(int, int) const;             // access element
   Real element(int, int) const;                // access element
#ifdef SETUP_C_SUBSCRIPTS
   Real* operator[](int m) { return store+m*ncols_val-(m*(m+1))/2; }
   const Real* operator[](int m) const
      { return store+m*ncols_val-(m*(m+1))/2; }
#endif
   MatrixType type() const;
   GeneralMatrix* MakeSolver() { return this; } // for solving
   void Solver(MatrixColX&, const MatrixColX&);
   LogAndSign log_determinant() const;
   Real trace() const;
   void GetRow(MatrixRowCol&);
   void GetCol(MatrixRowCol&);
   void GetCol(MatrixColX&);
   void RestoreCol(MatrixRowCol&);
   void RestoreCol(MatrixColX& c) { RestoreCol((MatrixRowCol&)c); }
   void NextRow(MatrixRowCol&);
   void resize(int);                       // change dimensions
   void ReSize(int m) { resize(m); }
   void resize(const GeneralMatrix& A);
   void ReSize(const GeneralMatrix& A) { resize(A); }
   void resize_keep(int);
   MatrixBandWidth bandwidth() const;
   void operator+=(const UpperTriangularMatrix& M) { PlusEqual(M); }
   void operator-=(const UpperTriangularMatrix& M) { MinusEqual(M); }
   void operator+=(Real f) { GeneralMatrix::operator+=(f); }
   void operator-=(Real f) { GeneralMatrix::operator-=(f); }
   void swap(UpperTriangularMatrix& gm)
      { GeneralMatrix::swap((GeneralMatrix&)gm); }
   NEW_DELETE(UpperTriangularMatrix)
};

/// Lower triangular matrix.
class LowerTriangularMatrix : public GeneralMatrix
{
   GeneralMatrix* Image() const;                // copy of matrix
public:
   LowerTriangularMatrix() {}
   ~LowerTriangularMatrix() {}
   LowerTriangularMatrix(ArrayLengthSpecifier);
   LowerTriangularMatrix(const LowerTriangularMatrix& gm)
      : GeneralMatrix() { GetMatrix(&gm); }
   LowerTriangularMatrix(const BaseMatrix& M);
   void operator=(const BaseMatrix&);
   void operator=(Real f) { GeneralMatrix::operator=(f); }
   void operator=(const LowerTriangularMatrix& m) { Eq(m); }
   Real& operator()(int, int);                  // access element
   Real& element(int, int);                     // access element
   Real operator()(int, int) const;             // access element
   Real element(int, int) const;                // access element
#ifdef SETUP_C_SUBSCRIPTS
   Real* operator[](int m) { return store+(m*(m+1))/2; }
   const Real* operator[](int m) const { return store+(m*(m+1))/2; }
#endif
   MatrixType type() const;
   GeneralMatrix* MakeSolver() { return this; } // for solving
   void Solver(MatrixColX&, const MatrixColX&);
   LogAndSign log_determinant() const;
   Real trace() const;
   void GetRow(MatrixRowCol&);
   void GetCol(MatrixRowCol&);
   void GetCol(MatrixColX&);
   void RestoreCol(MatrixRowCol&);
   void RestoreCol(MatrixColX& c) { RestoreCol((MatrixRowCol&)c); }
   void NextRow(MatrixRowCol&);
   void resize(int);                       // change dimensions
   void ReSize(int m) { resize(m); }
   void resize_keep(int);
   void resize(const GeneralMatrix& A);
   void ReSize(const GeneralMatrix& A) { resize(A); }
   MatrixBandWidth bandwidth() const;
   void operator+=(const LowerTriangularMatrix& M) { PlusEqual(M); }
   void operator-=(const LowerTriangularMatrix& M) { MinusEqual(M); }
   void operator+=(Real f) { GeneralMatrix::operator+=(f); }
   void operator-=(Real f) { GeneralMatrix::operator-=(f); }
   void swap(LowerTriangularMatrix& gm)
      { GeneralMatrix::swap((GeneralMatrix&)gm); }
   NEW_DELETE(LowerTriangularMatrix)
};

#endif

