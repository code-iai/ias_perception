#ifndef __NEW_MAT_MATRIX_H__
#define __NEW_MAT_MATRIX_H__


/// The usual rectangular matrix.
class Matrix : public GeneralMatrix
{
   GeneralMatrix* Image() const;                // copy of matrix
public:
   Matrix() {}
   ~Matrix() {}
   Matrix(int, int);                            // standard declaration
   Matrix(const BaseMatrix&);                   // evaluate BaseMatrix
   void operator=(const BaseMatrix&);
   void operator=(Real f) { GeneralMatrix::operator=(f); }
   void operator=(const Matrix& m) { Eq(m); }
   MatrixType type() const;
   Real& operator()(int, int);                  // access element
   Real& element(int, int);                     // access element
   Real operator()(int, int) const;             // access element
   Real element(int, int) const;                // access element
#ifdef SETUP_C_SUBSCRIPTS
   Real* operator[](int m) { return store+m*ncols_val; }
   const Real* operator[](int m) const { return store+m*ncols_val; }
   // following for Numerical Recipes in C++
   Matrix(Real, int, int);
   Matrix(const Real*, int, int);
#endif
   Matrix(const Matrix& gm) : GeneralMatrix() { GetMatrix(&gm); }
   GeneralMatrix* MakeSolver();
   Real trace() const;
   void GetRow(MatrixRowCol&);
   void GetCol(MatrixRowCol&);
   void GetCol(MatrixColX&);
   void RestoreCol(MatrixRowCol&);
   void RestoreCol(MatrixColX&);
   void NextRow(MatrixRowCol&);
   void NextCol(MatrixRowCol&);
   void NextCol(MatrixColX&);
   virtual void resize(int,int);           // change dimensions
      // virtual so we will catch it being used in a vector called as a matrix
   virtual void resize_keep(int,int);
   virtual void ReSize(int m, int n) { resize(m, n); }
   void resize(const GeneralMatrix& A);
   void ReSize(const GeneralMatrix& A) { resize(A); }
   Real maximum_absolute_value2(int& i, int& j) const;
   Real minimum_absolute_value2(int& i, int& j) const;
   Real maximum2(int& i, int& j) const;
   Real minimum2(int& i, int& j) const;
   void operator+=(const Matrix& M) { PlusEqual(M); }
   void operator-=(const Matrix& M) { MinusEqual(M); }
   void operator+=(Real f) { GeneralMatrix::Add(f); }
   void operator-=(Real f) { GeneralMatrix::Add(-f); }
   void swap(Matrix& gm) { GeneralMatrix::swap((GeneralMatrix&)gm); }
   friend Real dotproduct(const Matrix& A, const Matrix& B);
   NEW_DELETE(Matrix)
};

#endif

