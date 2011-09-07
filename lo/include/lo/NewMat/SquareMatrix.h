#ifndef __NEW_MAT_SQUARE_MATRIX_H__
#define __NEW_MAT_SQUARE_MATRIX_H__

/// Square matrix.
class SquareMatrix : public Matrix
{
   GeneralMatrix* Image() const;                // copy of matrix
public:
   SquareMatrix() {}
   ~SquareMatrix() {}
   SquareMatrix(ArrayLengthSpecifier);          // standard declaration
   SquareMatrix(const BaseMatrix&);             // evaluate BaseMatrix
   void operator=(const BaseMatrix&);
   void operator=(Real f) { GeneralMatrix::operator=(f); }
   void operator=(const SquareMatrix& m) { Eq(m); }
   void operator=(const Matrix& m);
   MatrixType type() const;
   SquareMatrix(const SquareMatrix& gm) : Matrix() { GetMatrix(&gm); }
   SquareMatrix(const Matrix& gm);
   void resize(int);                            // change dimensions
   void ReSize(int m) { resize(m); }
   void resize_keep(int);
   void resize_keep(int,int);
   void resize(int,int);                        // change dimensions
   void ReSize(int m, int n) { resize(m, n); }
   void resize(const GeneralMatrix& A);
   void ReSize(const GeneralMatrix& A) { resize(A); }
   void operator+=(const Matrix& M) { PlusEqual(M); }
   void operator-=(const Matrix& M) { MinusEqual(M); }
   void operator+=(Real f) { GeneralMatrix::Add(f); }
   void operator-=(Real f) { GeneralMatrix::Add(-f); }
   void swap(SquareMatrix& gm) { GeneralMatrix::swap((GeneralMatrix&)gm); }
   NEW_DELETE(SquareMatrix)
};

#endif

