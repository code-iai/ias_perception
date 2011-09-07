#ifndef __NEW_MAT_NRIC_MATRIX_H__
#define __NEW_MAT_NRIC_MATRIX_H__

/// Rectangular matrix for use with Numerical Recipes in C.
class nricMatrix : public Matrix
{
   GeneralMatrix* Image() const;                // copy of matrix
   Real** row_pointer;                          // points to rows
   void MakeRowPointer();                       // build rowpointer
   void DeleteRowPointer();
public:
   nricMatrix() {}
   nricMatrix(int m, int n)                     // standard declaration
      :  Matrix(m,n) { MakeRowPointer(); }
   nricMatrix(const BaseMatrix& bm)             // evaluate BaseMatrix
      :  Matrix(bm) { MakeRowPointer(); }
   void operator=(const BaseMatrix& bm)
      { DeleteRowPointer(); Matrix::operator=(bm); MakeRowPointer(); }
   void operator=(Real f) { GeneralMatrix::operator=(f); }
   void operator=(const nricMatrix& m)
      { DeleteRowPointer(); Eq(m); MakeRowPointer(); }
   void operator<<(const BaseMatrix& X)
      { DeleteRowPointer(); Eq(X,this->type(),true); MakeRowPointer(); }
   nricMatrix(const nricMatrix& gm) : Matrix()
      { GetMatrix(&gm); MakeRowPointer(); }
   void resize(int m, int n)               // change dimensions
      { DeleteRowPointer(); Matrix::resize(m,n); MakeRowPointer(); }
   void resize_keep(int m, int n)               // change dimensions
      { DeleteRowPointer(); Matrix::resize_keep(m,n); MakeRowPointer(); }
   void ReSize(int m, int n)               // change dimensions
      { DeleteRowPointer(); Matrix::resize(m,n); MakeRowPointer(); }
   void resize(const GeneralMatrix& A);
   void ReSize(const GeneralMatrix& A) { resize(A); }
   ~nricMatrix() { DeleteRowPointer(); }
   Real** nric() const { CheckStore(); return row_pointer-1; }
   void cleanup();                                // to clear store
   void MiniCleanUp();
   void operator+=(const Matrix& M) { PlusEqual(M); }
   void operator-=(const Matrix& M) { MinusEqual(M); }
   void operator+=(Real f) { GeneralMatrix::Add(f); }
   void operator-=(Real f) { GeneralMatrix::Add(-f); }
   void swap(nricMatrix& gm);
   NEW_DELETE(nricMatrix)
};

#endif

