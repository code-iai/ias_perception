#ifndef __NEW_MAT_SYMMETRIX_MATRIX_H__
#define __NEW_MAT_SYMMETRIX_MATRIX_H__

/// Symmetric matrix.
class SymmetricMatrix : public GeneralMatrix
{
   GeneralMatrix* Image() const;                // copy of matrix
public:
   SymmetricMatrix() {}
   ~SymmetricMatrix() {}
   SymmetricMatrix(ArrayLengthSpecifier);
   SymmetricMatrix(const BaseMatrix&);
   void operator=(const BaseMatrix&);
   void operator=(Real f) { GeneralMatrix::operator=(f); }
   void operator=(const SymmetricMatrix& m) { Eq(m); }
   Real& operator()(int, int);                  // access element
   Real& element(int, int);                     // access element
   Real operator()(int, int) const;             // access element
   Real element(int, int) const;                // access element
#ifdef SETUP_C_SUBSCRIPTS
   Real* operator[](int m) { return store+(m*(m+1))/2; }
   const Real* operator[](int m) const { return store+(m*(m+1))/2; }
#endif
   MatrixType type() const;
   SymmetricMatrix(const SymmetricMatrix& gm)
      : GeneralMatrix() { GetMatrix(&gm); }
   Real sum_square() const;
   Real sum_absolute_value() const;
   Real sum() const;
   Real trace() const;
   void GetRow(MatrixRowCol&);
   void GetCol(MatrixRowCol&);
   void GetCol(MatrixColX&);
   void RestoreCol(MatrixRowCol&) {}
   void RestoreCol(MatrixColX&);
   GeneralMatrix* Transpose(TransposedMatrix*, MatrixType);
   void resize(int);                           // change dimensions
   void ReSize(int m) { resize(m); }
   void resize_keep(int);
   void resize(const GeneralMatrix& A);
   void ReSize(const GeneralMatrix& A) { resize(A); }
   void operator+=(const SymmetricMatrix& M) { PlusEqual(M); }
   void operator-=(const SymmetricMatrix& M) { MinusEqual(M); }
   void operator+=(Real f) { GeneralMatrix::Add(f); }
   void operator-=(Real f) { GeneralMatrix::Add(-f); }
   void swap(SymmetricMatrix& gm) { GeneralMatrix::swap((GeneralMatrix&)gm); }
   NEW_DELETE(SymmetricMatrix)
};

#endif

