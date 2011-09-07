#ifndef __NEW_MAT_IDENTITY_MATRIX_H__
#define __NEW_MAT_IDENTITY_MATRIX_H__

/// Identity matrix.
class IdentityMatrix : public GeneralMatrix
{
   GeneralMatrix* Image() const;          // copy of matrix
public:
   IdentityMatrix() {}
   ~IdentityMatrix() {}
   IdentityMatrix(ArrayLengthSpecifier n) : GeneralMatrix(1)
      { nrows_val = ncols_val = n.Value(); *store = 1; }
   IdentityMatrix(const IdentityMatrix& gm)
      : GeneralMatrix() { GetMatrix(&gm); }
   IdentityMatrix(const BaseMatrix&);
   void operator=(const BaseMatrix&);
   void operator=(const IdentityMatrix& m) { Eq(m); }
   void operator=(Real f) { GeneralMatrix::operator=(f); }
   MatrixType type() const;

   LogAndSign log_determinant() const;
   Real trace() const;
   Real sum_square() const;
   Real sum_absolute_value() const;
   Real sum() const { return trace(); }
   void GetRow(MatrixRowCol&);
   void GetCol(MatrixRowCol&);
   void GetCol(MatrixColX&);
   void NextRow(MatrixRowCol&);
   void NextCol(MatrixRowCol&);
   void NextCol(MatrixColX&);
   GeneralMatrix* MakeSolver() { return this; } // for solving
   void Solver(MatrixColX&, const MatrixColX&);
   GeneralMatrix* Transpose(TransposedMatrix*, MatrixType);
   void resize(int n);
   void ReSize(int n) { resize(n); }
   void resize(const GeneralMatrix& A);
   void ReSize(const GeneralMatrix& A) { resize(A); }
   MatrixBandWidth bandwidth() const;
//   ReturnMatrix Reverse() const;                // reverse order of elements
   void swap(IdentityMatrix& gm)
      { GeneralMatrix::swap((GeneralMatrix&)gm); }
   NEW_DELETE(IdentityMatrix)
};


#endif

