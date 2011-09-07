#ifndef __NEW_MAT_SUB_MATRIX_H__
#define __NEW_MAT_SUB_MATRIX_H__

// ************************** submatrices ******************************/

/// A submatrix of a matrix.
/// \internal
class GetSubMatrix : public NegatedMatrix
{
   int row_skip;
   int row_number;
   int col_skip;
   int col_number;
   bool IsSym;

   GetSubMatrix
      (const BaseMatrix* bmx, int rs, int rn, int cs, int cn, bool is)
      : NegatedMatrix(bmx),
      row_skip(rs), row_number(rn), col_skip(cs), col_number(cn), IsSym(is) {}
   void SetUpLHS();
   friend class BaseMatrix;
public:
   GetSubMatrix() : row_skip(0), row_number(0), col_skip(0), col_number(0) {}
   GetSubMatrix(const GetSubMatrix& g)
      : NegatedMatrix(g.bm), row_skip(g.row_skip), row_number(g.row_number),
      col_skip(g.col_skip), col_number(g.col_number), IsSym(g.IsSym) {}
   ~GetSubMatrix() {}
   GeneralMatrix* Evaluate(MatrixType mt=MatrixTypeUnSp);
   void operator=(const BaseMatrix&);
   void operator+=(const BaseMatrix&);
   void operator-=(const BaseMatrix&);
   void operator=(const GetSubMatrix& m) { operator=((const BaseMatrix&)m); }
   void operator<<(const BaseMatrix&);
   void operator<<(const double*);                // copy from array
   void operator<<(const float*);                // copy from array
   void operator<<(const int*);                 // copy from array
   MatrixInput operator<<(double);                // for loading a list
   MatrixInput operator<<(float);                // for loading a list
   MatrixInput operator<<(int f);
   void operator=(Real);                        // copy from constant
   void operator+=(Real);                       // add constant
   void operator-=(Real r) { operator+=(-r); }  // subtract constant
   void operator*=(Real);                       // multiply by constant
   void operator/=(Real r) { operator*=(1.0/r); } // divide by constant
   void inject(const GeneralMatrix&);           // copy stored els only
   void Inject(const GeneralMatrix& GM) { inject(GM); }
   MatrixBandWidth bandwidth() const;
   NEW_DELETE(GetSubMatrix)
};

#endif

