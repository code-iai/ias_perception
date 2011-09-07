#ifndef __NEW_MAT_BASE_MATRIX_H__
#define __NEW_MAT_BASE_MATRIX_H__

#define MatrixTypeUnSp 0
//static MatrixType MatrixTypeUnSp(MatrixType::US);
//						// AT&T needs this

/// Base of the matrix classes.
class BaseMatrix : public Janitor
{
protected:
   virtual int search(const BaseMatrix*) const = 0;
						// count number of times matrix is referred to
public:
   virtual GeneralMatrix* Evaluate(MatrixType mt=MatrixTypeUnSp) = 0;
						// evaluate temporary
   // for old version of G++
   //   virtual GeneralMatrix* Evaluate(MatrixType mt) = 0;
   //   GeneralMatrix* Evaluate() { return Evaluate(MatrixTypeUnSp); }
   AddedMatrix operator+(const BaseMatrix&) const;    // results of operations
   MultipliedMatrix operator*(const BaseMatrix&) const;
   SubtractedMatrix operator-(const BaseMatrix&) const;
   ConcatenatedMatrix operator|(const BaseMatrix&) const;
   StackedMatrix operator&(const BaseMatrix&) const;
   ShiftedMatrix operator+(Real) const;
   ScaledMatrix operator*(Real) const;
   ScaledMatrix operator/(Real) const;
   ShiftedMatrix operator-(Real) const;
   TransposedMatrix t() const;
//   TransposedMatrix t;
   NegatedMatrix operator-() const;                   // change sign of elements
   ReversedMatrix reverse() const;
   ReversedMatrix Reverse() const;
   InvertedMatrix i() const;
//   InvertedMatrix i;
   RowedMatrix as_row() const;
   RowedMatrix AsRow() const;
   ColedMatrix as_column() const;
   ColedMatrix AsColumn() const;
   DiagedMatrix as_diagonal() const;
   DiagedMatrix AsDiagonal() const;
   MatedMatrix as_matrix(int,int) const;
   MatedMatrix AsMatrix(int m, int n) const;
   GetSubMatrix submatrix(int,int,int,int) const;
   GetSubMatrix SubMatrix(int fr, int lr, int fc, int lc) const;
   GetSubMatrix sym_submatrix(int,int) const;
   GetSubMatrix SymSubMatrix(int f, int l) const;
   GetSubMatrix row(int) const;
   GetSubMatrix rows(int,int) const;
   GetSubMatrix column(int) const;
   GetSubMatrix columns(int,int) const;
   GetSubMatrix Row(int f) const;
   GetSubMatrix Rows(int f, int l) const;
   GetSubMatrix Column(int f) const;
   GetSubMatrix Columns(int f, int l) const;
   Real as_scalar() const;                      // conversion of 1 x 1 matrix
   Real AsScalar() const;
   virtual LogAndSign log_determinant() const;
   LogAndSign LogDeterminant() const { return log_determinant(); }
   Real determinant() const;
   Real Determinant() const { return determinant(); }
   virtual Real sum_square() const;
   Real SumSquare() const { return sum_square(); }
   Real norm_Frobenius() const;
   Real norm_frobenius() const { return norm_Frobenius(); }
   Real NormFrobenius() const { return norm_Frobenius(); }
   virtual Real sum_absolute_value() const;
   Real SumAbsoluteValue() const { return sum_absolute_value(); }
   virtual Real sum() const;
   virtual Real Sum() const { return sum(); }
   virtual Real maximum_absolute_value() const;
   Real MaximumAbsoluteValue() const { return maximum_absolute_value(); }
   virtual Real maximum_absolute_value1(int& i) const;
   Real MaximumAbsoluteValue1(int& i) const
      { return maximum_absolute_value1(i); }
   virtual Real maximum_absolute_value2(int& i, int& j) const;
   Real MaximumAbsoluteValue2(int& i, int& j) const
      { return maximum_absolute_value2(i,j); }
   virtual Real minimum_absolute_value() const;
   Real MinimumAbsoluteValue() const { return minimum_absolute_value(); }
   virtual Real minimum_absolute_value1(int& i) const;
   Real MinimumAbsoluteValue1(int& i) const
      { return minimum_absolute_value1(i); }
   virtual Real minimum_absolute_value2(int& i, int& j) const;
   Real MinimumAbsoluteValue2(int& i, int& j) const
      { return minimum_absolute_value2(i,j); }
   virtual Real maximum() const;
   Real Maximum() const { return maximum(); }
   virtual Real maximum1(int& i) const;
   Real Maximum1(int& i) const { return maximum1(i); }
   virtual Real maximum2(int& i, int& j) const;
   Real Maximum2(int& i, int& j) const { return maximum2(i,j); }
   virtual Real minimum() const;
   Real Minimum() const { return minimum(); }
   virtual Real minimum1(int& i) const;
   Real Minimum1(int& i) const { return minimum1(i); }
   virtual Real minimum2(int& i, int& j) const;
   Real Minimum2(int& i, int& j) const { return minimum2(i,j); }
   virtual Real trace() const;
   Real Trace() const { return trace(); }
   Real norm1() const;
   Real Norm1() const { return norm1(); }
   Real norm_infinity() const;
   Real NormInfinity() const { return norm_infinity(); }
   virtual MatrixBandWidth bandwidth() const;  // bandwidths of band matrix
   virtual MatrixBandWidth BandWidth() const { return bandwidth(); }
   void IEQND() const;                         // called by ineq. ops
   ReturnMatrix sum_square_columns() const;
   ReturnMatrix sum_square_rows() const;
   ReturnMatrix sum_columns() const;
   ReturnMatrix sum_rows() const;
   virtual void cleanup() {}
   void CleanUp() { cleanup(); }

//   virtual ReturnMatrix Reverse() const;       // reverse order of elements
//protected:
//   BaseMatrix() : t(this), i(this) {}

   friend class GeneralMatrix;
   friend class Matrix;
   friend class SquareMatrix;
   friend class nricMatrix;
   friend class RowVector;
   friend class ColumnVector;
   friend class SymmetricMatrix;
   friend class UpperTriangularMatrix;
   friend class LowerTriangularMatrix;
   friend class DiagonalMatrix;
   friend class CroutMatrix;
   friend class BandMatrix;
   friend class LowerBandMatrix;
   friend class UpperBandMatrix;
   friend class SymmetricBandMatrix;
   friend class AddedMatrix;
   friend class MultipliedMatrix;
   friend class SubtractedMatrix;
   friend class SPMatrix;
   friend class KPMatrix;
   friend class ConcatenatedMatrix;
   friend class StackedMatrix;
   friend class SolvedMatrix;
   friend class ShiftedMatrix;
   friend class NegShiftedMatrix;
   friend class ScaledMatrix;
   friend class TransposedMatrix;
   friend class ReversedMatrix;
   friend class NegatedMatrix;
   friend class InvertedMatrix;
   friend class RowedMatrix;
   friend class ColedMatrix;
   friend class DiagedMatrix;
   friend class MatedMatrix;
   friend class GetSubMatrix;
   friend class ReturnMatrix;
   friend class LinearEquationSolver;
   friend class GenericMatrix;
   NEW_DELETE(BaseMatrix)
};

#endif

