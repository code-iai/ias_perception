#ifndef __NEW_MAT_CROUT_MATIX_H__
#define __NEW_MAT_CROUT_MATIX_H__

/// LU matrix.
/// A square matrix decomposed into upper and lower triangular
/// in preparation for inverting or solving equations.
class CroutMatrix : public GeneralMatrix
{
   int* indx;
   bool d;                              // number of row swaps = even or odd
   bool sing;
   void ludcmp();
   void get_aux(CroutMatrix&);                  // for copying indx[] etc
   GeneralMatrix* Image() const;                // copy of matrix
public:
   CroutMatrix(const BaseMatrix&);
   CroutMatrix() : indx(0), d(true), sing(true) {}
   CroutMatrix(const CroutMatrix&);
   void operator=(const CroutMatrix&);
   GeneralMatrix* Evaluate(MatrixType mt=MatrixTypeUnSp);
   MatrixType type() const;
   void lubksb(Real*, int=0);
   ~CroutMatrix();
   GeneralMatrix* MakeSolver() { return this; } // for solving
   LogAndSign log_determinant() const;
   void Solver(MatrixColX&, const MatrixColX&);
   void GetRow(MatrixRowCol&);
   void GetCol(MatrixRowCol&);
   void GetCol(MatrixColX& c) { GetCol((MatrixRowCol&)c); }
   void cleanup();                                // to clear store
   void MiniCleanUp();
   bool IsEqual(const GeneralMatrix&) const;
   bool is_singular() const { return sing; }
   bool IsSingular() const { return sing; }
   const int* const_data_indx() const { return indx; }
   bool even_exchanges() const { return d; }
   void swap(CroutMatrix& gm);
   NEW_DELETE(CroutMatrix)
};

#endif

