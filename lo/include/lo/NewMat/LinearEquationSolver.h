#ifndef __NEW_MAT_LINEAR_EQUATION_SOLVER_H__
#define __NEW_MAT_LINEAR_EQUATION_SOLVER_H__

// ******************** linear equation solving ****************************/

/// A class for finding A.i() * B.
/// This is supposed to choose the appropriate method depending on the
/// type A. Not very satisfactory as it doesn't know about Cholesky for
/// for positive definite matrices.
class LinearEquationSolver : public BaseMatrix
{
   GeneralMatrix* gm;
   int search(const BaseMatrix*) const { return 0; }
   friend class BaseMatrix;
public:
   LinearEquationSolver(const BaseMatrix& bm);
   ~LinearEquationSolver() { delete gm; }
   void cleanup() { delete gm; }
   GeneralMatrix* Evaluate(MatrixType) { return gm; }
   // probably should have an error message if MatrixType != UnSp
   NEW_DELETE(LinearEquationSolver)
};

#endif

