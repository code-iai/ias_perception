#ifndef __NEW_MAT_MATRIX_INPUT_H__
#define __NEW_MAT_MATRIX_INPUT_H__

// ************************** matrix input *******************************/

/// Class for reading values into a (small) matrix within a program.
/// \internal
/// Is able to detect a mismatch in the number of elements.

class MatrixInput
{
   int n;                  // number values still to be read
   Real* r;                // pointer to next location to be read to
public:
   MatrixInput() : n(0), r(0) {}
   MatrixInput(const MatrixInput& mi) : n(mi.n), r(mi.r) {}
   MatrixInput(int nx, Real* rx) : n(nx), r(rx) {}
   ~MatrixInput();
   MatrixInput operator<<(double);
   MatrixInput operator<<(float);
   MatrixInput operator<<(int f);
   friend class GeneralMatrix;
};

#endif

