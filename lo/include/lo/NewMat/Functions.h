#ifndef __NEW_MAT_FUNCTIONS_H__
#define __NEW_MAT_FUNCTIONS_H__

#ifdef use_namespace
namespace NEWMAT {
#endif


// ************************ functions ************************************ //

bool operator==(const GeneralMatrix& A, const GeneralMatrix& B);
bool operator==(const BaseMatrix& A, const BaseMatrix& B);
inline bool operator!=(const GeneralMatrix& A, const GeneralMatrix& B)
   { return ! (A==B); }
inline bool operator!=(const BaseMatrix& A, const BaseMatrix& B)
   { return ! (A==B); }

   // inequality operators are dummies included for compatibility
   // with STL. They throw an exception if actually called.
inline bool operator<=(const BaseMatrix& A, const BaseMatrix&)
   { A.IEQND(); return true; }
inline bool operator>=(const BaseMatrix& A, const BaseMatrix&)
   { A.IEQND(); return true; }
inline bool operator<(const BaseMatrix& A, const BaseMatrix&)
   { A.IEQND(); return true; }
inline bool operator>(const BaseMatrix& A, const BaseMatrix&)
   { A.IEQND(); return true; }

bool is_zero(const BaseMatrix& A);
inline bool IsZero(const BaseMatrix& A) { return is_zero(A); }

Real dotproduct(const Matrix& A, const Matrix& B);
Matrix crossproduct(const Matrix& A, const Matrix& B);
ReturnMatrix crossproduct_rows(const Matrix& A, const Matrix& B);
ReturnMatrix crossproduct_columns(const Matrix& A, const Matrix& B);

inline Real DotProduct(const Matrix& A, const Matrix& B)
   { return dotproduct(A, B); }
inline Matrix CrossProduct(const Matrix& A, const Matrix& B)
   { return crossproduct(A, B); }
inline ReturnMatrix CrossProductRows(const Matrix& A, const Matrix& B)
   { return crossproduct_rows(A, B); }
inline ReturnMatrix CrossProductColumns(const Matrix& A, const Matrix& B)
   { return crossproduct_columns(A, B); }

void newmat_block_copy(int n, Real* from, Real* to);

// ********************* friend functions ******************************** //

// Functions declared as friends - G++ wants them declared externally as well

bool Rectangular(MatrixType a, MatrixType b, MatrixType c);
bool Compare(const MatrixType&, MatrixType&);
Real dotproduct(const Matrix& A, const Matrix& B);
SPMatrix SP(const BaseMatrix&, const BaseMatrix&);
KPMatrix KP(const BaseMatrix&, const BaseMatrix&);
ShiftedMatrix operator+(Real f, const BaseMatrix& BM);
NegShiftedMatrix operator-(Real, const BaseMatrix&);
ScaledMatrix operator*(Real f, const BaseMatrix& BM);

// ********************* inline functions ******************************** //

inline LogAndSign log_determinant(const BaseMatrix& B)
   { return B.log_determinant(); }
inline LogAndSign LogDeterminant(const BaseMatrix& B)
   { return B.log_determinant(); }
inline Real determinant(const BaseMatrix& B)
   { return B.determinant(); }
inline Real Determinant(const BaseMatrix& B)
   { return B.determinant(); }
inline Real sum_square(const BaseMatrix& B) { return B.sum_square(); }
inline Real SumSquare(const BaseMatrix& B) { return B.sum_square(); }
inline Real norm_Frobenius(const BaseMatrix& B) { return B.norm_Frobenius(); }
inline Real norm_frobenius(const BaseMatrix& B) { return B.norm_Frobenius(); }
inline Real NormFrobenius(const BaseMatrix& B) { return B.norm_Frobenius(); }
inline Real trace(const BaseMatrix& B) { return B.trace(); }
inline Real Trace(const BaseMatrix& B) { return B.trace(); }
inline Real sum_absolute_value(const BaseMatrix& B)
   { return B.sum_absolute_value(); }
inline Real SumAbsoluteValue(const BaseMatrix& B)
   { return B.sum_absolute_value(); }
inline Real sum(const BaseMatrix& B)
   { return B.sum(); }
inline Real Sum(const BaseMatrix& B)
   { return B.sum(); }
inline Real maximum_absolute_value(const BaseMatrix& B)
   { return B.maximum_absolute_value(); }
inline Real MaximumAbsoluteValue(const BaseMatrix& B)
   { return B.maximum_absolute_value(); }
inline Real minimum_absolute_value(const BaseMatrix& B)
   { return B.minimum_absolute_value(); }
inline Real MinimumAbsoluteValue(const BaseMatrix& B)
   { return B.minimum_absolute_value(); }
inline Real maximum(const BaseMatrix& B) { return B.maximum(); }
inline Real Maximum(const BaseMatrix& B) { return B.maximum(); }
inline Real minimum(const BaseMatrix& B) { return B.minimum(); }
inline Real Minimum(const BaseMatrix& B) { return B.minimum(); }
inline Real norm1(const BaseMatrix& B) { return B.norm1(); }
inline Real Norm1(const BaseMatrix& B) { return B.norm1(); }
inline Real norm1(RowVector& RV) { return RV.maximum_absolute_value(); }
inline Real Norm1(RowVector& RV) { return RV.maximum_absolute_value(); }
inline Real norm_infinity(const BaseMatrix& B) { return B.norm_infinity(); }
inline Real NormInfinity(const BaseMatrix& B) { return B.norm_infinity(); }
inline Real norm_infinity(ColumnVector& CV)
   { return CV.maximum_absolute_value(); }
inline Real NormInfinity(ColumnVector& CV)
   { return CV.maximum_absolute_value(); }
inline bool IsZero(const GeneralMatrix& A) { return A.IsZero(); }
inline bool is_zero(const GeneralMatrix& A) { return A.is_zero(); }


inline MatrixInput MatrixInput::operator<<(int f) { return *this << (Real)f; }
inline MatrixInput GeneralMatrix::operator<<(int f) { return *this << (Real)f; }
inline MatrixInput BandMatrix::operator<<(int f) { return *this << (Real)f; }
inline MatrixInput GetSubMatrix::operator<<(int f) { return *this << (Real)f; }

inline ReversedMatrix BaseMatrix::Reverse() const { return reverse(); }
inline RowedMatrix BaseMatrix::AsRow() const { return as_row(); }
inline ColedMatrix BaseMatrix::AsColumn() const { return as_column(); }
inline DiagedMatrix BaseMatrix::AsDiagonal() const { return as_diagonal(); }
inline MatedMatrix BaseMatrix::AsMatrix(int m, int n) const
   { return as_matrix(m, n); }
inline GetSubMatrix BaseMatrix::SubMatrix(int fr, int lr, int fc, int lc) const
   { return submatrix(fr, lr, fc, lc); }
inline GetSubMatrix BaseMatrix::SymSubMatrix(int f, int l) const
   { return sym_submatrix(f, l); }
inline GetSubMatrix BaseMatrix::Row(int f) const { return row(f); }
inline GetSubMatrix BaseMatrix::Rows(int f, int l) const { return rows(f, l); }
inline GetSubMatrix BaseMatrix::Column(int f) const { return column(f); }
inline GetSubMatrix BaseMatrix::Columns(int f, int l) const
   { return columns(f, l); }
inline Real BaseMatrix::AsScalar() const { return as_scalar(); }

inline ReturnMatrix GeneralMatrix::ForReturn() const { return for_return(); }

inline void swap(Matrix& A, Matrix& B) { A.swap(B); }
inline void swap(SquareMatrix& A, SquareMatrix& B) { A.swap(B); }
inline void swap(nricMatrix& A, nricMatrix& B) { A.swap(B); }
inline void swap(UpperTriangularMatrix& A, UpperTriangularMatrix& B)
   { A.swap(B); }
inline void swap(LowerTriangularMatrix& A, LowerTriangularMatrix& B)
   { A.swap(B); }
inline void swap(SymmetricMatrix& A, SymmetricMatrix& B) { A.swap(B); }
inline void swap(DiagonalMatrix& A, DiagonalMatrix& B) { A.swap(B); }
inline void swap(RowVector& A, RowVector& B) { A.swap(B); }
inline void swap(ColumnVector& A, ColumnVector& B) { A.swap(B); }
inline void swap(CroutMatrix& A, CroutMatrix& B) { A.swap(B); }
inline void swap(BandMatrix& A, BandMatrix& B) { A.swap(B); }
inline void swap(UpperBandMatrix& A, UpperBandMatrix& B) { A.swap(B); }
inline void swap(LowerBandMatrix& A, LowerBandMatrix& B) { A.swap(B); }
inline void swap(SymmetricBandMatrix& A, SymmetricBandMatrix& B) { A.swap(B); }
inline void swap(BandLUMatrix& A, BandLUMatrix& B) { A.swap(B); }
inline void swap(IdentityMatrix& A, IdentityMatrix& B) { A.swap(B); }
inline void swap(GenericMatrix& A, GenericMatrix& B) { A.swap(B); }

#ifdef OPT_COMPATIBLE                    // for compatibility with opt++

inline Real Norm2(const ColumnVector& CV) { return CV.norm_Frobenius(); }
inline Real Dot(ColumnVector& CV1, ColumnVector& CV2)
   { return dotproduct(CV1, CV2); }

#endif

// ************************** applications *****************************/


void QRZT(Matrix&, LowerTriangularMatrix&);

void QRZT(const Matrix&, Matrix&, Matrix&);

void QRZ(Matrix&, UpperTriangularMatrix&);

void QRZ(const Matrix&, Matrix&, Matrix&);

inline void HHDecompose(Matrix& X, LowerTriangularMatrix& L)
{ QRZT(X,L); }

inline void HHDecompose(const Matrix& X, Matrix& Y, Matrix& M)
{ QRZT(X, Y, M); }

void updateQRZT(Matrix& X, LowerTriangularMatrix& L);

void updateQRZ(Matrix& X, UpperTriangularMatrix& U);

inline void UpdateQRZT(Matrix& X, LowerTriangularMatrix& L)
   { updateQRZT(X, L); }

inline void UpdateQRZ(Matrix& X, UpperTriangularMatrix& U)
   { updateQRZ(X, U); }

// Matrix A's first n columns are orthonormal
// so A.Columns(1,n).t() * A.Columns(1,n) is the identity matrix.
// Fill out the remaining columns of A to make them orthonormal
// so A.t() * A is the identity matrix
void extend_orthonormal(Matrix& A, int n);


ReturnMatrix Cholesky(const SymmetricMatrix&);

ReturnMatrix Cholesky(const SymmetricBandMatrix&);


// produces the Cholesky decomposition of A + x.t() * x where A = chol.t() * chol
// and x is a RowVector
void update_Cholesky(UpperTriangularMatrix& chol, RowVector x);
inline void UpdateCholesky(UpperTriangularMatrix& chol, const RowVector& x)
   { update_Cholesky(chol, x); }

// produces the Cholesky decomposition of A - x.t() * x where A = chol.t() * chol
// and x is a RowVector
void downdate_Cholesky(UpperTriangularMatrix &chol, RowVector x);
inline void DowndateCholesky(UpperTriangularMatrix &chol, const RowVector& x)
   { downdate_Cholesky(chol, x); }

// a RIGHT circular shift of the rows and columns from
// 1,...,k-1,k,k+1,...l,l+1,...,p to
// 1,...,k-1,l,k,k+1,...l-1,l+1,...p
void right_circular_update_Cholesky(UpperTriangularMatrix &chol, int k, int l);
inline void RightCircularUpdateCholesky(UpperTriangularMatrix &chol,
  int k, int l) { right_circular_update_Cholesky(chol, k, l); }

// a LEFT circular shift of the rows and columns from
// 1,...,k-1,k,k+1,...l,l+1,...,p to
// 1,...,k-1,k+1,...l,k,l+1,...,p to
void left_circular_update_Cholesky(UpperTriangularMatrix &chol, int k, int l);
inline void LeftCircularUpdateCholesky(UpperTriangularMatrix &chol,
   int k, int l) { left_circular_update_Cholesky(chol, k, l); }


void SVD(const Matrix&, DiagonalMatrix&, Matrix&, Matrix&,
    bool=true, bool=true);

void SVD(const Matrix&, DiagonalMatrix&);

inline void SVD(const Matrix& A, DiagonalMatrix& D, Matrix& U,
   bool withU = true) { SVD(A, D, U, U, withU, false); }

void SortSV(DiagonalMatrix& D, Matrix& U, bool ascending = false);

void SortSV(DiagonalMatrix& D, Matrix& U, Matrix& V, bool ascending = false);

void Jacobi(const SymmetricMatrix&, DiagonalMatrix&);

void Jacobi(const SymmetricMatrix&, DiagonalMatrix&, SymmetricMatrix&);

void Jacobi(const SymmetricMatrix&, DiagonalMatrix&, Matrix&);

void Jacobi(const SymmetricMatrix&, DiagonalMatrix&, SymmetricMatrix&,
   Matrix&, bool=true);

void eigenvalues(const SymmetricMatrix&, DiagonalMatrix&);

void eigenvalues(const SymmetricMatrix&, DiagonalMatrix&, SymmetricMatrix&);

void eigenvalues(const SymmetricMatrix&, DiagonalMatrix&, Matrix&);

inline void EigenValues(const SymmetricMatrix& A, DiagonalMatrix& D)
   { eigenvalues(A, D); }

inline void EigenValues(const SymmetricMatrix& A, DiagonalMatrix& D,
   SymmetricMatrix& S) { eigenvalues(A, D, S); }

inline void EigenValues(const SymmetricMatrix& A, DiagonalMatrix& D, Matrix& V)
   { eigenvalues(A, D, V); }

class SymmetricEigenAnalysis
// not implemented yet
{
public:
   SymmetricEigenAnalysis(const SymmetricMatrix&);
private:
   DiagonalMatrix diag;
   DiagonalMatrix offdiag;
   SymmetricMatrix backtransform;
   FREE_CHECK(SymmetricEigenAnalysis)
};

void sort_ascending(GeneralMatrix&);

void sort_descending(GeneralMatrix&);

inline void SortAscending(GeneralMatrix& gm) { sort_ascending(gm); }

inline void SortDescending(GeneralMatrix& gm) { sort_descending(gm); }

/// Decide which fft method to use and carry out new fft function
class FFT_Controller
{
public:
   static bool OnlyOldFFT;
   static bool ar_1d_ft (int PTS, Real* X, Real *Y);
   static bool CanFactor(int PTS);
};

void FFT(const ColumnVector&, const ColumnVector&,
   ColumnVector&, ColumnVector&);

void FFTI(const ColumnVector&, const ColumnVector&,
   ColumnVector&, ColumnVector&);

void RealFFT(const ColumnVector&, ColumnVector&, ColumnVector&);

void RealFFTI(const ColumnVector&, const ColumnVector&, ColumnVector&);

void DCT_II(const ColumnVector&, ColumnVector&);

void DCT_II_inverse(const ColumnVector&, ColumnVector&);

void DST_II(const ColumnVector&, ColumnVector&);

void DST_II_inverse(const ColumnVector&, ColumnVector&);

void DCT(const ColumnVector&, ColumnVector&);

void DCT_inverse(const ColumnVector&, ColumnVector&);

void DST(const ColumnVector&, ColumnVector&);

void DST_inverse(const ColumnVector&, ColumnVector&);

void FFT2(const Matrix& U, const Matrix& V, Matrix& X, Matrix& Y);

void FFT2I(const Matrix& U, const Matrix& V, Matrix& X, Matrix& Y);


// This class is used by the new FFT program

// Suppose an integer is expressed as a sequence of digits with each
// digit having a different radix.
// This class supposes we are counting with this multi-radix number
// but also keeps track of the number with the digits (and radices)
// reversed.
// The integer starts at zero
// operator++() increases it by 1
// Counter gives the number of increments
// Reverse() gives the value with the digits in reverse order
// Swap is true if reverse is less than counter
// Finish is true when we have done a complete cycle and are back at zero

class MultiRadixCounter
{
   const SimpleIntArray& Radix;
                              // radix of each digit
                              // n-1 highest order, 0 lowest order
   SimpleIntArray& Value;     // value of each digit
   const int n;               // number of digits
   int reverse;               // value when order of digits is reversed
   int product;               // product of radices
   int counter;               // counter
   bool finish;               // true when we have gone over whole range
public:
   MultiRadixCounter(int nx, const SimpleIntArray& rx,
      SimpleIntArray& vx);
   void operator++();         // increment the multi-radix counter
   bool Swap() const { return reverse < counter; }
   bool Finish() const { return finish; }
   int Reverse() const { return reverse; }
   int Counter() const { return counter; }
};

// multiplication by Helmert matrix
ReturnMatrix Helmert(int n, bool full=false);
ReturnMatrix Helmert(const ColumnVector& X, bool full=false);
ReturnMatrix Helmert(int n, int j, bool full=false);
ReturnMatrix Helmert_transpose(const ColumnVector& Y, bool full=false);
Real Helmert_transpose(const ColumnVector& Y, int j, bool full=false);
ReturnMatrix Helmert(const Matrix& X, bool full=false);
ReturnMatrix Helmert_transpose(const Matrix& Y, bool full=false);



// Copyright (C) 1991,2,3,4: R B Davies

#ifndef WANT_STREAM
#define WANT_STREAM
#endif

// **************************** input/output *****************************/

ostream& operator<<(ostream&, const BaseMatrix&);

ostream& operator<<(ostream&, const GeneralMatrix&);


/*  Use in some old versions of G++ without complete iomanipulators

class Omanip_precision
{
   int x;
public:
   Omanip_precision(int i) : x(i) {}
   friend ostream& operator<<(ostream& os, Omanip_precision i);
};


Omanip_precision setprecision(int i);

class Omanip_width
{
   int x;
public:
   Omanip_width(int i) : x(i) {}
   friend ostream& operator<<(ostream& os, Omanip_width i);
};

Omanip_width setw(int i);

*/

#ifdef use_namespace
}
#endif

#ifdef use_namespace
namespace ROBOOP {
  using namespace NEWMAT;
#endif

#ifndef M_PI
#define M_PI 3.14159265358979
#endif

#define GRAVITY 9.81

// global variables
extern Real fourbyfourident[];
extern Real threebythreeident[];

// angle conversion
inline double deg2rad(const double angle_deg){ return angle_deg*M_PI/180; }
inline double rad2deg(const double angle_rad){ return angle_rad*180/M_PI; }

// vector operation

ReturnMatrix x_prod_matrix(const ColumnVector & x);

ReturnMatrix pinv(const Matrix & M);

// numerical analysis tools

ReturnMatrix Integ_Trap(const ColumnVector & present, ColumnVector & past, const Real dt);

void Runge_Kutta4(ReturnMatrix (*xdot)(Real time, const Matrix & xin),
                  const Matrix & xo, Real to, Real tf, int nsteps,
                  RowVector & tout, Matrix & xout);

void Runge_Kutta4_Real_time(ReturnMatrix (*xdot)(Real time, const Matrix & xin),
                            const Matrix & xo, Real to, Real tf, int nsteps);

void Runge_Kutta4_Real_time(ReturnMatrix (*xdot)(Real time, const Matrix & xin,
                            bool & exit, bool & init),
                            const Matrix & xo, Real to, Real tf, int nsteps);

void odeint(ReturnMatrix (*xdot)(Real time, const Matrix & xin),
            Matrix & xo, Real to, Real tf, Real eps, Real h1, Real hmin,
            int & nok, int & nbad,
            RowVector & tout, Matrix & xout, Real dtsav);

ReturnMatrix sign(const Matrix & x);

short sign(const Real x);

const double epsilon = 0.0000001;

inline bool isZero(const double x)
{
    if ( fabs(x) < epsilon)
    {
        return true;
    }
    return false;
}


// translation
ReturnMatrix trans(const ColumnVector & a);

// rotation matrices
ReturnMatrix rotx(const Real alpha);
ReturnMatrix roty(const Real beta);
ReturnMatrix rotz(const Real gamma);
ReturnMatrix rotk(const Real theta, const ColumnVector & k);

ReturnMatrix rpy(const ColumnVector & a);
ReturnMatrix eulzxz(const ColumnVector & a);
ReturnMatrix rotd(const Real theta, const ColumnVector & k1, const ColumnVector & k2);


// inverse on rotation matrices
ReturnMatrix irotk(const Matrix & R);
ReturnMatrix irpy(const Matrix & R);
ReturnMatrix ieulzxz(const Matrix & R);

#ifdef use_namespace
}
#endif

#endif

