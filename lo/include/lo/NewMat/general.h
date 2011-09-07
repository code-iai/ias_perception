#ifndef NEW_MAT_GENERAL_H
#define NEW_MAT_GENERAL_H

#ifdef use_namespace
namespace NEWMAT { using namespace RBD_COMMON; }
namespace RBD_LIBRARIES { using namespace NEWMAT; }
namespace NEWMAT {
#endif

//#define DO_REPORT                     // to activate REPORT

#ifdef NO_LONG_NAMES
#define UpperTriangularMatrix UTMatrix
#define LowerTriangularMatrix LTMatrix
#define SymmetricMatrix SMatrix
#define DiagonalMatrix DMatrix
#define BandMatrix BMatrix
#define UpperBandMatrix UBMatrix
#define LowerBandMatrix LBMatrix
#define SymmetricBandMatrix SBMatrix
#define BandLUMatrix BLUMatrix
#endif

// ************************** general utilities ****************************/

class GeneralMatrix;                            // defined later
class BaseMatrix;                               // defined later
class MatrixInput;                              // defined later

void MatrixErrorNoSpace(const void*);           ///< test for allocation fails

/// Return from LogDeterminant function.
/// Members are the log of the absolute value and the sign (+1, -1 or 0)
class LogAndSign
{
   Real log_val;
   int sign_val;
public:
   LogAndSign() { log_val=0.0; sign_val=1; }
   LogAndSign(Real);
   void operator*=(Real);                       ///< multiply by a real
   void pow_eq(int k);                          ///< raise to power of k
   void PowEq(int k) { pow_eq(k); }
   void ChangeSign() { sign_val = -sign_val; }
   void change_sign() { sign_val = -sign_val; } ///< change sign
   Real LogValue() const { return log_val; }
   Real log_value() const { return log_val; }   ///< log of the absolute value
   int Sign() const { return sign_val; }
   int sign() const { return sign_val; }        ///< sign of the value
   Real value() const;                          ///< the value
   Real Value() const { return value(); }
   FREE_CHECK(LogAndSign)
};

// the following class is for counting the number of times a piece of code
// is executed. It is used for locating any code not executed by test
// routines. Use turbo GREP locate all places this code is called and
// check which ones are not accessed.
// Somewhat implementation dependent as it relies on "cout" still being
// present when ExeCounter objects are destructed.

#ifdef DO_REPORT

class ExeCounter
{
   int line;                                    // code line number
   int fileid;                                  // file identifier
   long nexe;                                   // number of executions
   static int nreports;                         // number of reports
public:
   ExeCounter(int,int);
   void operator++() { nexe++; }
   ~ExeCounter();                               // prints out reports
};

#endif


// ************************** class MatrixType *****************************

/// Find the type of a matrix resulting from matrix operations.
/// Also identify what conversions are permissible.
/// This class must be updated when new matrix types are added.

class MatrixType
{
public:
   enum Attribute {  Valid     = 1,
                     Diagonal  = 2,             // order of these is important
                     Symmetric = 4,
                     Band      = 8,
                     Lower     = 16,
                     Upper     = 32,
                     Square    = 64,
                     Skew      = 128,
                     LUDeco    = 256,
                     Ones      = 512 };

   enum            { US = 0,
                     UT = Valid + Upper + Square,
                     LT = Valid + Lower + Square,
                     Rt = Valid,
                     Sq = Valid + Square,
                     Sm = Valid + Symmetric + Square,
                     Sk = Valid + Skew + Square,
                     Dg = Valid + Diagonal + Band + Lower + Upper + Symmetric
                        + Square,
                     Id = Valid + Diagonal + Band + Lower + Upper + Symmetric
                        + Square + Ones,
                     RV = Valid,     //   do not separate out
                     CV = Valid,     //   vectors
                     BM = Valid + Band + Square,
                     UB = Valid + Band + Upper + Square,
                     LB = Valid + Band + Lower + Square,
                     SB = Valid + Band + Symmetric + Square,
                     KB = Valid + Band + Skew + Square,
                     Ct = Valid + LUDeco + Square,
                     BC = Valid + Band + LUDeco + Square,
                     Mask = ~Square
                   };


   static int nTypes() { return 13; }          // number of different types
					       // exclude Ct, US, BC
public:
   int attribute;
   bool DataLossOK;                            // true if data loss is OK when
                                               // this represents a destination
public:
   MatrixType () : DataLossOK(false) {}
   MatrixType (int i) : attribute(i), DataLossOK(false) {}
   MatrixType (int i, bool dlok) : attribute(i), DataLossOK(dlok) {}
   MatrixType (const MatrixType& mt)
      : attribute(mt.attribute), DataLossOK(mt.DataLossOK) {}
   void operator=(const MatrixType& mt)
      { attribute = mt.attribute; DataLossOK = mt.DataLossOK; }
   void SetDataLossOK() { DataLossOK = true; }
   int operator+() const { return attribute; }
   MatrixType operator+(MatrixType mt) const
      { return MatrixType(attribute & mt.attribute); }
   MatrixType operator*(const MatrixType&) const;
   MatrixType SP(const MatrixType&) const;
   MatrixType KP(const MatrixType&) const;
   MatrixType operator|(const MatrixType& mt) const
      { return MatrixType(attribute & mt.attribute & Valid); }
   MatrixType operator&(const MatrixType& mt) const
      { return MatrixType(attribute & mt.attribute & Valid); }
   bool operator>=(MatrixType mt) const
      { return ( attribute & ~mt.attribute & Mask ) == 0; }
   bool operator<(MatrixType mt) const         // for MS Visual C++ 4
      { return ( attribute & ~mt.attribute & Mask ) != 0; }
   bool operator==(MatrixType t) const
      { return (attribute == t.attribute); }
   bool operator!=(MatrixType t) const
      { return (attribute != t.attribute); }
   bool operator!() const { return (attribute & Valid) == 0; }
   MatrixType i() const;                       ///< type of inverse
   MatrixType t() const;                       ///< type of transpose
   MatrixType AddEqualEl() const               ///< add constant to matrix
      { return MatrixType(attribute & (Valid + Symmetric + Square)); }
   MatrixType MultRHS() const;                 ///< type for rhs of multiply
   MatrixType sub() const                      ///< type of submatrix
      { return MatrixType(attribute & Valid); }
   MatrixType ssub() const                     ///< type of sym submatrix
      { return MatrixType(attribute); }        // not for selection matrix
   GeneralMatrix* New() const;                 ///< new matrix of given type
   GeneralMatrix* New(int,int,BaseMatrix*) const;
                                               ///< new matrix of given type
   const char* value() const;                  ///< type as char string
   const char* Value() const { return value(); }
   friend bool Rectangular(MatrixType a, MatrixType b, MatrixType c);
   friend bool Compare(const MatrixType&, MatrixType&);
                                               ///< compare and check conversion
   bool is_band() const { return (attribute & Band) != 0; }
   bool is_diagonal() const { return (attribute & Diagonal) != 0; }
   bool is_symmetric() const { return (attribute & Symmetric) != 0; }
   bool CannotConvert() const { return (attribute & LUDeco) != 0; }
                                               // used by operator==
   FREE_CHECK(MatrixType)
};


// *********************** class MatrixBandWidth ***********************/

///Upper and lower bandwidths of a matrix.
///That is number of diagonals strictly above or below main diagonal,
///e.g. diagonal matrix has 0 upper and lower bandwiths.
///-1 means the matrix may have the maximum bandwidth.
class MatrixBandWidth
{
public:
   int lower_val;
   int upper_val;
   MatrixBandWidth() : lower_val(0), upper_val(0) {}
   MatrixBandWidth(const int l, const int u) : lower_val(l), upper_val(u) {}
   MatrixBandWidth(const int i) : lower_val(i), upper_val(i) {}
   MatrixBandWidth operator+(const MatrixBandWidth&) const;
   MatrixBandWidth operator*(const MatrixBandWidth&) const;
   MatrixBandWidth minimum(const MatrixBandWidth&) const;
   MatrixBandWidth t() const { return MatrixBandWidth(upper_val,lower_val); }
   bool operator==(const MatrixBandWidth& bw) const
      { return (lower_val == bw.lower_val) && (upper_val == bw.upper_val); }
   bool operator!=(const MatrixBandWidth& bw) const { return !operator==(bw); }
   int Upper() const { return upper_val; }
   int upper() const { return upper_val; }
   int Lower() const { return lower_val; }
   int lower() const { return lower_val; }
   FREE_CHECK(MatrixBandWidth)
};


// ********************* Array length specifier ************************/

/// This class is used to avoid constructors such as
/// ColumnVector(int) being used for conversions.
/// Eventually this should be replaced by the use of the keyword "explicit".

class ArrayLengthSpecifier
{
   int v;
public:
   int Value() const { return v; }
   int value() const { return v; }
   ArrayLengthSpecifier(int l) : v(l) {}
};

// ************************* Matrix routines ***************************/


class MatrixRowCol;                             // defined later
class MatrixRow;
class MatrixCol;
class MatrixColX;

class GeneralMatrix;                            // defined later
class AddedMatrix;
class MultipliedMatrix;
class SubtractedMatrix;
class SPMatrix;
class KPMatrix;
class ConcatenatedMatrix;
class StackedMatrix;
class SolvedMatrix;
class ShiftedMatrix;
class NegShiftedMatrix;
class ScaledMatrix;
class TransposedMatrix;
class ReversedMatrix;
class NegatedMatrix;
class InvertedMatrix;
class RowedMatrix;
class ColedMatrix;
class DiagedMatrix;
class MatedMatrix;
class GetSubMatrix;
class ReturnMatrix;
class Matrix;
class SquareMatrix;
class nricMatrix;
class RowVector;
class ColumnVector;
class SymmetricMatrix;
class UpperTriangularMatrix;
class LowerTriangularMatrix;
class DiagonalMatrix;
class CroutMatrix;
class BandMatrix;
class LowerBandMatrix;
class UpperBandMatrix;
class SymmetricBandMatrix;
class LinearEquationSolver;
class GenericMatrix;

#endif

