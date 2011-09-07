#ifndef __NEW_MAT_TEMPORARY_H__
#define __NEW_MAT_TEMPORARY_H__

#include "myexcept.h"

// *************************** temporary classes *************************/

/// Product of two matrices.
/// \internal
class MultipliedMatrix : public BaseMatrix
{
protected:
   // if these union statements cause problems, simply remove them
   // and declare the items individually
   union { const BaseMatrix* bm1; GeneralMatrix* gm1; };
						  // pointers to summands
   union { const BaseMatrix* bm2; GeneralMatrix* gm2; };
   MultipliedMatrix(const BaseMatrix* bm1x, const BaseMatrix* bm2x)
      : bm1(bm1x),bm2(bm2x) {}
   int search(const BaseMatrix*) const;
   friend class BaseMatrix;
   friend class GeneralMatrix;
   friend class GenericMatrix;
public:
   MultipliedMatrix() : BaseMatrix(), bm1(0), bm2(0) {}
   ~MultipliedMatrix() {}
   GeneralMatrix* Evaluate(MatrixType mt=MatrixTypeUnSp);
   MatrixBandWidth bandwidth() const;
   NEW_DELETE(MultipliedMatrix)
};

/// Sum of two matrices.
/// \internal
class AddedMatrix : public MultipliedMatrix
{
protected:
   AddedMatrix(const BaseMatrix* bm1x, const BaseMatrix* bm2x)
      : MultipliedMatrix(bm1x,bm2x) {}

   friend class BaseMatrix;
   friend class GeneralMatrix;
   friend class GenericMatrix;
public:
   AddedMatrix() : MultipliedMatrix() {}
   ~AddedMatrix() {}
   GeneralMatrix* Evaluate(MatrixType mt=MatrixTypeUnSp);
   MatrixBandWidth bandwidth() const;
   NEW_DELETE(AddedMatrix)
};

/// Schur (elementwise) product of two matrices.
/// \internal
class SPMatrix : public AddedMatrix
{
protected:
   SPMatrix(const BaseMatrix* bm1x, const BaseMatrix* bm2x)
      : AddedMatrix(bm1x,bm2x) {}

   friend class BaseMatrix;
   friend class GeneralMatrix;
   friend class GenericMatrix;
public:
   SPMatrix() : AddedMatrix() {}
   ~SPMatrix() {}
   GeneralMatrix* Evaluate(MatrixType mt=MatrixTypeUnSp);
   MatrixBandWidth bandwidth() const;

   friend SPMatrix SP(const BaseMatrix&, const BaseMatrix&);

   NEW_DELETE(SPMatrix)
};

/// Kronecker product of two matrices.
/// \internal
class KPMatrix : public MultipliedMatrix
{
protected:
   KPMatrix(const BaseMatrix* bm1x, const BaseMatrix* bm2x)
      : MultipliedMatrix(bm1x,bm2x) {}

   friend class BaseMatrix;
   friend class GeneralMatrix;
   friend class GenericMatrix;
public:
   KPMatrix() : MultipliedMatrix() {}
   ~KPMatrix() {}
   MatrixBandWidth bandwidth() const;
   GeneralMatrix* Evaluate(MatrixType mt=MatrixTypeUnSp);
   friend KPMatrix KP(const BaseMatrix&, const BaseMatrix&);
   NEW_DELETE(KPMatrix)
};

/// Two matrices horizontally concatenated.
/// \internal
class ConcatenatedMatrix : public MultipliedMatrix
{
protected:
   ConcatenatedMatrix(const BaseMatrix* bm1x, const BaseMatrix* bm2x)
      : MultipliedMatrix(bm1x,bm2x) {}

   friend class BaseMatrix;
   friend class GeneralMatrix;
   friend class GenericMatrix;
public:
   ConcatenatedMatrix() : MultipliedMatrix() {}
   ~ConcatenatedMatrix() {}
   MatrixBandWidth bandwidth() const;
   GeneralMatrix* Evaluate(MatrixType mt=MatrixTypeUnSp);
   NEW_DELETE(ConcatenatedMatrix)
};

/// Two matrices vertically concatenated.
/// \internal
class StackedMatrix : public ConcatenatedMatrix
{
protected:
   StackedMatrix(const BaseMatrix* bm1x, const BaseMatrix* bm2x)
      : ConcatenatedMatrix(bm1x,bm2x) {}

   friend class BaseMatrix;
   friend class GeneralMatrix;
   friend class GenericMatrix;
public:
   StackedMatrix() :ConcatenatedMatrix() {}
   ~StackedMatrix() {}
   GeneralMatrix* Evaluate(MatrixType mt=MatrixTypeUnSp);
   NEW_DELETE(StackedMatrix)
};

/// Inverted matrix times matrix.
/// \internal
class SolvedMatrix : public MultipliedMatrix
{
   SolvedMatrix(const BaseMatrix* bm1x, const BaseMatrix* bm2x)
      : MultipliedMatrix(bm1x,bm2x) {}
   friend class BaseMatrix;
   friend class InvertedMatrix;                        // for operator*
public:
   SolvedMatrix() : MultipliedMatrix() {}
   ~SolvedMatrix() {}
   GeneralMatrix* Evaluate(MatrixType mt=MatrixTypeUnSp);
   MatrixBandWidth bandwidth() const;
   NEW_DELETE(SolvedMatrix)
};

/// Difference between two matrices.
/// \internal
class SubtractedMatrix : public AddedMatrix
{
   SubtractedMatrix(const BaseMatrix* bm1x, const BaseMatrix* bm2x)
      : AddedMatrix(bm1x,bm2x) {}
   friend class BaseMatrix;
   friend class GeneralMatrix;
   friend class GenericMatrix;
public:
   SubtractedMatrix() : AddedMatrix() {}
   ~SubtractedMatrix() {}
   GeneralMatrix* Evaluate(MatrixType mt=MatrixTypeUnSp);
   NEW_DELETE(SubtractedMatrix)
};

/// Any type of matrix plus Real.
/// \internal
class ShiftedMatrix : public BaseMatrix
{
protected:
   union { const BaseMatrix* bm; GeneralMatrix* gm; };
   Real f;
   ShiftedMatrix(const BaseMatrix* bmx, Real fx) : bm(bmx),f(fx) {}
   int search(const BaseMatrix*) const;
   friend class BaseMatrix;
   friend class GeneralMatrix;
   friend class GenericMatrix;
public:
   ShiftedMatrix() : BaseMatrix(), bm(0) {}
   ~ShiftedMatrix() {}
   GeneralMatrix* Evaluate(MatrixType mt=MatrixTypeUnSp);
   friend ShiftedMatrix operator+(Real f, const BaseMatrix& BM);
   NEW_DELETE(ShiftedMatrix)
};

/// Real minus matrix.
/// \internal
class NegShiftedMatrix : public ShiftedMatrix
{
protected:
   NegShiftedMatrix(Real fx, const BaseMatrix* bmx) : ShiftedMatrix(bmx,fx) {}
   friend class BaseMatrix;
   friend class GeneralMatrix;
   friend class GenericMatrix;
public:
   NegShiftedMatrix() : ShiftedMatrix() {}
   ~NegShiftedMatrix() {}
   GeneralMatrix* Evaluate(MatrixType mt=MatrixTypeUnSp);
   friend NegShiftedMatrix operator-(Real, const BaseMatrix&);
   NEW_DELETE(NegShiftedMatrix)
};

/// Any type of matrix times Real.
/// \internal
class ScaledMatrix : public ShiftedMatrix
{
   ScaledMatrix(const BaseMatrix* bmx, Real fx) : ShiftedMatrix(bmx,fx) {}
   friend class BaseMatrix;
   friend class GeneralMatrix;
   friend class GenericMatrix;
public:
   ScaledMatrix() : ShiftedMatrix() {}
   ~ScaledMatrix() {}
   GeneralMatrix* Evaluate(MatrixType mt=MatrixTypeUnSp);
   MatrixBandWidth bandwidth() const;
   friend ScaledMatrix operator*(Real f, const BaseMatrix& BM);
   NEW_DELETE(ScaledMatrix)
};

/// Any type of matrix times -1.
/// \internal
class NegatedMatrix : public BaseMatrix
{
protected:
   union { const BaseMatrix* bm; GeneralMatrix* gm; };
   NegatedMatrix(const BaseMatrix* bmx) : bm(bmx) {}
   int search(const BaseMatrix*) const;
private:
   friend class BaseMatrix;
public:
   NegatedMatrix() : BaseMatrix(), bm(0) {}
   ~NegatedMatrix() {}
   GeneralMatrix* Evaluate(MatrixType mt=MatrixTypeUnSp);
   MatrixBandWidth bandwidth() const;
   NEW_DELETE(NegatedMatrix)
};

/// Transposed matrix.
/// \internal
class TransposedMatrix : public NegatedMatrix
{
   TransposedMatrix(const BaseMatrix* bmx) : NegatedMatrix(bmx) {}
   friend class BaseMatrix;
public:
   TransposedMatrix() : NegatedMatrix() {}
   ~TransposedMatrix() {}
   GeneralMatrix* Evaluate(MatrixType mt=MatrixTypeUnSp);
   MatrixBandWidth bandwidth() const;
   NEW_DELETE(TransposedMatrix)
};

/// Any type of matrix with order of elements reversed.
/// \internal
class ReversedMatrix : public NegatedMatrix
{
   ReversedMatrix(const BaseMatrix* bmx) : NegatedMatrix(bmx) {}
   friend class BaseMatrix;
public:
   ReversedMatrix() : NegatedMatrix() {}
   ~ReversedMatrix() {}
   GeneralMatrix* Evaluate(MatrixType mt=MatrixTypeUnSp);
   NEW_DELETE(ReversedMatrix)
};

/// Inverse of matrix.
/// \internal
class InvertedMatrix : public NegatedMatrix
{
   InvertedMatrix(const BaseMatrix* bmx) : NegatedMatrix(bmx) {}
public:
   InvertedMatrix() : NegatedMatrix() {}
   ~InvertedMatrix() {}
   SolvedMatrix operator*(const BaseMatrix&) const;       // inverse(A) * B
   ScaledMatrix operator*(Real t) const { return BaseMatrix::operator*(t); }
   friend class BaseMatrix;
   GeneralMatrix* Evaluate(MatrixType mt=MatrixTypeUnSp);
   MatrixBandWidth bandwidth() const;
   NEW_DELETE(InvertedMatrix)
};

/// Any type of matrix interpreted as a RowVector.
/// \internal
class RowedMatrix : public NegatedMatrix
{
   RowedMatrix(const BaseMatrix* bmx) : NegatedMatrix(bmx) {}
   friend class BaseMatrix;
public:
   RowedMatrix() : NegatedMatrix() {}
   ~RowedMatrix() {}
   GeneralMatrix* Evaluate(MatrixType mt=MatrixTypeUnSp);
   MatrixBandWidth bandwidth() const;
   NEW_DELETE(RowedMatrix)
};

/// Any type of matrix interpreted as a ColumnVector.
/// \internal
class ColedMatrix : public NegatedMatrix
{
   ColedMatrix(const BaseMatrix* bmx) : NegatedMatrix(bmx) {}
   friend class BaseMatrix;
public:
   ColedMatrix() : NegatedMatrix() {}
   ~ColedMatrix() {}
   GeneralMatrix* Evaluate(MatrixType mt=MatrixTypeUnSp);
   MatrixBandWidth bandwidth() const;
   NEW_DELETE(ColedMatrix)
};

/// Any type of matrix interpreted as a DiagonalMatrix.
/// \internal
class DiagedMatrix : public NegatedMatrix
{
   DiagedMatrix(const BaseMatrix* bmx) : NegatedMatrix(bmx) {}
   friend class BaseMatrix;
public:
   DiagedMatrix() : NegatedMatrix() {}
   ~DiagedMatrix() {}
   GeneralMatrix* Evaluate(MatrixType mt=MatrixTypeUnSp);
   MatrixBandWidth bandwidth() const;
   NEW_DELETE(DiagedMatrix)
};

/// Any type of matrix interpreted as a (rectangular) Matrix.
/// \internal
class MatedMatrix : public NegatedMatrix
{
   int nr, nc;
   MatedMatrix(const BaseMatrix* bmx, int nrx, int ncx)
      : NegatedMatrix(bmx), nr(nrx), nc(ncx) {}
   friend class BaseMatrix;
public:
   MatedMatrix() : NegatedMatrix() {}
   ~MatedMatrix() {}
   GeneralMatrix* Evaluate(MatrixType mt=MatrixTypeUnSp);
   MatrixBandWidth bandwidth() const;
   NEW_DELETE(MatedMatrix)
};

/// A matrix in an "envelope' for return from a function.
/// \internal
class ReturnMatrix : public BaseMatrix
{
   GeneralMatrix* gm;
   int search(const BaseMatrix*) const;
public:
   ReturnMatrix() : BaseMatrix(), gm(0) {}
   ~ReturnMatrix() {}
   GeneralMatrix* Evaluate(MatrixType mt=MatrixTypeUnSp);
   friend class BaseMatrix;
   ReturnMatrix(const ReturnMatrix& tm) : BaseMatrix(), gm(tm.gm) {}
   ReturnMatrix(const GeneralMatrix* gmx) : gm((GeneralMatrix*&)gmx) {}
//   ReturnMatrix(GeneralMatrix&);
   MatrixBandWidth bandwidth() const;
   NEW_DELETE(ReturnMatrix)
};


#endif

