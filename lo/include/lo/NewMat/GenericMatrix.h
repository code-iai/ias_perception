#ifndef __NEW_MAT_GENERIC_MATRIX_H__
#define __NEW_MAT_GENERIC_MATRIX_H__

// ************************** GenericMatrix class ************************/

/// A matrix which can be of any GeneralMatrix type.
class GenericMatrix : public BaseMatrix
{
   GeneralMatrix* gm;
   int search(const BaseMatrix* bm) const;
   friend class BaseMatrix;
public:
   GenericMatrix() : gm(0) {}
   GenericMatrix(const BaseMatrix& bm)
      { gm = ((BaseMatrix&)bm).Evaluate(); gm = gm->Image(); }
   GenericMatrix(const GenericMatrix& bm) : BaseMatrix()
      { gm = bm.gm->Image(); }
   void operator=(const GenericMatrix&);
   void operator=(const BaseMatrix&);
   void operator+=(const BaseMatrix&);
   void operator-=(const BaseMatrix&);
   void operator*=(const BaseMatrix&);
   void operator|=(const BaseMatrix&);
   void operator&=(const BaseMatrix&);
   void operator+=(Real);
   void operator-=(Real r) { operator+=(-r); }
   void operator*=(Real);
   void operator/=(Real r) { operator*=(1.0/r); }
   ~GenericMatrix() { delete gm; }
   void cleanup() { delete gm; gm = 0; }
   void Release() { gm->Release(); }
   void release() { gm->release(); }
   GeneralMatrix* Evaluate(MatrixType = MatrixTypeUnSp);
   MatrixBandWidth bandwidth() const;
   void swap(GenericMatrix& x);
   NEW_DELETE(GenericMatrix)
};

#endif

