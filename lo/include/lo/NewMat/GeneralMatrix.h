#ifndef __NEW_MAT_GENERAL_MATRIX_H__
#define __NEW_MAT_GENERAL_MATRIX_H__

/// The classes for matrices that can contain data are derived from this.
class GeneralMatrix : public BaseMatrix         // declarable matrix types
{
   virtual GeneralMatrix* Image() const;        // copy of matrix
protected:
   int tag_val;                                 // shows whether can reuse
   int nrows_val, ncols_val;                    // dimensions
   int storage;                                 // total store required
   Real* store;                                 // point to store (0=not set)
   GeneralMatrix();                             // initialise with no store
   GeneralMatrix(ArrayLengthSpecifier);         // constructor getting store
   void Add(GeneralMatrix*, Real);              // sum of GM and Real
   void Add(Real);                              // add Real to this
   void NegAdd(GeneralMatrix*, Real);           // Real - GM
   void NegAdd(Real);                           // this = this - Real
   void Multiply(GeneralMatrix*, Real);         // product of GM and Real
   void Multiply(Real);                         // multiply this by Real
   void Negate(GeneralMatrix*);                 // change sign
   void Negate();                               // change sign
   void ReverseElements();                      // internal reverse of elements
   void ReverseElements(GeneralMatrix*);        // reverse order of elements
   void operator=(Real);                        // set matrix to constant
   Real* GetStore();                            // get store or copy
   GeneralMatrix* BorrowStore(GeneralMatrix*, MatrixType);
                                                // temporarily access store
   void GetMatrix(const GeneralMatrix*);        // used by = and initialise
   void Eq(const BaseMatrix&, MatrixType);      // used by =
   void Eq(const GeneralMatrix&);               // version with no conversion
   void Eq(const BaseMatrix&, MatrixType, bool);// used by <<
   void Eq2(const BaseMatrix&, MatrixType);     // cut down version of Eq
   int search(const BaseMatrix*) const;
   virtual GeneralMatrix* Transpose(TransposedMatrix*, MatrixType);
   void CheckConversion(const BaseMatrix&);     // check conversion OK
   void resize(int, int, int);                  // change dimensions
   virtual short SimpleAddOK(const GeneralMatrix*) { return 0; }
             // see bandmat.cpp for explanation
   virtual void MiniCleanUp()
      { store = 0; storage = 0; nrows_val = 0; ncols_val = 0; tag_val = -1;}
             // CleanUp when the data array has already been deleted
   void PlusEqual(const GeneralMatrix& gm);
   void MinusEqual(const GeneralMatrix& gm);
   void PlusEqual(Real f);
   void MinusEqual(Real f);
   void swap(GeneralMatrix& gm);                // swap values
public:
   GeneralMatrix* Evaluate(MatrixType mt=MatrixTypeUnSp);
   virtual MatrixType type() const = 0;         // type of a matrix
   MatrixType Type() const { return type(); }
   int Nrows() const { return nrows_val; }      // get dimensions
   int Ncols() const { return ncols_val; }
   int Storage() const { return storage; }
   Real* Store() const { return store; }
   // updated names
   int nrows() const { return nrows_val; }      // get dimensions
   int ncols() const { return ncols_val; }
   int size() const { return storage; }
   Real* data() { return store; }
   const Real* data() const { return store; }
   const Real* const_data() const { return store; }
   virtual ~GeneralMatrix();                    // delete store if set
   void tDelete();                              // delete if tag_val permits
   bool reuse();                                // true if tag_val allows reuse
   void protect() { tag_val=-1; }               // cannot delete or reuse
   void Protect() { tag_val=-1; }               // cannot delete or reuse
   int tag() const { return tag_val; }
   int Tag() const { return tag_val; }
   bool is_zero() const;                        // test matrix has all zeros
   bool IsZero() const { return is_zero(); }    // test matrix has all zeros
   void Release() { tag_val=1; }                // del store after next use
   void Release(int t) { tag_val=t; }           // del store after t accesses
   void ReleaseAndDelete() { tag_val=0; }       // delete matrix after use
   void release() { tag_val=1; }                // del store after next use
   void release(int t) { tag_val=t; }           // del store after t accesses
   void release_and_delete() { tag_val=0; }     // delete matrix after use
   void operator<<(const double*);              // assignment from an array
   void operator<<(const float*);               // assignment from an array
   void operator<<(const int*);                 // assignment from an array
   void operator<<(const BaseMatrix& X)
      { Eq(X,this->type(),true); }              // = without checking type
   void inject(const GeneralMatrix&);           // copy stored els only
   void Inject(const GeneralMatrix& GM) { inject(GM); }
   void operator+=(const BaseMatrix&);
   void operator-=(const BaseMatrix&);
   void operator*=(const BaseMatrix&);
   void operator|=(const BaseMatrix&);
   void operator&=(const BaseMatrix&);
   void operator+=(Real);
   void operator-=(Real r) { operator+=(-r); }
   void operator*=(Real);
   void operator/=(Real r) { operator*=(1.0/r); }
   virtual GeneralMatrix* MakeSolver();         // for solving
   virtual void Solver(MatrixColX&, const MatrixColX&) {}
   virtual void GetRow(MatrixRowCol&) = 0;      // Get matrix row
   virtual void RestoreRow(MatrixRowCol&) {}    // Restore matrix row
   virtual void NextRow(MatrixRowCol&);         // Go to next row
   virtual void GetCol(MatrixRowCol&) = 0;      // Get matrix col
   virtual void GetCol(MatrixColX&) = 0;        // Get matrix col
   virtual void RestoreCol(MatrixRowCol&) {}    // Restore matrix col
   virtual void RestoreCol(MatrixColX&) {}      // Restore matrix col
   virtual void NextCol(MatrixRowCol&);         // Go to next col
   virtual void NextCol(MatrixColX&);           // Go to next col
   Real sum_square() const;
   Real sum_absolute_value() const;
   Real sum() const;
   Real maximum_absolute_value1(int& i) const;
   Real minimum_absolute_value1(int& i) const;
   Real maximum1(int& i) const;
   Real minimum1(int& i) const;
   Real maximum_absolute_value() const;
   Real maximum_absolute_value2(int& i, int& j) const;
   Real minimum_absolute_value() const;
   Real minimum_absolute_value2(int& i, int& j) const;
   Real maximum() const;
   Real maximum2(int& i, int& j) const;
   Real minimum() const;
   Real minimum2(int& i, int& j) const;
   LogAndSign log_determinant() const;
   virtual bool IsEqual(const GeneralMatrix&) const;
                                                // same type, same values
   void CheckStore() const;                     // check store is non-zero
   virtual void SetParameters(const GeneralMatrix*) {}
                                                // set parameters in GetMatrix
   operator ReturnMatrix() const;               // for building a ReturnMatrix
   ReturnMatrix for_return() const;
   ReturnMatrix ForReturn() const;
   //virtual bool SameStorageType(const GeneralMatrix& A) const;
   //virtual void ReSizeForAdd(const GeneralMatrix& A, const GeneralMatrix& B);
   //virtual void ReSizeForSP(const GeneralMatrix& A, const GeneralMatrix& B);
   virtual void resize(const GeneralMatrix& A);
   virtual void ReSize(const GeneralMatrix& A) { resize(A); }
   MatrixInput operator<<(double);                // for loading a list
   MatrixInput operator<<(float);                // for loading a list
   MatrixInput operator<<(int f);
//   ReturnMatrix Reverse() const;                // reverse order of elements
   void cleanup();                              // to clear store

   friend class Matrix;
   friend class SquareMatrix;
   friend class nricMatrix;
   friend class SymmetricMatrix;
   friend class UpperTriangularMatrix;
   friend class LowerTriangularMatrix;
   friend class DiagonalMatrix;
   friend class CroutMatrix;
   friend class RowVector;
   friend class ColumnVector;
   friend class BandMatrix;
   friend class LowerBandMatrix;
   friend class UpperBandMatrix;
   friend class SymmetricBandMatrix;
   friend class BaseMatrix;
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
   NEW_DELETE(GeneralMatrix)
};

#endif

