#ifndef __NEW_MAT_BAND_MATRIX_H__
#define __NEW_MAT_BAND_MATRIX_H__

// ***************************** band matrices ***************************/

/// Band matrix.
class BandMatrix : public GeneralMatrix
{
   GeneralMatrix* Image() const;                // copy of matrix
protected:
   void CornerClear() const;                    // set unused elements to zero
   short SimpleAddOK(const GeneralMatrix* gm);
public:
   int lower_val, upper_val;                            // band widths
   BandMatrix() { lower_val=0; upper_val=0; CornerClear(); }
   ~BandMatrix() {}
   BandMatrix(int n,int lb,int ub) { resize(n,lb,ub); CornerClear(); }
                                                // standard declaration
   BandMatrix(const BaseMatrix&);               // evaluate BaseMatrix
   void operator=(const BaseMatrix&);
   void operator=(Real f) { GeneralMatrix::operator=(f); }
   void operator=(const BandMatrix& m) { Eq(m); }
   MatrixType type() const;
   Real& operator()(int, int);                  // access element
   Real& element(int, int);                     // access element
   Real operator()(int, int) const;             // access element
   Real element(int, int) const;                // access element
#ifdef SETUP_C_SUBSCRIPTS
   Real* operator[](int m) { return store+(upper_val+lower_val)*m+lower_val; }
   const Real* operator[](int m) const
      { return store+(upper_val+lower_val)*m+lower_val; }
#endif
   BandMatrix(const BandMatrix& gm) : GeneralMatrix() { GetMatrix(&gm); }
   LogAndSign log_determinant() const;
   GeneralMatrix* MakeSolver();
   Real trace() const;
   Real sum_square() const
      { CornerClear(); return GeneralMatrix::sum_square(); }
   Real sum_absolute_value() const
      { CornerClear(); return GeneralMatrix::sum_absolute_value(); }
   Real sum() const
      { CornerClear(); return GeneralMatrix::sum(); }
   Real maximum_absolute_value() const
      { CornerClear(); return GeneralMatrix::maximum_absolute_value(); }
   Real minimum_absolute_value() const
      { int i, j; return GeneralMatrix::minimum_absolute_value2(i, j); }
   Real maximum() const { int i, j; return GeneralMatrix::maximum2(i, j); }
   Real minimum() const { int i, j; return GeneralMatrix::minimum2(i, j); }
   void GetRow(MatrixRowCol&);
   void GetCol(MatrixRowCol&);
   void GetCol(MatrixColX&);
   void RestoreCol(MatrixRowCol&);
   void RestoreCol(MatrixColX& c) { RestoreCol((MatrixRowCol&)c); }
   void NextRow(MatrixRowCol&);
   virtual void resize(int, int, int);             // change dimensions
   virtual void ReSize(int m, int n, int b) { resize(m, n, b); }
   void resize(const GeneralMatrix& A);
   void ReSize(const GeneralMatrix& A) { resize(A); }
   //bool SameStorageType(const GeneralMatrix& A) const;
   //void ReSizeForAdd(const GeneralMatrix& A, const GeneralMatrix& B);
   //void ReSizeForSP(const GeneralMatrix& A, const GeneralMatrix& B);
   MatrixBandWidth bandwidth() const;
   void SetParameters(const GeneralMatrix*);
   MatrixInput operator<<(double);                // will give error
   MatrixInput operator<<(float);                // will give error
   MatrixInput operator<<(int f);
   void operator<<(const double* r);              // will give error
   void operator<<(const float* r);              // will give error
   void operator<<(const int* r);               // will give error
      // the next is included because Zortech and Borland
      // cannot find the copy in GeneralMatrix
   void operator<<(const BaseMatrix& X) { GeneralMatrix::operator<<(X); }
   void swap(BandMatrix& gm);
   NEW_DELETE(BandMatrix)
};

/// Upper triangular band matrix.
class UpperBandMatrix : public BandMatrix
{
   GeneralMatrix* Image() const;                // copy of matrix
public:
   UpperBandMatrix() {}
   ~UpperBandMatrix() {}
   UpperBandMatrix(int n, int ubw)              // standard declaration
      : BandMatrix(n, 0, ubw) {}
   UpperBandMatrix(const BaseMatrix&);          // evaluate BaseMatrix
   void operator=(const BaseMatrix&);
   void operator=(Real f) { GeneralMatrix::operator=(f); }
   void operator=(const UpperBandMatrix& m) { Eq(m); }
   MatrixType type() const;
   UpperBandMatrix(const UpperBandMatrix& gm) : BandMatrix() { GetMatrix(&gm); }
   GeneralMatrix* MakeSolver() { return this; }
   void Solver(MatrixColX&, const MatrixColX&);
   LogAndSign log_determinant() const;
   void resize(int, int, int);             // change dimensions
   void ReSize(int m, int n, int b) { resize(m, n, b); }
   void resize(int n,int ubw)              // change dimensions
      { BandMatrix::resize(n,0,ubw); }
   void ReSize(int n,int ubw)              // change dimensions
      { BandMatrix::resize(n,0,ubw); }
   void resize(const GeneralMatrix& A) { BandMatrix::resize(A); }
   void ReSize(const GeneralMatrix& A) { BandMatrix::resize(A); }
   Real& operator()(int, int);
   Real operator()(int, int) const;
   Real& element(int, int);
   Real element(int, int) const;
#ifdef SETUP_C_SUBSCRIPTS
   Real* operator[](int m) { return store+upper_val*m; }
   const Real* operator[](int m) const { return store+upper_val*m; }
#endif
   void swap(UpperBandMatrix& gm)
      { BandMatrix::swap((BandMatrix&)gm); }
   NEW_DELETE(UpperBandMatrix)
};

/// Lower triangular band matrix.
class LowerBandMatrix : public BandMatrix
{
   GeneralMatrix* Image() const;                // copy of matrix
public:
   LowerBandMatrix() {}
   ~LowerBandMatrix() {}
   LowerBandMatrix(int n, int lbw)              // standard declaration
      : BandMatrix(n, lbw, 0) {}
   LowerBandMatrix(const BaseMatrix&);          // evaluate BaseMatrix
   void operator=(const BaseMatrix&);
   void operator=(Real f) { GeneralMatrix::operator=(f); }
   void operator=(const LowerBandMatrix& m) { Eq(m); }
   MatrixType type() const;
   LowerBandMatrix(const LowerBandMatrix& gm) : BandMatrix() { GetMatrix(&gm); }
   GeneralMatrix* MakeSolver() { return this; }
   void Solver(MatrixColX&, const MatrixColX&);
   LogAndSign log_determinant() const;
   void resize(int, int, int);             // change dimensions
   void ReSize(int m, int n, int b) { resize(m, n, b); }
   void resize(int n,int lbw)             // change dimensions
      { BandMatrix::resize(n,lbw,0); }
   void ReSize(int n,int lbw)             // change dimensions
      { BandMatrix::resize(n,lbw,0); }
   void resize(const GeneralMatrix& A) { BandMatrix::resize(A); }
   void ReSize(const GeneralMatrix& A) { BandMatrix::resize(A); }
   Real& operator()(int, int);
   Real operator()(int, int) const;
   Real& element(int, int);
   Real element(int, int) const;
#ifdef SETUP_C_SUBSCRIPTS
   Real* operator[](int m) { return store+lower_val*(m+1); }
   const Real* operator[](int m) const { return store+lower_val*(m+1); }
#endif
   void swap(LowerBandMatrix& gm)
      { BandMatrix::swap((BandMatrix&)gm); }
   NEW_DELETE(LowerBandMatrix)
};

/// Symmetric band matrix.
class SymmetricBandMatrix : public GeneralMatrix
{
   GeneralMatrix* Image() const;                // copy of matrix
   void CornerClear() const;                    // set unused elements to zero
   short SimpleAddOK(const GeneralMatrix* gm);
public:
   int lower_val;                                   // lower band width
   SymmetricBandMatrix() { lower_val=0; CornerClear(); }
   ~SymmetricBandMatrix() {}
   SymmetricBandMatrix(int n, int lb) { resize(n,lb); CornerClear(); }
   SymmetricBandMatrix(const BaseMatrix&);
   void operator=(const BaseMatrix&);
   void operator=(Real f) { GeneralMatrix::operator=(f); }
   void operator=(const SymmetricBandMatrix& m) { Eq(m); }
   Real& operator()(int, int);                  // access element
   Real& element(int, int);                     // access element
   Real operator()(int, int) const;             // access element
   Real element(int, int) const;                // access element
#ifdef SETUP_C_SUBSCRIPTS
   Real* operator[](int m) { return store+lower_val*(m+1); }
   const Real* operator[](int m) const { return store+lower_val*(m+1); }
#endif
   MatrixType type() const;
   SymmetricBandMatrix(const SymmetricBandMatrix& gm)
      : GeneralMatrix() { GetMatrix(&gm); }
   GeneralMatrix* MakeSolver();
   Real sum_square() const;
   Real sum_absolute_value() const;
   Real sum() const;
   Real maximum_absolute_value() const
      { CornerClear(); return GeneralMatrix::maximum_absolute_value(); }
   Real minimum_absolute_value() const
      { int i, j; return GeneralMatrix::minimum_absolute_value2(i, j); }
   Real maximum() const { int i, j; return GeneralMatrix::maximum2(i, j); }
   Real minimum() const { int i, j; return GeneralMatrix::minimum2(i, j); }
   Real trace() const;
   LogAndSign log_determinant() const;
   void GetRow(MatrixRowCol&);
   void GetCol(MatrixRowCol&);
   void GetCol(MatrixColX&);
   void RestoreCol(MatrixRowCol&) {}
   void RestoreCol(MatrixColX&);
   GeneralMatrix* Transpose(TransposedMatrix*, MatrixType);
   void resize(int,int);                       // change dimensions
   void ReSize(int m,int b) { resize(m, b); }
   void resize(const GeneralMatrix& A);
   void ReSize(const GeneralMatrix& A) { resize(A); }
   //bool SameStorageType(const GeneralMatrix& A) const;
   //void ReSizeForAdd(const GeneralMatrix& A, const GeneralMatrix& B);
   //void ReSizeForSP(const GeneralMatrix& A, const GeneralMatrix& B);
   MatrixBandWidth bandwidth() const;
   void SetParameters(const GeneralMatrix*);
   void operator<<(const double* r);              // will give error
   void operator<<(const float* r);              // will give error
   void operator<<(const int* r);               // will give error
   void operator<<(const BaseMatrix& X) { GeneralMatrix::operator<<(X); }
   void swap(SymmetricBandMatrix& gm);
   NEW_DELETE(SymmetricBandMatrix)
};

/// LU decomposition of a band matrix.
class BandLUMatrix : public GeneralMatrix
{
   int* indx;
   bool d;
   bool sing;                                   // true if singular
   Real* store2;
   int storage2;
   int m1,m2;                                   // lower and upper
   void ludcmp();
   void get_aux(BandLUMatrix&);                 // for copying indx[] etc
   GeneralMatrix* Image() const;                // copy of matrix
public:
   BandLUMatrix(const BaseMatrix&);
   BandLUMatrix()
     : indx(0), d(true), sing(true), store2(0), storage2(0), m1(0), m2(0) {}
   BandLUMatrix(const BandLUMatrix&);
   void operator=(const BandLUMatrix&);
   GeneralMatrix* Evaluate(MatrixType mt=MatrixTypeUnSp);
   MatrixType type() const;
   void lubksb(Real*, int=0);
   ~BandLUMatrix();
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
   const Real* const_data2() const { return store2; }
   int size2() const { return storage2; }
   const int* const_data_indx() const { return indx; }
   bool even_exchanges() const { return d; }
   MatrixBandWidth bandwidth() const;
   void swap(BandLUMatrix& gm);
   NEW_DELETE(BandLUMatrix)
};

#endif

