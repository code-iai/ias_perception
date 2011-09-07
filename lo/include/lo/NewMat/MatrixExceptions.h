#ifndef __NEW_MAT_MATRIX_EXCEPTIONS_H__
#define __NEW_MAT_MATRIX_EXCEPTIONS_H__

// *************************** exceptions ********************************/

/// Not positive definite exception.
class NPDException : public Runtime_error
{
public:
   static unsigned long Select;
   NPDException(const GeneralMatrix&);
};

/// Covergence failure exception.
class ConvergenceException : public Runtime_error
{
public:
   static unsigned long Select;
   ConvergenceException(const GeneralMatrix& A);
   ConvergenceException(const char* c);
};

/// Singular matrix exception.
class SingularException : public Runtime_error
{
public:
   static unsigned long Select;
   SingularException(const GeneralMatrix& A);
};

/// Real overflow exception.
class OverflowException : public Runtime_error
{
public:
   static unsigned long Select;
   OverflowException(const char* c);
};

/// Miscellaneous exception (details in character string).
class ProgramException : public Logic_error
{
protected:
   ProgramException();
public:
   static unsigned long Select;
   ProgramException(const char* c);
   ProgramException(const char* c, const GeneralMatrix&);
   ProgramException(const char* c, const GeneralMatrix&, const GeneralMatrix&);
   ProgramException(const char* c, MatrixType, MatrixType);
};

/// Index exception.
class IndexException : public Logic_error
{
public:
   static unsigned long Select;
   IndexException(int i, const GeneralMatrix& A);
   IndexException(int i, int j, const GeneralMatrix& A);
   // next two are for access via element function
   IndexException(int i, const GeneralMatrix& A, bool);
   IndexException(int i, int j, const GeneralMatrix& A, bool);
};

/// Cannot convert to vector exception.
class VectorException : public Logic_error
{
public:
   static unsigned long Select;
   VectorException();
   VectorException(const GeneralMatrix& A);
};

/// A matrix is not square exception.
class NotSquareException : public Logic_error
{
public:
   static unsigned long Select;
   NotSquareException(const GeneralMatrix& A);
   NotSquareException();
};

/// Submatrix dimension exception.
class SubMatrixDimensionException : public Logic_error
{
public:
   static unsigned long Select;
   SubMatrixDimensionException();
};

/// Incompatible dimensions exception.
class IncompatibleDimensionsException : public Logic_error
{
public:
   static unsigned long Select;
   IncompatibleDimensionsException();
   IncompatibleDimensionsException(const GeneralMatrix&);
   IncompatibleDimensionsException(const GeneralMatrix&, const GeneralMatrix&);
};

/// Not defined exception.
class NotDefinedException : public Logic_error
{
public:
   static unsigned long Select;
   NotDefinedException(const char* op, const char* matrix);
};

/// Cannot build matrix with these properties exception.
class CannotBuildException : public Logic_error
{
public:
   static unsigned long Select;
   CannotBuildException(const char* matrix);
};


/// Internal newmat exception - shouldn't happen.
class InternalException : public Logic_error
{
public:
   static unsigned long Select;          // for identifying exception
   InternalException(const char* c);
};

#endif

