#ifndef __NEW_MAT_ARRAY_H__
#define __NEW_MAT_ARRAY_H__

// **************** a very simple integer array class ********************/

/// A very simple integer array class.
/// A minimal array class to imitate a C style array but giving dynamic storage
/// mostly intended for internal use by newmat.
/// Probably to be replaced by a templated class when I start using templates.

class SimpleIntArray : public Janitor
{
protected:
   int* a;                    ///< pointer to the array
   int n;                     ///< length of the array
public:
   SimpleIntArray(int xn);    ///< build an array length xn
   SimpleIntArray() : a(0), n(0) {}  ///< build an array length 0
   ~SimpleIntArray();         ///< return the space to memory
   int& operator[](int i);    ///< access element of the array - start at 0
   int operator[](int i) const;
			      ///< access element of constant array
   void operator=(int ai);    ///< set the array equal to a constant
   void operator=(const SimpleIntArray& b);
			      ///< copy the elements of an array
   SimpleIntArray(const SimpleIntArray& b);
			      ///< make a new array equal to an existing one
   int Size() const { return n; }
			      ///< return the size of the array
   int size() const { return n; }
			      ///< return the size of the array
   int* Data() { return a; }  ///< pointer to the data
   const int* Data() const { return a; }  ///< pointer to the data
   int* data() { return a; }  ///< pointer to the data
   const int* data() const { return a; }  ///< pointer to the data
   const int* const_data() const { return a; }  ///< pointer to the data
   void resize(int i, bool keep = false);
                              ///< change length, keep data if keep = true
   void ReSize(int i, bool keep = false) { resize(i, keep); }
                              ///< change length, keep data if keep = true
   void resize_keep(int i) { resize(i, true); }
                              ///< change length, keep data
   void cleanup() { resize(0); }   ///< set length to zero
   void CleanUp() { resize(0); }   ///< set length to zero
   NEW_DELETE(SimpleIntArray)
};

// ********************** C subscript classes ****************************

/// Let matrix simulate a C type two dimensional array
class RealStarStar
{
   Real** a;
public:
   RealStarStar(Matrix& A);
   ~RealStarStar() { delete [] a; }
   operator Real**() { return a; }
};

/// Let matrix simulate a C type const two dimensional array
class ConstRealStarStar
{
   const Real** a;
public:
   ConstRealStarStar(const Matrix& A);
   ~ConstRealStarStar() { delete [] a; }
   operator const Real**() { return a; }
};


#endif

