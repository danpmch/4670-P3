///////////////////////////////////////////////////////////////////////////
//
// NAME
//  Transform.h -- coordinate transformation matrices
//
// DESCRIPTION
//  These classes are used to implement 3x3 and 4x4 coordinate transformation
//  matrices.  Basic operations supported are:
//   . multiplication
//   . inverses
//   . translation and rotation
//
// SEE ALSO
//  Transform.cpp       implementation
//
// Copyright � Richard Szeliski, 2001.  See Copyright.h for more details
// (modified for CSE576 Spring 2005)
//
///////////////////////////////////////////////////////////////////////////

#include <cstdio>

class CVector3
{
public:
	CVector3();                         // default constructor (zero)
	CVector3(double x, double y, double z) { m_array[0] = x; m_array[1] = y; m_array[2] = z; }
	double &operator[](int i);          // element access
	const double &operator[](int i) const;    // element access
private:
	double m_array[3];
};

inline double &CVector3::operator[](int i)
{
    return m_array[i];
}

inline const double &CVector3::operator[](int i) const
{
    return m_array[i];
}

class CTransform3x3
{
public:
    CTransform3x3();                    // default constructor (identity)
    static CTransform3x3 Translation(float tx, float ty);
    static CTransform3x3 Rotation(float degrees);
	static CTransform3x3 Scale(float s);
    CTransform3x3 Inverse(void  );      // matrix inverse
	CVector3 operator*(const CVector3& v) const;
    CTransform3x3 operator*(const CTransform3x3& m);
    double* operator[](int i);          // access the elements
    const double* operator[](int i) const;    // access the elements
    void print();
private:
    double m_array[3][3];               // data array
};


inline void CTransform3x3::print()
{
  for( int row = 0; row < 3; row++ )
  {
    for( int col = 0; col < 3; col++ )
    {
      printf( "%f, ", m_array[ row ][ col ] );
    }
  }
}

inline double* CTransform3x3::operator[](int i)
{
    return m_array[i];
}

inline const double* CTransform3x3::operator[](int i) const
{
    return m_array[i];
}
