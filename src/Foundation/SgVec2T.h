// Copyright 2015 Autodesk, Inc. All rights reserved. 
//
// Use of this software is subject to the terms of the Autodesk 
// license agreement provided at the time of installation or download, 
// or which otherwise accompanies this software in either electronic 
// or hard copy form.

/**
 * @file SgVec2T.h
 * @brief Contains the declaration of the class SgVec2T.
 *
 */

#ifndef SGVEC2T_H
#define SGVEC2T_H

#include <iostream>
#include <math.h>

/**
 * @brief A 2d point/vector class.
 *
 * This class represents 2d points and vectors of Ts. In reality, this
 * is a vector with the base point at the global origin. Why? Because
 * you cant really add points, subtract points, and so forth -- at least
 * mathematically.
 */
template <typename T>
class SgVec2T
{
public:

    /** Default constructor. */
    SgVec2T()
        { _vec[0] = _vec[1] = 0; }

    /** Scalar constructor. */
    explicit SgVec2T( T v )
        { setValue( v, v ); }

    /** Component constructor. */
    SgVec2T( T x, T y )
        { setValue( x, y ); }

    /** Array constructor. */
    explicit SgVec2T( const double v[2] )
        { setValue( v[0], v[1] ); }

    /** Array constructor. */
    explicit SgVec2T( const float v[2] )
        { setValue( v[0], v[1] ); }

    /** Copy constructor. */
    /** Change of type copy constructor. */
    SgVec2T( const SgVec2T<float> &v )
    { _vec[0]=T(v[0]); _vec[1]=T(v[1]); }
    SgVec2T( const SgVec2T<double> &v )
    { _vec[0]=T(v[0]); _vec[1]=T(v[1]); }

    /* Destructor. */
    ~SgVec2T() {;}

    /* Accesses indexed component of vector */
    T       &operator []( int i )          { return (_vec[i]); }
    const T &operator []( int i ) const    { return (_vec[i]); }

    /** Vector assignment. */
    SgVec2T<T> &operator =( const SgVec2T<T> &v )
        { _vec[0]=v[0]; _vec[1]=v[1]; return *this; }

    /** Constant assignment. */
    SgVec2T<T> &operator =( T v )
        { _vec[0]=v; _vec[1]=v; return *this; }

    /** Component-wise scalar multiplication. */
    SgVec2T<T> &operator *=( T d )
        { _vec[0]*=d; _vec[1]*=d; return *this; }

    /** Component-wise scalar division. */
    SgVec2T<T> &operator /=( T d )
        { _vec[0]/=d; _vec[1]/=d; return *this; }

    /** Component-wise vector addition. */
    SgVec2T<T> &operator +=( const SgVec2T<T> &v )
        { _vec[0]+=v[0]; _vec[1]+=v[1]; return *this; } 

    /** Component-wise vector subtraction. */
    SgVec2T<T> &operator -=( const SgVec2T<T> &v )
        { _vec[0]-=v[0]; _vec[1]-=v[1]; return *this; } 
    
    /** Nondestructive unary negation - returns a new vector. */
    SgVec2T<T> operator -() const
        { return SgVec2T<T>( -_vec[0], -_vec[1] ); }

    /** Equality comparison. */
    friend bool operator ==( const SgVec2T<T> &v1, const SgVec2T<T> &v2 )
        { return (v1[0] == v2[0] && v1[1] == v2[1]); }

    /** Inequality comparison. */
    friend bool operator !=( const SgVec2T<T> &v1, const SgVec2T<T> &v2 )
        { return !(v1 == v2); }

    /** Component-wise binary scalar multiplication. */
    friend SgVec2T<T> operator *( const SgVec2T<T> &v, T d )
        { return SgVec2T<T>( v[0]*d, v[1]*d ); }
    
    /** Component-wise binary scalar multiplication. */
    friend SgVec2T<T> operator *( T d, const SgVec2T<T> &v )
        { return v * d; }
    
    /** Cross product. */
    friend T operator *( const SgVec2T<T> &v1, const SgVec2T<T> &v2 )
        { return v1[0] * v2[1] - v1[1] * v2[0]; }
    
    /** Component-wise binary scalar division operators. */
    friend SgVec2T<T> operator /( const SgVec2T<T> &v, T d )
        { return SgVec2T<T>( v[0]/d, v[1]/d ); }

    /** Component-wise binary vector addition. */
    friend SgVec2T<T> operator +( const SgVec2T<T> &v1, const SgVec2T<T> &v2 )
        { return SgVec2T<T>( v1[0]+v2[0], v1[1]+v2[1] ); }

    /** Component-wise binary vector subtraction. */
    friend SgVec2T<T> operator -( const SgVec2T<T> &v1, const SgVec2T<T> &v2 )
        { return SgVec2T<T>( v1[0]-v2[0], v1[1]-v2[1] ); }

    /** Output a formatted string for the vector to a stream. */
    friend std::ostream & operator <<( std::ostream  &os,
                                       const SgVec2T<T> &v )
        { os << v[0] << " " << v[1]; return os; }

    /** Input a vector from a formatted string. */
    friend std::istream & operator >>( std::istream  &is,
                                       SgVec2T<T>     &v )
        { is >> v[0] >> v[1]; return is; }

    /** Get coordinates. */
    void getValue( T &x, T &y ) const
        { x = _vec[0]; y = _vec[1]; }
    
    /** Get coordinates as array. */
    const T *getValue() const
        { return _vec; }

    /** Set coordinates. */
    void setValue( T x, T y )
        { _vec[0] = x; _vec[1] = y; }
    
    /** Set coordinates as array. */
    void setValue( const T v[2] )
        { _vec[0] = v[0]; _vec[1] = v[1]; }

    /** Inner product. */
    T dot( const SgVec2T<T> &v ) const
        { return _vec[0]*v[0] + _vec[1]*v[1]; }

    /** Negate vector. */
    void negate()
        { _vec[0]*=-1; _vec[1]*=-1; }

    /** Length of vector. */
    T length() const
        { return sqrt(_vec[0]*_vec[0] + _vec[1]*_vec[1]); }

    /** Normalize vector. */
    T normalize()
        { T len = length();
          if ( len < T(0.000001) ) { setValue(T(0.0),T(0.0)); len = T(0.0); }
          else { T fact = T(1.0) / len; *this *= fact; } return len; }

    /** A normalized copy of the vector. */
    SgVec2T<T> normalized() const
        { T len = length();
          if ( len < T(0.000001) ) { return SgVec2T<T>(T(0.0),T(0.0)); }
          else { T fact = T(1.0) / len; return *this * fact; } }

    /** Return a vector orthogonal to the current vector. */
    SgVec2T<T> orthogonal() const {
        return SgVec2T<T>(-_vec[1], _vec[0]);
    }

    /**
     * Returns the angle in radians between the current vector and the
     * passed in vector.
     */
    T angle( const SgVec2T<T> &v ) const
        { double tmp=(double)this->dot(v)/(double)(this->length()*v.length());
            return static_cast<T>(acos(std::max(-1.0,std::min(tmp,1.0)))); }

private:

    /** Coordinates. */
    T  _vec[2];
};

typedef SgVec2T<double> SgVec2d;
typedef SgVec2T<float>  SgVec2f;

#endif
