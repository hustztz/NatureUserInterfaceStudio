// Copyright 2014 Autodesk, Inc. All rights reserved. 
//
// Use of this software is subject to the terms of the Autodesk 
// license agreement provided at the time of installation or download, 
// or which otherwise accompanies this software in either electronic 
// or hard copy form.

/**
 * @file SgVec3T.h
 * @brief Contains the declaration of the class SgVec3T.
 *
 * <b>CONFIDENTIAL INFORMATION: This software is the confidential and
 * proprietary information of Walt Disney Animation Studios ("WDAS").
 * This software may not be used, disclosed, reproduced or distributed
 * for any purpose without prior written authorization and license
 * from WDAS. Reproduction of any section of this software must include
 * this legend and all copyright notices.
 * Copyright Disney Enterprises, Inc. All rights reserved.</b>
 *
 * @author Ernie Petti
 * @author Thomas V Thompson II
 * @author Stephen D. Bowline
 *
 * @version Created 05/15/02
 */

#ifndef SGVEC3T_H
#define SGVEC3T_H

#include <iostream>
#include <math.h>

/**
 * @brief A 3d point/vector class.
 *
 * This class represents 3d points and vectors of Ts. In reality, this
 * is a vector with the base point at the global origin. Why? Because
 * you cant really add points, subtract points, and so forth -- at least
 * mathematically.
 */
template <typename T>
class SgVec3T
{
public:

    /** Default constructor. */
    SgVec3T()
        { _vec[0] = _vec[1] = _vec[2] = 0; }

    /** Scalar constructor. */
    explicit SgVec3T( T v )
        { setValue( v, v, v ); }

    /** Component constructor. */
    SgVec3T( T x, T y, T z )
        { setValue( x, y, z ); }

    /** Array constructor. */
    explicit SgVec3T( const double v[3] )
        { setValue( v[0], v[1], v[2] ); }

    /** Array constructor. */
    explicit SgVec3T( const float v[3] )
        { setValue( v[0], v[1], v[2] ); }

    /** Copy constructor. */
    /** Change of type copy constructor. */
    SgVec3T( const SgVec3T<float> &v )
    { _vec[0]=T(v[0]); _vec[1]=T(v[1]); _vec[2]=T(v[2]); }
    SgVec3T( const SgVec3T<double> &v )
    { _vec[0]=T(v[0]); _vec[1]=T(v[1]); _vec[2]=T(v[2]); }

    /* Destructor. */
    ~SgVec3T() {;}

    /* Accesses indexed component of vector */
    T       &operator []( int i )          { return (_vec[i]); }
    const T &operator []( int i ) const    { return (_vec[i]); }

    /** Vector assignment. */
    SgVec3T<T> &operator =( const SgVec3T<T> &v )
        { _vec[0]=v[0]; _vec[1]=v[1]; _vec[2]=v[2]; return *this; }

    /** Constant assignment. */
    SgVec3T<T> &operator =( T v )
        { _vec[0]=v; _vec[1]=v; _vec[2]=v; return *this; }

    /** Component-wise scalar multiplication. */
    SgVec3T<T> &operator *=( T d )
        { _vec[0]*=d; _vec[1]*=d; _vec[2]*=d; return *this; }

    /** Component-wise scalar division. */
    SgVec3T<T> &operator /=( T d )
        { _vec[0]/=d; _vec[1]/=d; _vec[2]/=d; return *this; }

    /** Component-wise vector addition. */
    SgVec3T<T> &operator +=( const SgVec3T<T> &v )
        { _vec[0]+=v[0]; _vec[1]+=v[1]; _vec[2]+=v[2]; return *this; } 

    /** Component-wise vector subtraction. */
    SgVec3T<T> &operator -=( const SgVec3T<T> &v )
        { _vec[0]-=v[0]; _vec[1]-=v[1]; _vec[2]-=v[2]; return *this; } 
    
    /** Nondestructive unary negation - returns a new vector. */
    SgVec3T<T> operator -() const
        { return SgVec3T<T>( -_vec[0], -_vec[1], -_vec[2] ); }

    /** Equality comparison. */
    friend bool operator ==( const SgVec3T<T> &v1, const SgVec3T<T> &v2 )
        { return (v1[0] == v2[0] && v1[1] == v2[1] && v1[2] == v2[2]); }

    /** Inequality comparison. */
    friend bool operator !=( const SgVec3T<T> &v1, const SgVec3T<T> &v2 )
        { return !(v1 == v2); }

    /** Component-wise binary scalar multiplication. */
    friend SgVec3T<T> operator *( const SgVec3T<T> &v, T d )
        { return SgVec3T<T>( v[0]*d, v[1]*d, v[2]*d ); }
    
    /** Component-wise binary scalar multiplication. */
    friend SgVec3T<T> operator *( T d, const SgVec3T<T> &v )
        { return v * d; }
    
    /** Cross product. */
    friend SgVec3T<T> operator *( const SgVec3T<T> &v1, const SgVec3T<T> &v2 )
        { return SgVec3T<T>(v1[1]*v2[2] - v1[2]*v2[1],
                           v1[2]*v2[0] - v1[0]*v2[2],
                           v1[0]*v2[1] - v1[1]*v2[0]); }
    
    /** Component-wise binary scalar division operators. */
    friend SgVec3T<T> operator /( const SgVec3T<T> &v, T d )
        { return SgVec3T<T>( v[0]/d, v[1]/d, v[2]/d ); }

    /** Component-wise binary vector addition. */
    friend SgVec3T<T> operator +( const SgVec3T<T> &v1, const SgVec3T<T> &v2 )
        { return SgVec3T<T>( v1[0]+v2[0], v1[1]+v2[1], v1[2]+v2[2] ); }

    /** Component-wise binary vector subtraction. */
    friend SgVec3T<T> operator -( const SgVec3T<T> &v1, const SgVec3T<T> &v2 )
        { return SgVec3T<T>( v1[0]-v2[0], v1[1]-v2[1], v1[2]-v2[2]); }

    /** Output a formatted string for the vector to a stream. */
    friend std::ostream & operator <<( std::ostream  &os,
                                       const SgVec3T<T> &v )
        { os << v[0] << " " << v[1] << " " << v[2]; return os; }

    /** Input a vector from a formatted string. */
    friend std::istream & operator >>( std::istream  &is,
                                       SgVec3T<T>     &v )
        { is >> v[0] >> v[1] >> v[2]; return is; }

    /** Get coordinates. */
    void getValue( T &x, T &y, T &z ) const
        { x = _vec[0]; y = _vec[1]; z = _vec[2]; }
    
    /** Get coordinates as array. */
    const T *getValue() const
        { return _vec; }

    /** Set coordinates. */
    void setValue( T x, T y, T z )
        { _vec[0] = x; _vec[1] = y; _vec[2] = z; }
    
    /** Set coordinates as array. */
    void setValue( const T v[3] )
        { _vec[0] = v[0]; _vec[1] = v[1]; _vec[2] = v[2]; }

    /** Inner product. */
    T dot( const SgVec3T<T> &v ) const
        { return _vec[0]*v[0] + _vec[1]*v[1] + _vec[2]*v[2]; }

    /** Negate vector. */
    void negate()
        { _vec[0]*=-1; _vec[1]*=-1; _vec[2]*=-1; }

    /** Length of vector. */
    T length() const
        { return sqrt(_vec[0]*_vec[0] + _vec[1]*_vec[1] +_vec[2]*_vec[2]); }

    /** Normalize vector. */
    T normalize()
        { T len = length();
          if ( len < T(0.000001) ) { setValue(T(0.0),T(0.0),T(0.0)); len = T(0.0); }
          else { T fact = T(1.0) / len; *this *= fact; } return len; }

    /** A normalized copy of the vector. */
    SgVec3T<T> normalized() const
        { T len = length();
          if ( len < T(0.000001) ) { return SgVec3T<T>(T(0.0),T(0.0),T(0.0)); }
          else { T fact = T(1.0) / len; return *this * fact; } }

    /** Return a vector orthogonal to the current vector. */
    SgVec3T<T> orthogonal() const {
        return SgVec3T<T>(_vec[1]+_vec[2], _vec[2]-_vec[0], -_vec[0]-_vec[1]);
    }

    /**
     * Returns the angle in radians between the current vector and the
     * passed in vector.
     */
    T angle( const SgVec3T<T> &v ) const
        { double tmp=(double)this->dot(v)/(double)(this->length()*v.length());
            return static_cast<T>(acos(std::max(-1.0,std::min(tmp,1.0)))); }

    /**
     * Returns the vector rotated by the angle given in radians about
     * the given axis.
     * NOTE: This uses a lot of matrix and quaternion math.  If we intend to
     * start doing more than this simple rotation calculation, it may benefit
     * to implement an SgMatrix and SgQuaternion class.
     */
    SgVec3T<T> rotateBy( const SgVec3T<T> &axis, double angle ) const
        {
            T ca = static_cast<T>(cos(angle)),
              sa = static_cast<T>(sin(angle)),
              ca1 = (1-ca);
            T a01 = axis[0]*axis[1];
            T a02 = axis[0]*axis[2];
            T a12 = axis[1]*axis[2];
            T a00 = axis[0]*axis[0];
            T a11 = axis[1]*axis[1];
            T a22 = axis[2]*axis[2];
            return SgVec3T<T>( _vec[0]*(a00*ca1+ca) +
                              _vec[1]*(a01*ca1-axis[2]*sa) +
                              _vec[2]*(a02*ca1+axis[1]*sa),
                            
                              _vec[0]*(a01*ca1+axis[2]*sa) +
                              _vec[1]*(a11*ca1+ca)+
                              _vec[2]*(a12*ca1-axis[0]*sa),
                               
                              _vec[0]*(a02*ca1-axis[1]*sa) +
                              _vec[1]*(a12*ca1+axis[0]*sa) +
                              _vec[2]*(a22*ca1+ca) );
        }
    
private:

    /** Coordinates. */
    T  _vec[3];
};

typedef SgVec3T<double> SgVec3d;
typedef SgVec3T<float>  SgVec3f;

#endif
