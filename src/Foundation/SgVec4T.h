// Copyright 2015 Autodesk, Inc. All rights reserved. 
//
// Use of this software is subject to the terms of the Autodesk 
// license agreement provided at the time of installation or download, 
// or which otherwise accompanies this software in either electronic 
// or hard copy form.

/**
* @file SgVec4T.h
* @brief Contains the declaration of the class SgVec4T.
*
* <b>CONFIDENTIAL INFORMATION: This software is the confidential and
* proprietary information of Walt Disney Animation Studios ("WDAS").
* This software may not be used, disclosed, reproduced or distributed
* for any purpose without prior written authorization and license
* from WDAS. Reproduction of any section of this software must include
* this legend and all copyright notices.
* Copyright Disney Enterprises, Inc. All rights reserved.</b>
*
*/

#ifndef SGVEC4T_H
#define SGVEC4T_H

#include <iostream>
#include <math.h>

template <typename T>
class SgVec4T
{
public:

    /** Default constructor. */
    SgVec4T()
    {
        _vec[0] = _vec[1] = _vec[2] = _vec[3] = 0;
    }

    /** Scalar constructor. */
    explicit SgVec4T(T v)
    {
        setValue(v, v, v, v);
    }

    /** Component constructor. */
    SgVec4T(T x, T y, T z, T w)
    {
        setValue(x, y, z, w);
    }

    /** Array constructor. */
    explicit SgVec4T(const double v[4])
    {
        setValue(v[0], v[1], v[2], v[3]);
    }

    /** Array constructor. */
    explicit SgVec4T(const float v[4])
    {
        setValue(v[0], v[1], v[2], v[3]);
    }

    /** Copy constructor. */
    /** Change of type copy constructor. */
    SgVec4T(const SgVec4T<float> &v)
    {
        _vec[0] = T(v[0]); _vec[1] = T(v[1]); _vec[2] = T(v[2]); _vec[3] = T(v[3]);
    }
    SgVec4T(const SgVec4T<double> &v)
    {
        _vec[0] = T(v[0]); _vec[1] = T(v[1]); _vec[2] = T(v[2]); _vec[3] = T(v[3]);
    }

    /* Destructor. */
    ~SgVec4T() { ; }

    /* Accesses indexed component of vector */
    T       &operator [](int i)          { return (_vec[i]); }
    const T &operator [](int i) const    { return (_vec[i]); }

    /** Vector assignment. */
    SgVec4T<T> &operator =(const SgVec4T<T> &v)
    {
        _vec[0] = v[0]; _vec[1] = v[1]; _vec[2] = v[2]; _vec[3] = v[3]; return *this;
    }

    /** Constant assignment. */
    SgVec4T<T> &operator =(T v)
    {
        _vec[0] = v; _vec[1] = v; _vec[2] = v; _vec[3] = v; return *this;
    }

    /** Component-wise scalar multiplication. */
    SgVec4T<T> &operator *=(T d)
    {
        _vec[0] *= d; _vec[1] *= d; _vec[2] *= d; _vec[3] *= d; return *this;
    }

    /** Component-wise scalar division. */
    SgVec4T<T> &operator /=(T d)
    {
        _vec[0] /= d; _vec[1] /= d; _vec[2] /= d; _vec[3] /= d; return *this;
    }

    /** Component-wise vector addition. */
    SgVec4T<T> &operator +=(const SgVec4T<T> &v)
    {
        _vec[0] += v[0]; _vec[1] += v[1]; _vec[2] += v[2]; _vec[3] += v[3]; return *this;
    }

    /** Component-wise vector subtraction. */
    SgVec4T<T> &operator -=(const SgVec4T<T> &v)
    {
        _vec[0] -= v[0]; _vec[1] -= v[1]; _vec[2] -= v[2]; _vec[3] -= v[3]; return *this;
    }

    /** Nondestructive unary negation - returns a new vector. */
    SgVec4T<T> operator -() const
    {
        return SgVec4T<T>(-_vec[0], -_vec[1], -_vec[2], -_vec[3]);
    }

    /** Equality comparison. */
    friend bool operator ==(const SgVec4T<T> &v1, const SgVec4T<T> &v2)
    {
        return (v1[0] == v2[0] && v1[1] == v2[1] && v1[2] == v2[2] && v1[3] == v2[3]);
    }

    /** Inequality comparison. */
    friend bool operator !=(const SgVec4T<T> &v1, const SgVec4T<T> &v2)
    {
        return !(v1 == v2);
    }

    /** Component-wise binary scalar multiplication. */
    friend SgVec4T<T> operator *(const SgVec4T<T> &v, T d)
    {
        return SgVec4T<T>(v[0] * d, v[1] * d, v[2] * d, v[3] * d);
    }

    /** Component-wise binary scalar multiplication. */
    friend SgVec4T<T> operator *(T d, const SgVec4T<T> &v)
    {
        return v * d;
    }

    /** Cross product. not supported by vec4 yet*/
    //friend SgVec4T<T> operator *( const SgVec4T<T> &v1, const SgVec4T<T> &v2 )


    /** Component-wise binary scalar division operators. */
    friend SgVec4T<T> operator /(const SgVec4T<T> &v, T d)
    {
        return SgVec4T<T>(v[0] / d, v[1] / d, v[2] / d, v[3] / d);
    }

    /** Component-wise binary vector addition. */
    friend SgVec4T<T> operator +(const SgVec4T<T> &v1, const SgVec4T<T> &v2)
    {
        return SgVec4T<T>(v1[0] + v2[0], v1[1] + v2[1], v1[2] + v2[2], v1[3] + v2[3]);
    }

    /** Component-wise binary vector subtraction. */
    friend SgVec4T<T> operator -(const SgVec4T<T> &v1, const SgVec4T<T> &v2)
    {
        return SgVec4T<T>(v1[0] - v2[0], v1[1] - v2[1], v1[2] - v2[2], v1[3] - v2[3]);
    }

    /** Output a formatted string for the vector to a stream. */
    friend std::ostream & operator <<(std::ostream  &os,
        const SgVec4T<T> &v)
    {
        os << v[0] << " " << v[1] << " " << v[2] << " " << v[3]; return os;
    }

    /** Input a vector from a formatted string. */
    friend std::istream & operator >>(std::istream  &is,
        SgVec4T<T>     &v)
    {
        is >> v[0] >> v[1] >> v[2] >> v[3]; return is;
    }

    /** Get coordinates. */
    void getValue(T &x, T &y, T &z, T &w) const
    {
        x = _vec[0]; y = _vec[1]; z = _vec[2]; w = _vec[3];
    }

    /** Get coordinates as array. */
    const T *getValue() const
    {
        return _vec;
    }

    /** Set coordinates. */
    void setValue(T x, T y, T z, T w)
    {
        _vec[0] = x; _vec[1] = y; _vec[2] = z; _vec[3] = w;
    }

    /** Set coordinates as array. */
    void setValue(const T v[4])
    {
        _vec[0] = v[0]; _vec[1] = v[1]; _vec[2] = v[2]; _vec[3] = v[3];
    }

    /** Inner product. */
    T dot(const SgVec4T<T> &v) const
    {
        return _vec[0] * v[0] + _vec[1] * v[1] + _vec[2] * v[2] + _vec[3] * v[3];
    }

    /** Negate vector. */
    void negate()
    {
        _vec[0] *= -1; _vec[1] *= -1; _vec[2] *= -1; _vec[3] *= -1;
    }

    /** Length of vector. */
    T length() const
    {
        return sqrt(_vec[0] * _vec[0] + _vec[1] * _vec[1] + _vec[2] * _vec[2] + _vec[3] * _vec[3]);
    }

    /** Normalize vector. */
    T normalize()
    {
        T len = length();
        if (len < T(0.000001)) { setValue(T(0.0), T(0.0), T(0.0), T(0.0));}
        else { T fact = T(1.0) / len; *this *= fact; } return len;
    }

    /** A normalized copy of the vector. */
    SgVec4T<T> normalized() const
    {
        T len = length();
        if (len < T(0.000001)) { return SgVec4T<T>(T(0.0), T(0.0), T(0.0), T(0.0)); }
        else {T fact = T(1.0) / len;return *this * fact; }
    }

    /**
     * Returns the angle in radians between the current vector and the
     * passed in vector.
     */
    //T angle( const SgVec4T<T> &v ) const

    /** Return a vector orthogonal to the current vector. */
    //SgVec4T<T> orthogonal()

    /**
    * Returns the vector rotated by the angle given in radians about
    * the given axis.
    * NOTE: This uses a lot of matrix and quaternion math.  If we intend to
    * start doing more than this simple rotation calculation, it may benefit
    * to implement an SgMatrix and SgQuaternion class.
    */
    //SgVec4T<T> rotateBy(const SgVec4T<T> &axis, double angle) const


private:

    /** Coordinates. */
    T  _vec[4];
};

typedef SgVec4T<double> SgVec4d;
typedef SgVec4T<float>  SgVec4f;

#endif
