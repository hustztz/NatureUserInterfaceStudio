// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#include "targetver.h"

#include <stdio.h>
#include <tchar.h>

template<class Interface>
inline void SafeRelease( Interface *& pInterfaceToRelease )
{
	if( pInterfaceToRelease != nullptr ){
		pInterfaceToRelease->Release();
		pInterfaceToRelease = nullptr;
	}
}

template<class Interface>
inline void SafeDelete( Interface *& pInterfaceToRelease )
{
	if( pInterfaceToRelease != nullptr ){
		delete pInterfaceToRelease;
		pInterfaceToRelease = nullptr;
	}
}

template<class Interface>
inline void SafeDeleteArray( Interface *& pInterfaceToRelease )
{
	if( pInterfaceToRelease != nullptr ){
		delete[] pInterfaceToRelease;
		pInterfaceToRelease = nullptr;
	}
}

inline int sign(float x)
{
	if(x>0)
		return 1;
	else
		return -1;
}

typedef unsigned long		DWORD;
typedef signed __int64      INT64;
typedef unsigned char       BYTE;
typedef unsigned short      UINT16;
typedef signed int          INT32;
typedef unsigned int        UINT32;
typedef unsigned int        UINT;

#define NAN_FLOAT -std::numeric_limits<float>::max()

#ifndef _tagBGRQUAD_
#define _tagBGRQUAD_
typedef struct tagBGRQUAD {
	BYTE    rgbBlue;
	BYTE    rgbGreen;
	BYTE    rgbRed;
	BYTE    rgbReserved;
} BGRQUAD;
#endif

#ifndef _RectI_
#define _RectI_
typedef struct _RectI
{
	INT32 Left;
	INT32 Top;
	INT32 Right;
	INT32 Bottom;
} 	RectI;
#endif // _RectI_

#ifndef _Vector3_
#define _Vector3_
typedef struct _Vector3
{
	float x;
	float y;
	float z;
} 	Vector3;
#endif _Vector3_

#ifndef _Vector4_
#define _Vector4_
typedef struct _Vector4
{
	float x;
	float y;
	float z;
	float w;
} 	Vector4;
#endif // _Vector4_

#ifndef _PointF_
#define _PointF_
typedef struct _PointF
{
	float X;
	float Y;
} 	PointF;
#endif // _PointF_

#ifndef _Point4D_
#define _Point4D_
typedef union _Point4D
{
	float data[4];
	struct {
		float x;
		float y;
		float z;
	};
} 	Point4D;
#endif // _Point4D_

#ifndef _CameraSpacePoint_
#define _CameraSpacePoint_
typedef struct _CameraSpacePoint
{
	float X;
	float Y;
	float Z;
} 	CameraSpacePoint;
#endif // _CameraSpacePoint_

#ifndef _ColorSpacePoint_
#define _ColorSpacePoint_
typedef struct _ColorSpacePoint
{
	float X;
	float Y;
} 	ColorSpacePoint;
#endif // _ColorSpacePoint_

#ifndef _DepthSpacePoint_
#define _DepthSpacePoint_
typedef struct _DepthSpacePoint
{
	float X;
	float Y;
} 	DepthSpacePoint;

#endif // _DepthSpacePoint_

#ifndef _Matrix4_
#define _Matrix4_
typedef struct _Matrix4
{
	float M11;
	float M12;
	float M13;
	float M14;
	float M21;
	float M22;
	float M23;
	float M24;
	float M31;
	float M32;
	float M33;
	float M34;
	float M41;
	float M42;
	float M43;
	float M44;
} 	Matrix4;
#endif _Matrix4_

#ifndef _DetectionResult_
#define _DetectionResult_
typedef enum _DetectionResult DetectionResult;
enum _DetectionResult
{
	DetectionResult_Unknown	= 0,
	DetectionResult_No	= 1,
	DetectionResult_Maybe	= 2,
	DetectionResult_Yes	= 3
} ;
#endif // _DetectionResult_

// TODO: reference additional headers your program requires here
