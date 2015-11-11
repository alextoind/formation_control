#ifndef __C2000TYPE_H__
#define __C2000TYPE_H__

#ifdef _WIN_32
typedef unsigned char i_uint8;
typedef char i_int8;
typedef unsigned short i_uint16;
typedef short i_int16;
typedef unsigned int i_uint32;
typedef int i_int32;
typedef unsigned long long i_uint64;
typedef long long i_int64;
typedef float i_float;
typedef double i_double;
#else
typedef unsigned char i_uint8;
typedef char i_int8;
typedef unsigned short i_uint16;
typedef short i_int16;
typedef unsigned long i_uint32;
typedef long i_int32;
typedef unsigned long long i_uint64;
typedef long long i_int64;
typedef float i_float;
typedef long double i_double;
#endif


#endif
