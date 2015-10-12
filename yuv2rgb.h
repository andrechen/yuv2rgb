/*
 * Copyright (C) 2012 Andre Chen and contributors.
 * andre.hl.chen@gmail.com
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifndef YUV_TO_RGB
#define YUV_TO_RGB

/*
 YUV 4:2:0 (sp for "semi-plane")image with a plane of 8 bit Y samples followed by an interleaved
 U/V plane containing 8 bit 2x2 subsampled chroma samples.
 except the interleave order of U and V is reversed.

Cumbersome YUV formats(http://www.fourcc.org/yuv.php)...

NV12
YUV 4:2:0 image with a plane of 8 bit Y samples followed by an interleaved U/V plane containing 8 bit 2x2 subsampled colour difference samples.
Microsoft defines this format as follows:
	"A format in which all Y samples are found first in memory as an array of unsigned char with an even number of lines 
	(possibly with a larger stride for memory alignment), followed immediately by an array of unsigned char containing interleaved Cb and Cr 
	samples (such that if addressed as a little-endian WORD type, Cb(U) would be in the LSBs and Cr(V) would be in the MSBs) with the same total 
	stride as the Y samples. This is the preferred 4:2:0 pixel format"
e.g. YYYYYYYY YYYYYYYY YYYYYYYY YYYYYYYY UVUVUVUV UVUVUVUV
 
NV21(aka YCrCb format. the default format for camera preview images)
YUV 4:2:0 image with a plane of 8 bit Y samples followed by an interleaved V/U plane containing 8 bit 2x2 subsampled chroma samples.
The same as NV12 except the interleave order of U and V is reversed.
e.g. YYYYYYYY YYYYYYYY YYYYYYYY YYYYYYYY VUVUVUVU VUVUVUVU


To convert Y'UV to RGB :
 matrix from:
 |R|   | 298    0     409 | | Y'- 16  |
 |G| = | 298  -100   -208 | | U - 128 |
 |B|   | 298   516     0  | | V - 128 |
 then shift 8 bits, i.e.
 
 in integer math:
 R = clamp((298*(Y'-16)+409*(V-128)+128)>>8)
 G = clamp((298*(Y'-16)-100*(U-128)-208*(V-128)+128)>>8)
 B = clamp((298*(Y'-16)+516*(U-128)+128)>>8)
 
 to encode RGB to Y'UV..
 Y' = (( 66 * R + 129 * G +  25 * B + 128) >> 8) +  16
 U  = ((-38 * R -  74 * G + 112 * B + 128) >> 8) + 128
 V  = ((112 * R -  94 * G -  18 * B + 128) >> 8) + 128
 */

//
// [in]
//		alpha : alpha value if rgba
//		yuv : nv21 image(size=width*height*3/2)
//      width : must be even
//      height = must be even
// [out]
//      rgb : rgb buffer(size>=width*height*3) byte order : R0 G0 B0  R1 G1 B1  R2 G2 B2
//		rgba : rgba buffer(size>=width*height*4) byte order : R0 G0 B0 A0  R1 G1 B1 A1  R2 G2 B2 A2
bool nv21_to_rgb(unsigned char* rgb, unsigned char const* nv21, int width, int height);
bool nv21_to_rgba(unsigned char* rgba, unsigned char alpha, unsigned char const* nv21, int width, int height);

// OpenCV style
bool nv21_to_bgr(unsigned char* bgr, unsigned char const* nv21, int width, int height);
bool nv21_to_bgra(unsigned char* bgra, unsigned char alpha, unsigned char const* nv21, int width, int height);

//
//
// to make the buile in android(activate neon), either...
// method 1)
//       ifeq ($(TARGET_ARCH_ABI),armeabi-v7a)
//			LOCAL_CFLAGS += -DARM_NEON_ENABLE -mfpu=neon -flax-vector-conversions
//       endif
//       LOCAL_SRC_FILES += yuv2rgb.cpp
//
// (note ARM_NEON_ENABLE is defined, in case you'd like to exclude neon stuff from the build)
//
// method 2) (use funny suffix as NDK suggestted)
//		ifeq ($(TARGET_ARCH_ABI),armeabi-v7a)
//			LOCAL_CFLAGS += -DARM_NEON_ENABLE -flax-vector-conversions
//			LOCAL_SRC_FILES += yuv2rgb.cpp.neon
//		else
//			LOCAL_SRC_FILES += yuv2rgb.cpp
//		endif
//
// this compiles on GCC(android), Xcode(iOS).
//

#endif