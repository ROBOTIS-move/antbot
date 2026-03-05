/*********************************************************************************************************************
Copyright (c) 2023 Vanjee
All rights reserved

By downloading, copying, installing or using the software you agree to this
license. If you do not agree to this license, do not download, install, copy or
use the software.

License Agreement
For Vanjee LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

3. Neither the names of the Vanjee, nor Suteng Innovation Technology, nor the
names of other contributors may be used to endorse or promote products derived
from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#pragma once

#include <iostream>
#include <type_traits>

#define DEFINE_MEMBER_CHECKER(member)       \
  template <typename T, typename V = bool>  \
  struct has_##member : std::false_type {}; \
  template <typename T>                     \
  struct has_##member<T, typename std::enable_if<!std::is_same<decltype(std::declval<T>().member), void>::value, bool>::type> : std::true_type {};

DEFINE_MEMBER_CHECKER(x)
DEFINE_MEMBER_CHECKER(y)
DEFINE_MEMBER_CHECKER(z)
DEFINE_MEMBER_CHECKER(intensity)
DEFINE_MEMBER_CHECKER(ring)
DEFINE_MEMBER_CHECKER(timestamp)
#ifdef ENABLE_GTEST
DEFINE_MEMBER_CHECKER(point_id)
DEFINE_MEMBER_CHECKER(hor_angle)
DEFINE_MEMBER_CHECKER(ver_angle)
DEFINE_MEMBER_CHECKER(distance)
#endif

#define VANJEE_HAS_MEMBER(C, member) has_##member<C>::value
template <typename T_Point>
inline typename std::enable_if<!VANJEE_HAS_MEMBER(T_Point, x)>::type setX(T_Point &point, const float &value) {
}

template <typename T_Point>
inline typename std::enable_if<VANJEE_HAS_MEMBER(T_Point, x)>::type setX(T_Point &point, const float &value) {
  point.x = value;
}

template <typename T_Point>
inline typename std::enable_if<!VANJEE_HAS_MEMBER(T_Point, y)>::type setY(T_Point &point, const float &value) {
}

template <typename T_Point>
inline typename std::enable_if<VANJEE_HAS_MEMBER(T_Point, y)>::type setY(T_Point &point, const float &value) {
  point.y = value;
}

template <typename T_Point>
inline typename std::enable_if<!VANJEE_HAS_MEMBER(T_Point, z)>::type setZ(T_Point &point, const float &value) {
}

template <typename T_Point>
inline typename std::enable_if<VANJEE_HAS_MEMBER(T_Point, z)>::type setZ(T_Point &point, const float &value) {
  point.z = value;
}

template <typename T_Point>
inline typename std::enable_if<!VANJEE_HAS_MEMBER(T_Point, intensity)>::type setIntensity(T_Point &point, const float &value) {
}

template <typename T_Point>
inline typename std::enable_if<VANJEE_HAS_MEMBER(T_Point, intensity)>::type setIntensity(T_Point &point, const float &value) {
  point.intensity = value;
}

template <typename T_Point>
inline typename std::enable_if<!VANJEE_HAS_MEMBER(T_Point, ring)>::type setRing(T_Point &point, const uint16_t &value) {
}

template <typename T_Point>
inline typename std::enable_if<VANJEE_HAS_MEMBER(T_Point, ring)>::type setRing(T_Point &point, const uint16_t &value) {
  point.ring = value;
}

template <typename T_Point>
inline typename std::enable_if<!VANJEE_HAS_MEMBER(T_Point, timestamp)>::type setTimestamp(T_Point &point, const double &value) {
}

template <typename T_Point>
inline typename std::enable_if<VANJEE_HAS_MEMBER(T_Point, timestamp)>::type setTimestamp(T_Point &point, const double &value) {
  point.timestamp = value;
}
#ifdef ENABLE_GTEST
template <typename T_Point>
inline typename std::enable_if<!VANJEE_HAS_MEMBER(T_Point, point_id)>::type setPointId(T_Point &point, const uint32_t &value) {
}

template <typename T_Point>
inline typename std::enable_if<VANJEE_HAS_MEMBER(T_Point, point_id)>::type setPointId(T_Point &point, const uint32_t &value) {
  point.point_id = value;
}

template <typename T_Point>
inline typename std::enable_if<!VANJEE_HAS_MEMBER(T_Point, hor_angle)>::type setHorAngle(T_Point &point, const float &value) {
}

template <typename T_Point>
inline typename std::enable_if<VANJEE_HAS_MEMBER(T_Point, hor_angle)>::type setHorAngle(T_Point &point, const float &value) {
  point.hor_angle = value;
}

template <typename T_Point>
inline typename std::enable_if<!VANJEE_HAS_MEMBER(T_Point, ver_angle)>::type setVerAngle(T_Point &point, const float &value) {
}

template <typename T_Point>
inline typename std::enable_if<VANJEE_HAS_MEMBER(T_Point, ver_angle)>::type setVerAngle(T_Point &point, const float &value) {
  point.ver_angle = value;
}

template <typename T_Point>
inline typename std::enable_if<!VANJEE_HAS_MEMBER(T_Point, distance)>::type setDistance(T_Point &point, const float &value) {
}

template <typename T_Point>
inline typename std::enable_if<VANJEE_HAS_MEMBER(T_Point, distance)>::type setDistance(T_Point &point, const float &value) {
  point.distance = value;
}
#endif
