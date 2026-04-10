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

#define USECOUT

#ifdef USECOUT
#include <iostream>
#define WJ_ERROR std::cout << "\033[1m\033[31m"
#define WJ_WARNING std::cout << "\033[1m\033[33m"
#define WJ_INFO std::cout << "\033[1m\033[32m"
#define WJ_INFOL std::cout << "\033[32m"
#define WJ_DEBUG std::cout << "\033[1m\033[36m"

#define WJ_TITLE std::cout << "\033[1m\033[35m"
#define WJ_MSG std::cout << "\033[1m\033[37m"
#define WJ_REND "\033[0m" << std::endl

#else
#include <sstream>
#include <string>

#define WJ_ERROR            \
  {                         \
    std::ostringstream oss; \
    oss << "\033[1m\033[31m[ERROR]"
#define WJ_WARNING          \
  {                         \
    std::ostringstream oss; \
    oss << "\033[1m\033[33m[WARNING]"
#define WJ_INFO             \
  {                         \
    std::ostringstream oss; \
    oss << "\033[1m\033[32m[INFO]"
#define WJ_INFOL            \
  {                         \
    std::ostringstream oss; \
    oss << "\033[32m[INFOl]"
#define WJ_DEBUG            \
  {                         \
    std::ostringstream oss; \
    oss << "\033[1m\033[36m[DEBUG]"

#define WJ_TITLE            \
  {                         \
    std::ostringstream oss; \
    oss << "\033[1m\033[35m[TITLE]"
#define WJ_MSG              \
  {                         \
    std::ostringstream oss; \
    oss << "\033[1m\033[37m[MSG]"
#define WJ_REND                    \
  "\033[0m" << std::endl;          \
  printf("%s", oss.str().c_str()); \
  }
#endif