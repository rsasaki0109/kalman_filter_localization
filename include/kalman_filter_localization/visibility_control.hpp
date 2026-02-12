// Copyright (c) 2020, Ryohei Sasaki
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of {copyright_holder} nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#ifndef KALMAN_FILTER_LOCALIZATION__VISIBILITY_CONTROL_HPP_
#define KALMAN_FILTER_LOCALIZATION__VISIBILITY_CONTROL_HPP_

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define KFL_EKFL_EXPORT __attribute__ ((dllexport))
    #define KFL_EKFL_IMPORT __attribute__ ((dllimport))
  #else
    #define KFL_EKFL_EXPORT __declspec(dllexport)
    #define KFL_EKFL_IMPORT __declspec(dllimport)
  #endif
  #ifdef KFL_EKFL_BUILDING_DLL
    #define KFL_EKFL_PUBLIC KFL_EKFL_EXPORT
  #else
    #define KFL_EKFL_PUBLIC KFL_EKFL_IMPORT
  #endif
  #define KFL_EKFL_PUBLIC_TYPE KFL_EKFL_PUBLIC
  #define KFL_EKFL_LOCAL
#else
  #define KFL_EKFL_EXPORT __attribute__ ((visibility("default")))
  #define KFL_EKFL_IMPORT
  #if __GNUC__ >= 4
    #define KFL_EKFL_PUBLIC __attribute__ ((visibility("default")))
    #define KFL_EKFL_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define KFL_EKFL_PUBLIC
    #define KFL_EKFL_LOCAL
  #endif
  #define KFL_EKFL_PUBLIC_TYPE
#endif

#endif  // KALMAN_FILTER_LOCALIZATION__VISIBILITY_CONTROL_HPP_
