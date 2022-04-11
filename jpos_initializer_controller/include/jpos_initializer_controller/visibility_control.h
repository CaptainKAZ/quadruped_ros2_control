// Copyright (c) 2022, Zou Yuanhao
// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef JPOS_INITIALIZER_CONTROLLER__VISIBILITY_CONTROL_H_
#define JPOS_INITIALIZER_CONTROLLER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define JPOS_INITIALIZER_CONTROLLER_EXPORT __attribute__((dllexport))
#define JPOS_INITIALIZER_CONTROLLER_IMPORT __attribute__((dllimport))
#else
#define JPOS_INITIALIZER_CONTROLLER_EXPORT __declspec(dllexport)
#define JPOS_INITIALIZER_CONTROLLER_IMPORT __declspec(dllimport)
#endif
#ifdef JPOS_INITIALIZER_CONTROLLER_BUILDING_DLL
#define JPOS_INITIALIZER_CONTROLLER_PUBLIC JPOS_INITIALIZER_CONTROLLER_EXPORT
#else
#define JPOS_INITIALIZER_CONTROLLER_PUBLIC JPOS_INITIALIZER_CONTROLLER_IMPORT
#endif
#define JPOS_INITIALIZER_CONTROLLER_PUBLIC_TYPE JPOS_INITIALIZER_CONTROLLER_PUBLIC
#define JPOS_INITIALIZER_CONTROLLER_LOCAL
#else
#define JPOS_INITIALIZER_CONTROLLER_EXPORT __attribute__((visibility("default")))
#define JPOS_INITIALIZER_CONTROLLER_IMPORT
#if __GNUC__ >= 4
#define JPOS_INITIALIZER_CONTROLLER_PUBLIC __attribute__((visibility("default")))
#define JPOS_INITIALIZER_CONTROLLER_LOCAL __attribute__((visibility("hidden")))
#else
#define JPOS_INITIALIZER_CONTROLLER_PUBLIC
#define JPOS_INITIALIZER_CONTROLLER_LOCAL
#endif
#define JPOS_INITIALIZER_CONTROLLER_PUBLIC_TYPE
#endif

#endif  // JPOS_INITIALIZER_CONTROLLER__VISIBILITY_CONTROL_H_
