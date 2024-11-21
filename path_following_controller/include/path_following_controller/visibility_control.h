// Copyright (c) 2024, Posidonia Technology d.o.o.
// All rights reserved.
// 
// Apache License, Version 2.0
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#ifndef PATH_FOLLOWING_CONTROLLER__VISIBILITY_CONTROL_H_
#define PATH_FOLLOWING_CONTROLLER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define PATH_FOLLOWING_CONTROLLER__VISIBILITY_EXPORT __attribute__((dllexport))
#define PATH_FOLLOWING_CONTROLLER__VISIBILITY_IMPORT __attribute__((dllimport))
#else
#define PATH_FOLLOWING_CONTROLLER__VISIBILITY_EXPORT __declspec(dllexport)
#define PATH_FOLLOWING_CONTROLLER__VISIBILITY_IMPORT __declspec(dllimport)
#endif
#ifdef PATH_FOLLOWING_CONTROLLER__VISIBILITY_BUILDING_DLL
#define PATH_FOLLOWING_CONTROLLER__VISIBILITY_PUBLIC PATH_FOLLOWING_CONTROLLER__VISIBILITY_EXPORT
#else
#define PATH_FOLLOWING_CONTROLLER__VISIBILITY_PUBLIC PATH_FOLLOWING_CONTROLLER__VISIBILITY_IMPORT
#endif
#define PATH_FOLLOWING_CONTROLLER__VISIBILITY_PUBLIC_TYPE PATH_FOLLOWING_CONTROLLER__VISIBILITY_PUBLIC
#define PATH_FOLLOWING_CONTROLLER__VISIBILITY_LOCAL
#else
#define PATH_FOLLOWING_CONTROLLER__VISIBILITY_EXPORT __attribute__((visibility("default")))
#define PATH_FOLLOWING_CONTROLLER__VISIBILITY_IMPORT
#if __GNUC__ >= 4
#define PATH_FOLLOWING_CONTROLLER__VISIBILITY_PUBLIC __attribute__((visibility("default")))
#define PATH_FOLLOWING_CONTROLLER__VISIBILITY_LOCAL __attribute__((visibility("hidden")))
#else
#define PATH_FOLLOWING_CONTROLLER__VISIBILITY_PUBLIC
#define PATH_FOLLOWING_CONTROLLER__VISIBILITY_LOCAL
#endif
#define PATH_FOLLOWING_CONTROLLER__VISIBILITY_PUBLIC_TYPE
#endif

#endif  // PATH_FOLLOWING_CONTROLLER__VISIBILITY_CONTROL_H_
