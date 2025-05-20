#ifndef _LEPTRINO_VISIBILITY_CONTROL_H
#define _LEPTRINO_VISIBILITY_CONTROL_H

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define LEPTRINO_FORCE_TORQUE_EXPORT __attribute__((dllexport))
#define LEPTRINO_FORCE_TORQUE_IMPORT __attribute__((dllimport))
#else
#define LEPTRINO_FORCE_TORQUE_EXPORT __declspec(dllexport)
#define LEPTRINO_FORCE_TORQUE_IMPORT __declspec(dllimport)
#endif
#ifdef LEPTRINO_FORCE_TORQUE_BUILDING_DLL
#define LEPTRINO_FORCE_TORQUE_PUBLIC LEPTRINO_FORCE_TORQUE_EXPORT
#else
#define LEPTRINO_FORCE_TORQUE_PUBLIC LEPTRINO_FORCE_TORQUE_IMPORT
#endif
#define LEPTRINO_FORCE_TORQUE_PUBLIC_TYPE LEPTRINO_FORCE_TORQUE_PUBLIC
#define LEPTRINO_FORCE_TORQUE_LOCAL
#else
#define LEPTRINO_FORCE_TORQUE_EXPORT __attribute__((visibility("default")))
#define LEPTRINO_FORCE_TORQUE_IMPORT
#if __GNUC__ >= 4
#define LEPTRINO_FORCE_TORQUE_PUBLIC __attribute__((visibility("default")))
#define LEPTRINO_FORCE_TORQUE_LOCAL __attribute__((visibility("hidden")))
#else
#define LEPTRINO_FORCE_TORQUE_PUBLIC
#define LEPTRINO_FORCE_TORQUE_LOCAL
#endif
#define LEPTRINO_FORCE_TORQUE_PUBLIC_TYPE
#endif

#endif // _LEPTRINO_VISIBILITY_CONTROL_H