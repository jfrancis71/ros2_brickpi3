
#ifndef BRICKPI3_MOTORS__VISIBILITY_CONTROL_H_
#define BRICKPI3_MOTORS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define BRICKPI3_MOTORS_EXPORT __attribute__((dllexport))
#define BRICKPI3_MOTORS_IMPORT __attribute__((dllimport))
#else
#define BRICKPI3_MOTORS_EXPORT __declspec(dllexport)
#define BRICKPI3_MOTORS_IMPORT __declspec(dllimport)
#endif
#ifdef BRICKPI3_MOTORS_BUILDING_DLL
#define BRICKPI3_MOTORS_PUBLIC BRICKPI3_MOTORS_EXPORT
#else
#define BRICKPI3_MOTORS_PUBLIC BRICKPI3_MOTORS_IMPORT
#endif
#define BRICKPI3_MOTORS_PUBLIC_TYPE BRICKPI3_MOTORS_PUBLIC
#define BRICKPI3_MOTORS_LOCAL
#else
#define BRICKPI3_MOTORS_EXPORT __attribute__((visibility("default")))
#define BRICKPI3_MOTORS_IMPORT
#if __GNUC__ >= 4
#define BRICKPI3_MOTORS_PUBLIC __attribute__((visibility("default")))
#define BRICKPI3_MOTORS_LOCAL __attribute__((visibility("hidden")))
#else
#define BRICKPI3_MOTORS_PUBLIC
#define BRICKPI3_MOTORS_LOCAL
#endif
#define BRICKPI3_MOTORS_PUBLIC_TYPE
#endif

#endif  // BRICKPI3_MOTORS__VISIBILITY_CONTROL_H_
