#ifndef VISION_MSGS_LAYERS__VISIBILITY_CONTROL_H_
#define VISION_MSGS_LAYERS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define VISION_MSGS_LAYERS_EXPORT __attribute__ ((dllexport))
    #define VISION_MSGS_LAYERS_IMPORT __attribute__ ((dllimport))
  #else
    #define VISION_MSGS_LAYERS_EXPORT __declspec(dllexport)
    #define VISION_MSGS_LAYERS_IMPORT __declspec(dllimport)
  #endif
  #ifdef VISION_MSGS_LAYERS_BUILDING_LIBRARY
    #define VISION_MSGS_LAYERS_PUBLIC VISION_MSGS_LAYERS_EXPORT
  #else
    #define VISION_MSGS_LAYERS_PUBLIC VISION_MSGS_LAYERS_IMPORT
  #endif
  #define VISION_MSGS_LAYERS_PUBLIC_TYPE VISION_MSGS_LAYERS_PUBLIC
  #define VISION_MSGS_LAYERS_LOCAL
#else
  #define VISION_MSGS_LAYERS_EXPORT __attribute__ ((visibility("default")))
  #define VISION_MSGS_LAYERS_IMPORT
  #if __GNUC__ >= 4
    #define VISION_MSGS_LAYERS_PUBLIC __attribute__ ((visibility("default")))
    #define VISION_MSGS_LAYERS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define VISION_MSGS_LAYERS_PUBLIC
    #define VISION_MSGS_LAYERS_LOCAL
  #endif
  #define VISION_MSGS_LAYERS_PUBLIC_TYPE
#endif

#endif  // VISION_MSGS_LAYERS__VISIBILITY_CONTROL_H_
