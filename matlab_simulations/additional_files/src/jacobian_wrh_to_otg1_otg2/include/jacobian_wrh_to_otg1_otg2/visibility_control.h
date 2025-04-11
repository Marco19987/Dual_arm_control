#ifndef JACOBIAN_WRH_TO_OTG1_OTG2__VISIBILITY_CONTROL_H_
#define JACOBIAN_WRH_TO_OTG1_OTG2__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define JACOBIAN_WRH_TO_OTG1_OTG2_EXPORT __attribute__ ((dllexport))
    #define JACOBIAN_WRH_TO_OTG1_OTG2_IMPORT __attribute__ ((dllimport))
  #else
    #define JACOBIAN_WRH_TO_OTG1_OTG2_EXPORT __declspec(dllexport)
    #define JACOBIAN_WRH_TO_OTG1_OTG2_IMPORT __declspec(dllimport)
  #endif
  #ifdef JACOBIAN_WRH_TO_OTG1_OTG2_BUILDING_LIBRARY
    #define JACOBIAN_WRH_TO_OTG1_OTG2_PUBLIC JACOBIAN_WRH_TO_OTG1_OTG2_EXPORT
  #else
    #define JACOBIAN_WRH_TO_OTG1_OTG2_PUBLIC JACOBIAN_WRH_TO_OTG1_OTG2_IMPORT
  #endif
  #define JACOBIAN_WRH_TO_OTG1_OTG2_PUBLIC_TYPE JACOBIAN_WRH_TO_OTG1_OTG2_PUBLIC
  #define JACOBIAN_WRH_TO_OTG1_OTG2_LOCAL
#else
  #define JACOBIAN_WRH_TO_OTG1_OTG2_EXPORT __attribute__ ((visibility("default")))
  #define JACOBIAN_WRH_TO_OTG1_OTG2_IMPORT
  #if __GNUC__ >= 4
    #define JACOBIAN_WRH_TO_OTG1_OTG2_PUBLIC __attribute__ ((visibility("default")))
    #define JACOBIAN_WRH_TO_OTG1_OTG2_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define JACOBIAN_WRH_TO_OTG1_OTG2_PUBLIC
    #define JACOBIAN_WRH_TO_OTG1_OTG2_LOCAL
  #endif
  #define JACOBIAN_WRH_TO_OTG1_OTG2_PUBLIC_TYPE
#endif
#endif  // JACOBIAN_WRH_TO_OTG1_OTG2__VISIBILITY_CONTROL_H_
// Generated 11-Apr-2025 11:17:44
 