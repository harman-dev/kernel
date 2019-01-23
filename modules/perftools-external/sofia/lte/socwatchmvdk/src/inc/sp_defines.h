#ifndef  _SP_DEFINES_H_
#define  _SP_DEFINES_H_

//
// Start off with none of the OS'es are defined
//
#undef SWDRV_OS_LINUX
#undef SWDRV_OS_ANDROID
#undef SWDRV_OS_UNIX

//
// Make sure none of the architectures is defined here
//
#undef SWDRV_IA32
#undef SWDRV_EM64T

//
// Make sure one (and only one) of the OS'es gets defined here
//
// Unfortunately entirex defines _WIN32 so we need to check for linux
// first.  The definition of these flags is one and only one
// _OS_xxx is allowed to be defined.
//
#if defined(__ANDROID__)
#define SWDRV_OS_ANDROID
#define SWDRV_OS_UNIX
#elif defined(__linux__)
#define SWDRV_OS_LINUX
#define SWDRV_OS_UNIX
#else
#error "Compiling for an unknown OS"
#endif

//
// Make sure one (and only one) architecture is defined here
// as well as one (and only one) pointer__ size
//
#if defined(_M_IX86) || defined(__i386__)
#define SWDRV_IA32
#elif defined(_M_AMD64) || defined(__x86_64__)
#define SWDRV_EM64T
#else
#error "Unknown architecture for compilation"
#endif

#endif // _SP_DEFINES_H_
