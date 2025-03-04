#ifndef _UTIL_H_
#define _UTIL_H_

#ifdef __cplusplus
extern "C" {
#endif

/// @brief Limits value x to between min_val and max_val I.e. [min_val, max_val]
/// @param x Value to clamp
/// @param min_val Minimum bound of output
/// @param max_val Maximum bound of output
/// @return Clamped value of x
float clamp(float x, float min_val, float max_val);

#ifdef __cplusplus
}
#endif
#endif // _UTIL_H_