#ifndef MISC_H
#define MISC_H

#include <Eigen/Core>
#include <boost/utility.hpp>
#include <boost/type_traits.hpp>
#include "ceres/ceres.h"

namespace teb_demo
{

#define SMALL_NUM 0.00000001

template <typename T>
inline T fast_sigmoid(T x)
{
  return x / ( (T)1 + std::abs<T>(x));
}

/**
 * @brief Calculate Euclidean distance between two 2D point datatypes
 * @param point1 object containing fields x and y
 * @param point2 object containing fields x and y
 * @return Euclidean distance: ||point2-point1||
*/
template <typename P1, typename P2>
inline double distance_points2d(const P1& point1, const P2& point2)
{
  return std::sqrt( std::pow(point2.x-point1.x,2) + std::pow(point2.y-point1.y,2) );
}


/**
 * @brief Calculate the 2d cross product (returns length of the resulting vector along the z-axis in 3d)
 * @param v1 object containing public methods x() and y()
 * @param v2 object containing fields x() and y()
 * @return magnitude that would result in the 3D case (along the z-axis)
*/
template <typename V1, typename V2>
inline double cross2d(const V1& v1, const V2& v2)
{
     return v1.x()*v2.y() - v2.x()*v1.y();
}

/** 
 * @brief Helper function that returns the const reference to a value defined by either its raw pointer type or const reference.
 * 
 * Return a constant reference for boths input variants (pointer or reference).
 * @remarks Makes only sense in combination with the overload getConstReference(const T& val).
 * @param ptr pointer of type T
 * @tparam T arbitrary type 
 * @return  If \c T is a pointer, return const *T (leading to const T&), otherwise const T& with out pointer-to-ref conversion
 */
template<typename T>
inline const T& get_const_reference(const T* ptr) {return *ptr;}

/** 
 * @brief Helper function that returns the const reference to a value defined by either its raw pointer type or const reference.
 * 
 * Return a constant reference for boths input variants (pointer or reference).
 * @remarks Makes only sense in combination with the overload getConstReference(const T* val).
 * @param val
 * @param dummy SFINAE helper variable
 * @tparam T arbitrary type 
 * @return  If \c T is a pointer, return const *T (leading to const T&), otherwise const T& with out pointer-to-ref conversion
 */
template<typename T>
inline const T& get_const_reference(const T& val, typename boost::disable_if<boost::is_pointer<T> >::type* dummy = 0) {return val;}



inline constexpr float cst(long double v)
{
  return (float)v;
}

constexpr float const_pi() { return cst(3.14159265358979323846); }

/**
 * return the square value
 */
template <typename T>
inline T square(T x)
{
  return x*x;
}

/**
 * return the hypot of x and y
 */
template <typename T>
inline T hypot(T x, T y)
{
  return (T) (std::sqrt(x*x + y*y));
}

/**
 * return the squared hypot of x and y
 */
template <typename T>
inline T hypot_sqr(T x, T y)
{
  return x*x + y*y;
}

/**
 * convert from degree to radian
 */
inline float deg2rad(float degree)
{
  return degree * cst(0.01745329251994329576);
}

/**
 * convert from radian to degree
 */
inline float rad2deg(float rad)
{
  return rad * cst(57.29577951308232087721);
}

/**
 * normalize the angle
 */
template <typename T>
inline T normalize_theta(T theta)
{
  const T pi = (T)const_pi();
  const T two = (T)(2);
  if (theta >= -pi && theta < pi)
    return theta;
  
  T multiplier = ceres::floor(theta / (two*pi));
  theta = theta - multiplier*two*pi;
  if (theta >= pi)
    theta -=two*pi;
  if (theta < -pi)
    theta += two*pi;

  return theta;
}

/**
 * inverse of an angle, i.e., +180 degree
 */
inline float inverse_theta(float th)
{
  return normalize_theta(th + const_pi());
}

/**
 * average two angles
 */
inline float average_angle(float theta1, float theta2)
{
  float x, y;

  x = std::cos(theta1) + std::cos(theta2);
  y = std::sin(theta1) + std::sin(theta2);
  if(x == 0 && y == 0)
    return 0;
  else
    return std::atan2(y, x);
}

/**
 * sign function.
 * @return the sign of x. +1 for x > 0, -1 for x < 0, 0 for x == 0
 */
template <typename T>
inline int sign(T x)
{
  if (x > 0)
    return 1;
  else if (x < 0)
    return -1;
  else
    return 0;
}

/**
 * clamp x to the interval [l, u]
 */
template <typename T>
inline T clamp(T l, T x, T u) 
{
  if (x < l)
    return l;
  if (x > u)
    return u;
  return x;
}

/**
 * wrap x to be in the interval [l, u]
 */
template <typename T>
inline T wrap(T l, T x, T u) 
{
  T intervalWidth = u - l;
  while (x < l)
    x += intervalWidth;
  while (x > u)
    x -= intervalWidth;
  return x;
}

/**
 * tests whether there is a NaN in the array
 */
inline bool arrayHasNaN(const float* array, int size, int* nanIndex = 0)
{
  for (int i = 0; i < size; ++i)
    if (std::isnan(array[i])) {
      if (nanIndex)
        *nanIndex = i;
      return true;
    }
  return false;
}



} // namespace teb_local_planner

#endif /* MISC_H */
