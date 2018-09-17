/**
 * @file utils.h
 * Contains some utility functions.
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <string>

template<typename T>
T square(T d)
{
  return d*d;
}

template<typename T>
T maxElement(const T* rgd, int len)
{
  return *std::max_element(rgd, rgd + len);
}

template<typename T>
T minElement(const T* rgd, int len)
{
  return *std::min_element(rgd, rgd + len);
}

template<typename T>
int maxIndex(const T* rgd, int len)
{
  return std::max_element(rgd, rgd + len) - rgd;
}

template<typename T>
int minIndex(const T* rgd, int len)
{
  return std::min_element(rgd, rgd + len) - rgd;
}

/** sqrt(a^2 + b^2) numerically stable. */
template<typename T>
T myhypot(T a, T b)
{
  const register T fabsa = std::fabs(a), fabsb = std::fabs(b);
  if(fabsa > fabsb)
  {
    const register T r = b / a;
    return fabsa*std::sqrt(T(1)+r*r);
  }
  else if(b != T(0))
  {
    const register T r = a / b;
    return fabsb*std::sqrt(T(1)+r*r);
  }
  else
    return T(0);
}
