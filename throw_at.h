#ifndef THROW_AT_H
#define THROW_AT_H
#include "EigenConvenience.h"
#include <vector>
// Throw element i in A at element RP(i) in B
template <typename T, typename TAlloc>
void throw_at(
  const std::vector<T,TAlloc> & A,
  const Eigen::VectorXi & RP,
  std::vector<T,TAlloc> & B);
template <typename T, typename TAlloc>
std::vector<T,TAlloc> throw_at(
  const std::vector<T,TAlloc> & A,
  const Eigen::VectorXi & RP);
#endif
