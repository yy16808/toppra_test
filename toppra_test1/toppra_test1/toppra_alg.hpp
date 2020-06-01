#ifndef TOPPRA_ALGORITHM_TOPPRA_HPP
#define TOPPRA_ALGORITHM_TOPPRA_HPP

#include "algorithm.hpp"
#include "constraint.hpp"
#include "geometric_path.hpp"
#include "toppra.hpp"

namespace toppra {
namespace algorithm {
class TOPPRA : public PathParametrizationAlgorithm {
 public:
  TOPPRA(LinearConstraintPtrs constraints, const GeometricPathPtr &path);

 protected:
  ReturnCode computeForwardPass(value_type vel_start);
};
}  // namespace algorithm
}  // namespace toppra

#endif
