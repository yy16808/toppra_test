
#include "linear_joint_acceleration.hpp"
#include "geometric_path.hpp"

namespace toppra {
namespace constraint {

std::ostream& LinearJointAcceleration::print (std::ostream& os) const
{
  os << "LinearJointAcceleration\n";
  return LinearConstraint::print(os) <<
    "    Lower acceleration limit: " << m_lower.transpose() << "\n"
    "    Upper acceleration limit: " << m_upper.transpose() << "\n";
}

void LinearJointAcceleration::check ()
{
  if ((m_lower.array() > m_upper.array()).any())
    throw std::invalid_argument("Bad acceleration limits.");
}

void LinearJointAcceleration::computeParams_impl(const GeometricPath& path,
        const Vector& times,
        Vectors& a, Vectors& b, Vectors& c,
        Matrices& F, Vectors& g,
        Bounds&, Bounds&)
{
  Eigen::Index N_1 = times.size();

  Eigen::Index ndofs (nbVariables());

  // Compute F and g
  Matrix& _F = F[0];
  _F.   topRows(ndofs).setIdentity();
  _F.bottomRows(ndofs).setZero();
  _F.bottomRows(ndofs).diagonal().setConstant(-1);
  Vector& _g = g[0];
  _g.head(ndofs) =  m_upper;
  _g.tail(ndofs) = -m_lower;
  //double coeff_limit = 0.0;
  //double headbuf=0.002, endbuf = 1- headbuf;
  //for (std::size_t i = 0; i < N_1; ++i) {
	 // Matrix& _F = F[i];
	 // _F.topRows(ndofs).setIdentity();
	 // _F.bottomRows(ndofs).setZero();
	 // _F.bottomRows(ndofs).diagonal().setConstant(-1);
	 // Vector& _g = g[i];
	 // if (i < headbuf*(N_1-1))
		//  coeff_limit = 0.1 + (i*1.0) / N_1 / headbuf*0.9;
	 // else if (i >=endbuf*(N_1-1))
		//  coeff_limit = 1 - (i*1.0 / (N_1-1)- endbuf) / headbuf*0.9;
	 // else
		//  coeff_limit = 1;
	 // if (i > 990)
	 // {
		//  int stop=1;
	 // }
	 // _g.head(ndofs) = m_upper*coeff_limit;
	 // _g.tail(ndofs) = -m_lower*coeff_limit;
  //}

  assert(ndofs == path.dof());
  for (std::size_t i = 0; i < N_1; ++i) {
    a[i] = path.eval_single(times[i], 1);
    assert(a[i].size() == ndofs);
    b[i] = path.eval_single(times[i], 2);
    assert(b[i].size() == ndofs);
    c[i].setZero();
  }
}

} // namespace constraint
} // namespace toppra
