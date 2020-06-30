#include "SplinePath.h"
#include "geometric_path.hpp"
#include "toppra.hpp"

namespace toppra {

	SplinePath::SplinePath()
	{
	}


	SplinePath::SplinePath(const std::vector<std::vector<value_type>> &positions, 
		const std::vector<value_type> &times) : GeometricPath(positions.size())
	{
		reset();
		assert(positions[0].size() == times.size());
		TOPPRA_LOG_DEBUG("Constructing new cubic polynomial");
		m_dof = positions.size();
		m_breakpoints = times;
		spline_instances.resize(m_dof);
		for (int i = 0; i < m_dof; i++) {
			spline_instances[i].set_boundary(spline_instances[i].first_deriv, 0.0, 
				spline_instances[i].first_deriv, 0.0, false);

			spline_instances[i].set_points(times, positions[i]);
		}
		
	}

	SplinePath::~SplinePath()
	{
	}
	Vector SplinePath::eval_single(value_type pos, int order) const
	{
		assert(order < 3 && order >= 0);
		Vector v(m_dof);
		v.setZero();
		if (order == 0) {
			for (int i = 0; i < m_dof; i++) {
				v(i, 0) = spline_instances[i](pos);
			}
		}
		else {
			for (int i = 0; i < m_dof; i++) {
				v(i, 0) = spline_instances[i].deriv(order,pos);
			}

		}

		return v;
	}
	Bound SplinePath::pathInterval() const
	{
		Bound v;
		v << m_breakpoints.front(), m_breakpoints.back();
		return v;
	}

	void SplinePath::reset() {
		m_breakpoints.clear();

	}

}