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
	Vectors SplinePath::eval(const Vector & positions, int order) const
	{
		return Vectors();
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


	void SplinePath::serialize(std::ostream &O) const {
#ifdef TOPPRA_OPT_MSGPACK
		MatricesData allraw;
		allraw.reserve(m_coefficients.size());
		for (const auto &c : m_coefficients) {
			MatrixData raw{ c.rows(), c.cols(),{ c.data(), c.data() + c.size() } };
			allraw.push_back(raw);
		}
		msgpack::pack(O, allraw);
		msgpack::pack(O, m_breakpoints);
#endif
	};


	void SplinePath::deserialize(std::istream &I) {
#ifdef TOPPRA_OPT_MSGPACK
		std::stringstream buffer;
		buffer << I.rdbuf();
		std::size_t offset = 0;

		auto oh = msgpack::unpack(buffer.str().data(), buffer.str().size(), offset);
		auto obj = oh.get();
		TOPPRA_LOG_DEBUG(obj << "at offset:=" << offset << "/" << buffer.str().size());
		MatricesData x;
		toppra::Matrices new_coefficients;
		obj.convert(x);
		for (auto const &y : x) {
			int nrow, ncol;
			nrow = std::get<0>(y);
			ncol = std::get<1>(y);
			std::vector<value_type> mdata = std::get<2>(y);
			toppra::Matrix m(nrow, ncol);
			for (size_t i = 0; i < mdata.size(); i++) m(i) = mdata[i];
			TOPPRA_LOG_DEBUG(nrow << ncol << mdata.size() << m);
			new_coefficients.push_back(m);
		}

		reset();
		m_coefficients = new_coefficients;
		oh = msgpack::unpack(buffer.str().data(), buffer.str().size(), offset);
		obj = oh.get();
		TOPPRA_LOG_DEBUG(obj << "at offset:=" << offset << "/" << buffer.str().size());
		assert(offset == buffer.str().size());
		obj.convert(m_breakpoints);

		TOPPRA_LOG_DEBUG("degree: " << m_degree);
		m_dof = new_coefficients[0].cols();
		m_degree = new_coefficients[0].rows() - 1;
		checkInputArgs();
		computeDerivativesCoefficients();
#endif
	};
}