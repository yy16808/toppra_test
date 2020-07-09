#pragma once
#include "geometric_path.hpp"
#include "toppra.hpp"
#include "spline.h"
namespace toppra {
	class SplinePath :
		public GeometricPath
	{
	public:
		SplinePath();
		/**
		* \brief Construct the cubic path.
		*/
		SplinePath(const std::vector<std::vector<value_type>> &positions,const std::vector<value_type> &times);

		~SplinePath();
		/**
		* /brief Evaluate the path at given position.
		*/
		Vector eval_single(value_type pos, int order = 0) const;
		/**
		* /brief Evaluate the path at given positions (vector).
		*
		* Default implementation: Evaluation each point one-by-one.
		*/
		virtual Vectors eval(const Vector &positions, int order = 0) const;

		void serialize(std::ostream &O) const override;
		void deserialize(std::istream &I) override;

		/**
		* Return the starting and ending path positions.
		*/
		Bound pathInterval() const;
		void reset();
	protected:
		std::vector<value_type> m_breakpoints;
		//Matrices m_coefficients, m_coefficients_1, m_coefficients_2;
		int m_degree;
		//tk::spline spline_instance[3];
	public:
		std::vector<tk::spline> spline_instances;
	};
}
