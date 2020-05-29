#pragma once
#include "joint_torque.hpp"
#include "constraint.hpp"
#include <vector>
using namespace std;

namespace toppra {
	namespace constraint {
		class scara_joint_torque : public toppra::constraint::JointTorque
		{
		public:
			Vector m_lower, m_upper, m_frictionCoeffs;
		public:
			scara_joint_torque() {}

			//scara_joint_torque(const Vector& lowerTlimit, const Vector& upperTlimit,const Vector& frictionCoeffs)
			//	//: LinearConstraint(2 * lowerTlimit.size(), lowerTlimit.size(), true, false, false)
			//	: m_lower(lowerTlimit)
			//	, m_upper(upperTlimit)
			//	, m_frictionCoeffs(frictionCoeffs)
			//{
			//	this->check();
			//}

			~scara_joint_torque();
			void check();
			void computeInverseDynamics(const Vector& q, const Vector& v, const Vector& a, Vector& tau);
		};

	}
}