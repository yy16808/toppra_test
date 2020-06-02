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
			
		
			scara_joint_torque(const Vector& lowerTlimit, const Vector& uperTlimit, const Vector& frictionCoeffs)
				: JointTorque(lowerTlimit, uperTlimit,frictionCoeffs)
				//, m_model(model)
				//, m_data(model)
			{

			}

			~scara_joint_torque();
			
			void computeInverseDynamics(const Vector& q, const Vector& v, const Vector& a, Vector& tau);
		};

	}
}