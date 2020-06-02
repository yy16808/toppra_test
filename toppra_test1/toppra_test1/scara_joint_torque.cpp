#include "scara_joint_torque.h"




namespace toppra {
	namespace constraint {


		scara_joint_torque::~scara_joint_torque()
		{
		}

		void scara_joint_torque::computeInverseDynamics(const Vector& q, const Vector& v, const Vector& a, Vector& tau)
		{
			double P1 = 1.0602, P2 = 0.2433, P3 = 0.2438, Jm1 = 0.6496;
			tau(0, 0) = (P1 - Jm1)*a(0, 0) + P2*(a(0, 0) + a(1, 0)) +
				P3*((2*a(0, 0)+ a(1, 0))*cos(q(1,0))-(v(1,0)*v(1, 0)+ 2*v(0, 0)*v(1, 0))*sin(q(1, 0)));
			
			tau(1, 0) = P2*(a(0, 0) + a(1, 0)) + P3*(a(0, 0)*cos(q(1, 0))+ v(0, 0)*v(0, 0)*sin(q(1, 0)));
		}
} // namespace constraint
} // namespace toppra