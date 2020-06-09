#include "scara_joint_torque.h"




namespace toppra {
	namespace constraint {


		scara_joint_torque::~scara_joint_torque()
		{
		}

		void scara_joint_torque::computeInverseDynamics(const Vector& q, const Vector& v, const Vector& a, Vector& tau)
		{
			double mp = 1,   L1 = 0.225, L2 = 0.275;
			double P1 = 1.0602+mp*L1*L1, P2 = 0.2433+mp*L2*L2, P3 = 0.2438 + mp*L1*L2, Jm1 = 0.6496;
			double ddq1, ddq2, dq1, dq2, q1, q2;
			ddq1 = a(0, 0);
			ddq2 = a(1, 0);
			dq1 = v(0, 0);
			dq2 = v(1, 0);
			q1 = q(0, 0);
			q2 = q(1, 0);
			tau(0, 0) = (P1 - Jm1)*ddq1 + P2*(ddq1 + ddq2) + P3*((2 * ddq1 + ddq2)*cos(q2) - (dq2*dq2 + 2 * dq1*dq2)*sin(q2));
			tau(1, 0) = P2*(ddq1 + ddq2) + P3*(ddq1*cos(q2)+dq1*dq1*sin(q2));
			//tau(0, 0) = (P1 - Jm1)*a(0, 0) + P2*(a(0, 0) + a(1, 0)) +
			//	P3*((2*a(0, 0)+ a(1, 0))*cos(q(1,0))-(v(1,0)*v(1, 0)+ 2*v(0, 0)*v(1, 0))*sin(q(1, 0)));
			//
			//tau(1, 0) = P2*(a(0, 0) + a(1, 0)) + P3*(a(0, 0)*cos(q(1, 0))+ v(0, 0)*v(0, 0)*sin(q(1, 0)));
		}
} // namespace constraint
} // namespace toppra