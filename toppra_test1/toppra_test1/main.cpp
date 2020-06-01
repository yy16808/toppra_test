#include "scara_joint_torque.h"
using namespace toppra;
using namespace constraint;
int main()
{
	//scara_joint_torque sjt;

	Vector vd1(3), vd2(3), vd3(3);
	vd1(0, 0) = 2;
	vd1(1, 0) = 3;
	vd1(2, 0) = 4.5;

	Vector scara_lowerTlimit(2), scara_uperTlimit(2), scara_frictionCoeffs(2);

	scara_lowerTlimit(0, 0) = -115;
	scara_lowerTlimit(1, 0) = -47;
	scara_uperTlimit(0, 0) = 115;
	scara_uperTlimit(1, 0) = 47;
	scara_frictionCoeffs(0, 0) = 0;
	scara_frictionCoeffs(1, 0) = 0;

	scara_joint_torque scara_trq(scara_lowerTlimit, scara_uperTlimit, scara_frictionCoeffs);

	//std::cout << vd1<< endl;

	return 0;
}