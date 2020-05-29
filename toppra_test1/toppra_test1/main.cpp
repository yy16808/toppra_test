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

	scara_joint_torque s_j_t(vd1,vd2,vd3);

	std::cout << vd1<< endl;

	return 0;
}