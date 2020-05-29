#include "scara_joint_torque.h"
using namespace toppra;
int main()
{
	//scara_joint_torque sjt;
	Vector vd(3);
	vd(0, 0) = 2;
	vd(1, 0) = 3;
	vd(2, 0) = 4.5;
	std::cout << vd << endl;

	return 0;
}