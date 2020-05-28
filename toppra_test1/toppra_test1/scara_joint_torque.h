#pragma once
#include "joint_torque.hpp"
using namespace std;
class scara_joint_torque :
	public toppra::constraint::JointTorque
{
public:
	//scara_joint_torque();
	~scara_joint_torque();
};

