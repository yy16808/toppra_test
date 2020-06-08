#include "scara_joint_torque.h"
#include "piecewise_poly_path.hpp"
#include "qpOASES-wrapper.hpp"
#include<windows.h>  
#include <fstream>
#include "toppra_alg.hpp"
using namespace toppra;
using namespace constraint;
#define PI 3.1415926
int main()
{
	double time = 0;
	double counts = 0;
	LARGE_INTEGER nFreq;
	LARGE_INTEGER nBeginTime;
	LARGE_INTEGER nEndTime;
	QueryPerformanceFrequency(&nFreq);
	QueryPerformanceCounter(&nBeginTime);//开始计时  

	//scara_joint_torque sjt;
	double  deg2rad = PI / 180;
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

	/*scara_joint_torque scara_trq(scara_lowerTlimit, scara_uperTlimit, scara_frictionCoeffs);
	scara_trq.print(cout);*/

	toppra::Matrix coeff{ 3, 2 };
	coeff(0, 0) = 0;
	coeff(0, 1) = 0;
	coeff(1, 0) = 14* deg2rad;
	coeff(1, 1) = 44 * deg2rad;
	coeff(2, 0) = -132 * deg2rad;
	coeff(2, 1) = -79 * deg2rad;

	//coeff(0, 0) = -14 * deg2rad;
	//coeff(0, 1) = -44 * deg2rad;
	//coeff(1, 0) = 28* deg2rad;
	//coeff(1, 1) = 88 * deg2rad;
	//coeff(2, 0) = -132 * deg2rad;
	//coeff(2, 1) = -79 * deg2rad;

	//coeff(0, 0) = 0;
	//coeff(0, 1) = 0;
	//coeff(1, 0) = 1;
	//coeff(1, 1) = 1;
	//coeff(2, 0) = 2;
	//coeff(2, 1) = 2;
	//coeff(3, 0) = 3;
	//coeff(3, 1) = 3;

	toppra::Matrices coefficents = { coeff};
	//toppra::PiecewisePolyPath p =
	//	toppra::PiecewisePolyPath(coefficents, std::vector<double>{0, 1,2});

//	toppra::Vector pos1 = p.eval_single(0.5, 0);
//	toppra::Vector pos2 = p.eval_single(0.5, 1);

	std::shared_ptr<toppra::PiecewisePolyPath> path;
	path = std::make_shared<toppra::PiecewisePolyPath>(coefficents, std::vector<double>{0, 1});
 	//toppra::PiecewisePolyPath path =
		//toppra::PiecewisePolyPath(coefficents, std::vector<double>{0, 1, 2});

	toppra::LinearConstraintPtrs v;

	v = toppra::LinearConstraintPtrs{ std::make_shared<toppra::constraint::scara_joint_torque>(scara_lowerTlimit, scara_uperTlimit, scara_frictionCoeffs) };

	algorithm::TOPPRA ta(v, path);

	std::shared_ptr<toppra::solver::qpOASESWrapper> qp_solver(new toppra::solver::qpOASESWrapper);

	ta.solver(qp_solver);
	ta.setN(300);
	ta.computePathParametrization();
	ParametrizationData m_data = ta.getParameterizationData();

	QueryPerformanceCounter(&nEndTime);//停止计时  
	time = (double)(nEndTime.QuadPart - nBeginTime.QuadPart) / (double)nFreq.QuadPart;//计算程序执行时间单位为s  
	cout << "程序执行时间：" << time * 1000 << "ms" << endl;

	
	Vector parametrization=m_data.parametrization;

	Matrix controllable_sets= m_data.controllable_sets;
	const int size = 301;
	double acc[size], total = 0, av_vel[size], vel[size],r_time[size] = {0.0};
	double delta = 1.0 / 300, next_vel = 0;

	for (int i = 0; i < size; i++) {
		
		if (i == size-1)
			acc[i] = acc[i - 1];
		else
			acc[i] = (parametrization(i + 1) - parametrization(i)) / (2 * delta);
		//total += acc[i];
		if (i != 300)
			next_vel = sqrt(parametrization(i + 1));
		else
			next_vel = 0;

		av_vel[i] = (sqrt(parametrization(i))+ next_vel)/2;
		vel[i] = sqrt(parametrization(i));

		if (i==0)
			r_time[i] = 0;
		else
			r_time[i]= r_time[i-1]+delta / av_vel[i-1];

		total+= delta / av_vel[i];

		cout <<i << "  " << acc[i] <<"  "<< vel[i] << "  " << av_vel[i] << " "
			<< r_time[i] <<endl;
	}

	std::ofstream outFile;
	//打开文件
	outFile.open("Test.txt");//ios::app 
	for (int i = 0; i < size; i++)
	{
		//写入数据
		if (i==0)
			outFile << acc[i] << "  " << vel[i] << "  " << 0 <<" "<<  0 << "\n";
		else
			outFile << acc[i] << "  " << vel[i] << "  " << i*delta << "  " << r_time[i] << "\n";
		
	}
	//关闭文件
	outFile.close();

	return 0;
}