#include "scara_joint_torque.h"
#include "SplinePath.h"
#include "linear_joint_acceleration.hpp"
#include "piecewise_poly_path.hpp"
#include "qpOASES-wrapper.hpp"
#include "glpk-wrapper.hpp"
#include<windows.h>  
#include <fstream>
#include "toppra_alg.hpp"

using namespace toppra;
using namespace constraint;
#define PI 3.1415926

using vvvectors = std::vector<std::vector<value_type> >;
Vectors makeVectors(vvvectors v) {
	Vectors ret;
	for (auto vi : v) {
		Vector vi_eigen(vi.size());
		for (std::size_t i = 0; i < vi.size(); i++) vi_eigen(i) = vi[i];
		ret.push_back(vi_eigen);
	}
	return ret;
}

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
	const int size = 201; //N+1

	Vector vd1(3), vd2(3), vd3(3);
	vd1(0, 0) = 2;
	vd1(1, 0) = 3;
	vd1(2, 0) = 4.5;

	Vector scara_lowerTlimit(2), scara_uperTlimit(2), scara_frictionCoeffs(2);

	scara_lowerTlimit(0, 0) = -115;
	scara_lowerTlimit(1, 0) = -50;
	scara_uperTlimit(0, 0) = 115;
	scara_uperTlimit(1, 0) = 50;
	scara_frictionCoeffs(0, 0) = 0;
	scara_frictionCoeffs(1, 0) = 0;

	Vector acc_lowerTlimit(2), acc_uperTlimit(2);

	acc_lowerTlimit(0, 0) = -800 * deg2rad;
	acc_lowerTlimit(1, 0) = -800 * deg2rad;
	acc_uperTlimit(0, 0) = 800 * deg2rad;
	acc_uperTlimit(1, 0) = 800 * deg2rad;


	////////----------------------linear path-------------------------////////////
	//toppra::Matrix coeff1{ 3, 2 }, coeff2{ 3, 2 };
	//coeff1(0, 0) = 0;
	//coeff1(0, 1) = 0;
	//coeff1(1, 0) = 14 *deg2rad;
	//coeff1(1, 1) = 44 * deg2rad;
	//coeff1(2, 0) = -132 * deg2rad ;
	//coeff1(2, 1) = -79 * deg2rad ;

	//coeff2(0, 0) = 0;
	//coeff2(0, 1) = 0;
	//coeff2(1, 0) = -14 * deg2rad;
	//coeff2(1, 1) = -44 * deg2rad;
	//coeff2(2, 0) = -104* deg2rad;
	//coeff2(2, 1) =  9 * deg2rad;
	//toppra::Matrices coefficents = { coeff1};
	//std::shared_ptr<toppra::PiecewisePolyPath> path;
	//path = std::make_shared<toppra::PiecewisePolyPath>(coefficents, std::vector<double>{0,1});

	//////----------------------cubic path-------------------------////////////
	double pos_a1[6] = { -132 * deg2rad, -118 * deg2rad, -132 * deg2rad, -118 * deg2rad, -132 * deg2rad, -118 * deg2rad };
	double pos_a2[6] = { -79 * deg2rad, -35 * deg2rad, -79 * deg2rad, -35 * deg2rad, -79 * deg2rad, -35 * deg2rad };
	double time_a[6] = {0,0.2,0.4,0.6,0.8, 1.0};
	std::vector<std::vector<value_type>> positions;
	std::vector<value_type> times;
	//将数组的所有元素插入到vector中
	positions.resize(2);

	positions[0].insert(positions[0].begin(), pos_a1, pos_a1 + 6);
	positions[1].insert(positions[1].begin(), pos_a2, pos_a2 + 6);
	positions[0].resize(6);
	positions[1].resize(6);
	times.insert(times.begin(), time_a, time_a + 6);
	std::shared_ptr<toppra::SplinePath> path;
	path = std::make_shared<toppra::SplinePath>(positions, times);

	////////----------------------Hermite path-------------------------////////////
	//std::shared_ptr<toppra::PiecewisePolyPath> path;
	//
	//toppra::Vectors pos = makeVectors({ { -132 * deg2rad, -79 * deg2rad },{ -118 * deg2rad, -35 * deg2rad },{ -132 * deg2rad, -79 * deg2rad } });
	//toppra::Vectors vec = makeVectors({ { 0, 0 },{ 0, 0 },{ 0, 0 } });
	//toppra::PiecewisePolyPath path_Hermite =
	//	toppra::PiecewisePolyPath::constructHermite(pos, vec, { 0, 0.5, 1 });

	//path = std::make_shared<toppra::PiecewisePolyPath>(path_Hermite);




	toppra::LinearConstraintPtrs v;
	//v = toppra::LinearConstraintPtrs{ std::make_shared<toppra::constraint::LinearJointAcceleration>(acc_lowerTlimit, acc_uperTlimit) };

	v = toppra::LinearConstraintPtrs{std::make_shared<toppra::constraint::scara_joint_torque>(scara_lowerTlimit, scara_uperTlimit, scara_frictionCoeffs),
		std::make_shared<toppra::constraint::LinearJointAcceleration>(acc_lowerTlimit, acc_uperTlimit) };

		// (acc_lowerTlimit, acc_uperTlimit) -1000 * toppra::Vector::Ones(2), 1000 * toppra::Vector::Ones(2)
		
	algorithm::TOPPRA ta(v, path);

	std::shared_ptr<toppra::solver::qpOASESWrapper> qp_solver(new toppra::solver::qpOASESWrapper);
	ta.solver(qp_solver);
	//std::shared_ptr<toppra::solver::GLPKWrapper> glpk_solver(new toppra::solver::GLPKWrapper);
	//ta.solver(glpk_solver);
	
	ta.setN(size-1);
	ta.computePathParametrization();
	ParametrizationData m_data = ta.getParameterizationData();

	QueryPerformanceCounter(&nEndTime);//停止计时  
	time = (double)(nEndTime.QuadPart - nBeginTime.QuadPart) / (double)nFreq.QuadPart;//计算程序执行时间单位为s  
	cout << "程序执行时间：" << time * 1000 << "ms" << endl;

	//获取ds^2数值
	Vector parametrization=m_data.parametrization;
	for (int i = 0; i < size; i++) {
		cout << i << "  " << m_data.parametrization[i] << "  " << endl;
	}

	Matrix controllable_sets= m_data.controllable_sets;
	
	double acc[size], total = 0, av_vel[size], vel[size],r_time[size] = {0.0};
	double delta = 1.0 / (size-1), next_vel = 0;
	//计算s的速度、加速度
	for (int i = 0; i < size; i++) {
		
		if (i == size -1)
			acc[i] = (0 - parametrization(i)) / (2 * delta);
		else
			acc[i] = (parametrization(i + 1) - parametrization(i)) / (2 * delta);
		//total += acc[i];
		if (i != size -1)
			next_vel = sqrt(max(0,parametrization(i + 1)));
		else
			next_vel = 0;

		av_vel[i] = (sqrt(max(0, parametrization(i)))+ next_vel)/2;

		if (parametrization(i)>=0)
			vel[i] = sqrt(parametrization(i));
		else
			vel[i] = -sqrt(-parametrization(i));
		 
		if (i==0)
			r_time[i] = 0;
		else
			r_time[i]= r_time[i-1]+delta / av_vel[i-1];

		total+= delta / av_vel[i];

		cout <<i << "  " << acc[i] <<"  "<< vel[i] << "  " << av_vel[i] << " "
			<< r_time[i] << "  " << i*1.0/(size-1)<<endl;
	}
	//计算路径q的位置、速度、加速度
	double q1_acc[size], q2_acc[size], q1_vel[size], q2_vel[size], q1_pos[size], q2_pos[size];
	//for (int i = 0; i < size; i++) {
	//	q1_pos[i] = path->spline_instances[0](i*1.0 / (size - 1));
	//	q2_pos[i] = path->spline_instances[1](i*1.0 / (size - 1));

	//	q1_vel[i] = path->spline_instances[0].deriv(1, i*1.0 / (size - 1))*vel[i];
	//	q2_vel[i] = path->spline_instances[1].deriv(1, i*1.0 / (size - 1))*vel[i];

	//	q1_acc[i] = path->spline_instances[0].deriv(2, i*1.0 / (size - 1))*parametrization(i)+
	//		path->spline_instances[0].deriv(1, i*1.0 / (size - 1))*acc[i];
	//	q2_acc[i] = path->spline_instances[1].deriv(2, i*1.0 / (size - 1))*parametrization(i) +
	//		path->spline_instances[1].deriv(1, i*1.0 / (size - 1))*acc[i];

	//}

	//保存数据到文件中
	std::ofstream outFile;
	//打开文件
	outFile.open("../../toppra_matlab/Test.txt");//ios::app 
	for (int i = 0; i < size; i++)
	{
		////写入数据
		//if (i==0)
		//	outFile << acc[i] << "  " << vel[i] << "  " << 0 <<" "<<  0 << "\n";
		//else
		//	outFile << acc[i] << "  " << vel[i] << "  " << i*delta << "  " << r_time[i] << "\n";
		outFile << acc[i] << "  " << vel[i] << "  " << i*delta << "  " << r_time[i] << 
			"  " << q1_acc[i] << "  " << q1_vel[i] << "  " << q1_pos[i] <<
			"  " << q2_acc[i] << "  " << q2_vel[i] << "  " << q2_pos[i] << "\n";
		
	}
	//关闭文件
	outFile.close();

	return 0;
}