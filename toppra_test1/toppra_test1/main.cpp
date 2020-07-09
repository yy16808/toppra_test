#include "scara_joint_torque.h"
#include "SplinePath.h"
#include "linear_joint_acceleration.hpp"
#include "piecewise_poly_path.hpp"
#include "qpOASES-wrapper.hpp"
#include "glpk-wrapper.hpp"
#include<windows.h>  
#include <fstream>
#include <math.h>
#include "toppra_alg.hpp"

using namespace toppra;
using namespace constraint;
using namespace std;
#define PI 3.1415926
enum ProfileType
{
	Sin = 0, Cos, Poly5, Poly7
};

void GetProfilePts(double startpt, double endpt, bool header_mode, ProfileType profile_t, int data_size, vector<vector<double>> &pts, double para_range = 1.0, double para_start = 0.0) {
	double delta = endpt - startpt;
	int size = 0;
	double t = 0.0;
	double q = 0.0;
	int start_index = 0;

	pts.resize(2);
	size = data_size; //�ֳ�size�Σ���size+1������

	if (profile_t = Poly5) {
		if (header_mode == true)
			start_index = 0;	//���������startpt
		else
			start_index = 1;	//������㲻��startpt

		double size_factor = 1.0 / size;

		for (int i = start_index; i <= size; i++) {
			t = i*size_factor;
			pts[0].push_back(t*para_range + para_start);
			q = 10 * pow(t, 3) - 15 * pow(t, 4) + 6 * pow(t, 5);
			pts[1].push_back(q);
		}
	}

}

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
	QueryPerformanceCounter(&nBeginTime);//��ʼ��ʱ  


	double  deg2rad = PI / 180;
	double  rad2deg = 180 / PI;


	Vector vd1(3), vd2(3), vd3(3);
	vd1(0, 0) = 2;
	vd1(1, 0) = 3;
	vd1(2, 0) = 4.5;

	//scara_joint_torque sjt;
	Vector scara_lowerTlimit(2), scara_uperTlimit(2), scara_frictionCoeffs(2);

	scara_lowerTlimit(0, 0) = -115;
	scara_lowerTlimit(1, 0) = -47;
	scara_uperTlimit(0, 0) = 115;
	scara_uperTlimit(1, 0) = 47;
	scara_frictionCoeffs(0, 0) = 0;
	scara_frictionCoeffs(1, 0) = 0;

	Vector acc_lowerTlimit(2), acc_uperTlimit(2);

	//good acc limit
	//acc_lowerTlimit(0, 0) = -20000 * deg2rad;
	//acc_lowerTlimit(1, 0) = -20000 * deg2rad;
	//acc_uperTlimit(0, 0) = 10000 * deg2rad;
	//acc_uperTlimit(1, 0) = 10000 * deg2rad;
	//bad acc limit
	acc_lowerTlimit(0, 0) = -10000 * deg2rad;
	acc_lowerTlimit(1, 0) = -10000 * deg2rad;
	acc_uperTlimit(0, 0) = 8000 * deg2rad;
	acc_uperTlimit(1, 0) = 8000 * deg2rad;

	/*acc_lowerTlimit(0, 0) = -20000 * deg2rad;
	acc_lowerTlimit(1, 0) = -51000 * deg2rad;
	acc_uperTlimit(0, 0) = 20000 * deg2rad;
	acc_uperTlimit(1, 0) = 50000 * deg2rad;*/


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
	//double pos_a1[6] = { -132 * deg2rad, -118 * deg2rad, -132 * deg2rad, -118 * deg2rad, -132 * deg2rad, -118 * deg2rad };
	//double pos_a2[6] = { -79 * deg2rad, -35 * deg2rad, -79 * deg2rad, -35 * deg2rad, -79 * deg2rad, -35 * deg2rad };
	//double time_a[6] = { 0,0.2,0.4,0.6,0.8,1.0 };
	const int array_size = 11;
	double a1_start = 6.575*deg2rad, a1_mid = 16.5*deg2rad, a1_end = 7.965*deg2rad;
	double a2_start = -131.4*deg2rad, a2_mid = -103 * deg2rad, a2_end = -65.59*deg2rad;

	double pos_a1[array_size] = { a1_start ,a1_mid ,a1_end, a1_mid , a1_start,a1_mid ,a1_end, a1_mid , a1_start ,a1_mid ,a1_end };
	double pos_a2[array_size] = { a2_start ,a2_mid ,a2_end, a2_mid , a2_start,a2_mid ,a2_end, a2_mid , a2_start ,a2_mid ,a2_end };
	double time_a[array_size] = { 0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9, 1.0 };
	std::vector<std::vector<value_type>> positions;
	std::vector<value_type> times;

	/*vector<vector<double>> a1_pts0,a1_pts1, a1_pts2, a1_pts3,a1_pts4,a1_pts5;
	vector<vector<double>> a2_pts0, a2_pts1, a2_pts2;
	vector<vector<double>> a2_pts0, a2_pts1, a2_pts3;
	double a1_start = 6.806*deg2rad, a1_middle = 16.5*deg2rad, a1_end = 7.965*deg2rad;
	double a2_start = -130.9*deg2rad, a2_end = -65.59*deg2rad;
	double piece = 1.0 / 6;
	GetProfilePts(6.806*deg2rad, 16.5*deg2rad, true, Poly5, 5, a1_pts0, piece,0);
	GetProfilePts(16.5*deg2rad, 7.965*deg2rad, false, Poly5, 5, a1_pts1, piece, piece);
	GetProfilePts( 7.965*deg2rad, 16.5*deg2rad, false, Poly5, 5, a1_pts2, piece, 2* piece);
	GetProfilePts(16.5*deg2rad, 6.806*deg2rad, false, Poly5, 5, a1_pts3, piece, 3 * piece);
	GetProfilePts( 6.806*deg2rad, 16.5*deg2rad, false, Poly5, 5, a1_pts4, piece, 4 * piece);
	GetProfilePts(16.5*deg2rad, 7.965*deg2rad, false, Poly5, 5, a1_pts1, piece, 5 * piece);

	GetProfilePts(a2_start, a2_end, true, Poly5, 10, a2_pts0, 2 * piece, 0);
	GetProfilePts(a2_end, a2_start, false, Poly5, 10, a2_pts1, 2 * piece, 2*piece);
	GetProfilePts(a2_start, a2_end,  false, Poly5, 10, a2_pts2, 2 * piece, 4 * piece);*/

	//�����������Ԫ�ز��뵽vector��
	positions.resize(2);

	positions[0].insert(positions[0].begin(), pos_a1, pos_a1 + array_size);
	positions[1].insert(positions[1].begin(), pos_a2, pos_a2 + array_size);
	positions[0].resize(array_size);
	positions[1].resize(array_size);
	times.insert(times.begin(), time_a, time_a + array_size);
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

	//v = toppra::LinearConstraintPtrs{std::make_shared<toppra::constraint::scara_joint_torque>(scara_lowerTlimit, scara_uperTlimit, scara_frictionCoeffs),
	//	std::make_shared<toppra::constraint::LinearJointAcceleration>(acc_lowerTlimit, acc_uperTlimit) };

	// (acc_lowerTlimit, acc_uperTlimit) -1000 * toppra::Vector::Ones(2), 1000 * toppra::Vector::Ones(2)
	v = toppra::LinearConstraintPtrs{ std::make_shared<toppra::constraint::scara_joint_torque>(scara_lowerTlimit, scara_uperTlimit, scara_frictionCoeffs),
		std::make_shared<toppra::constraint::LinearJointAcceleration>(acc_lowerTlimit, acc_uperTlimit) };
	v[0]->discretizationType(Interpolation);
	algorithm::TOPPRA ta(v, path);

	//���������
	//std::shared_ptr<toppra::solver::qpOASESWrapper> qp_solver(new toppra::solver::qpOASESWrapper);
	//ta.solver(qp_solver);
	std::shared_ptr<toppra::solver::GLPKWrapper> glpk_solver(new toppra::solver::GLPKWrapper);
	ta.solver(glpk_solver);

	const int size = 101; //N+1 ����gridpoints size
	ta.setN(size - 1);
	ta.computePathParametrization();
	ParametrizationData m_data = ta.getParameterizationData();

	QueryPerformanceCounter(&nEndTime);//ֹͣ��ʱ  
	time = (double)(nEndTime.QuadPart - nBeginTime.QuadPart) / (double)nFreq.QuadPart;//�������ִ��ʱ�䵥λΪs  
	cout << "����ִ��ʱ�䣺" << time * 1000 << "ms" << endl;

	//��ȡds^2��ֵ
	Vector parametrization = m_data.parametrization;
	//for (int i = 0; i < size; i++) {
	//	cout << i << "  " << m_data.parametrization[i] << "  " << endl;
	//}

	Matrix controllable_sets = m_data.controllable_sets;

	double acc[size], total = 0, av_vel[size], vel[size], r_time[size] = { 0.0 };
	double delta = 1.0 / (size - 1), last_vel = 0;

	std::vector<double>  vec_q1_pos, vec_q2_pos, vec_time;
	//����s���ٶȡ����ٶ�
	for (int i = 0; i < size; i++) {

		if (i == size-1)
			acc[i] = (0-parametrization(i)) / (2 * delta);
		else
			acc[i] = (parametrization(i+1) - parametrization(i)) / (2 * delta);

		if (i != 0)
			last_vel = sqrt( parametrization(i - 1));
		else
			last_vel = 0;

		vel[i] = sqrt(parametrization(i));

		av_vel[i] = (vel[i] + last_vel) / 2;

		if (i == 0)
			r_time[i] = 0;
		else
			r_time[i] = r_time[i - 1] + delta / av_vel[i];

		vec_time.push_back(r_time[i]);

		total += delta / av_vel[i];

		// cout << i <<  "�ٶȣ�  " << vel[i]*rad2deg << "  " << endl;
		//cout <<i << "  " << acc[i] <<"  "<< vel[i] << "  " << av_vel[i] << " "
		//	<< r_time[i] << "  " << i*1.0/(size-1)<<endl;
	}
	//����·��q��λ�á��ٶȡ����ٶ�
	double q1_acc[size], q2_acc[size], q1_vel[size], q2_vel[size], q1_pos[size], q2_pos[size];

	for (int i = 0; i < size; i++) {
		q1_pos[i] = path->spline_instances[0](i*1.0 / (size - 1));
		q2_pos[i] = path->spline_instances[1](i*1.0 / (size - 1));
		vec_q1_pos.push_back(q1_pos[i]);
		vec_q2_pos.push_back(q2_pos[i]);

		q1_vel[i] = path->spline_instances[0].deriv(1, i*1.0 / (size - 1))*vel[i];
		q2_vel[i] = path->spline_instances[1].deriv(1, i*1.0 / (size - 1))*vel[i];

		q1_acc[i] = path->spline_instances[0].deriv(2, i*1.0 / (size - 1))*parametrization(i) +
			path->spline_instances[0].deriv(1, i*1.0 / (size - 1))*acc[i];
		q2_acc[i] = path->spline_instances[1].deriv(2, i*1.0 / (size - 1))*parametrization(i) +
			path->spline_instances[1].deriv(1, i*1.0 / (size - 1))*acc[i];

		cout << i << "�����ٶȣ�  " << path->spline_instances[1].deriv(1, i*1.0 / (size - 1))<< "  " 
			<< "�����ٶ�  " << q2_vel[i]*rad2deg << "S�ٶ�" << vel[i] * rad2deg << "  " << endl;

	}
	std::ofstream outFile;
	//���ļ�
	outFile.open("../../toppra_matlab/Test.txt");//ios::app 
	for (int i = 0; i < size; i++)
	{
		////д������
		//if (i==0)
		//	outFile << acc[i] << "  " << vel[i] << "  " << 0 <<" "<<  0 << "\n";
		//else
		//	outFile << acc[i] << "  " << vel[i] << "  " << i*delta << "  " << r_time[i] << "\n";
		outFile << acc[i] << "  " << vel[i] << "  " << i*delta << "  " << r_time[i] <<
			"  " << q1_acc[i] << "  " << q1_vel[i] << "  " << q1_pos[i] <<
			"  " << q2_acc[i] << "  " << q2_vel[i] << "  " << q2_pos[i] << "\n";

	}
	//�ر��ļ�
	outFile.close();

	
	// ���²�ֵ��������

	//vec_q1_pos(size), vec_q2_pos(size), vec_time(size)
	tk::spline q1_trj, q2_trj;
	q1_trj.set_boundary(q1_trj.first_deriv, 0.0, q1_trj.first_deriv, 0.0, false);
	q1_trj.set_points(vec_time, vec_q1_pos);    // currently it is required that X is already sorted

	q2_trj.set_boundary(q2_trj.first_deriv, 0.0, q2_trj.first_deriv, 0.0, false);
	q2_trj.set_points(vec_time, vec_q2_pos);    // currently it is required that X is already sorted

												//�������ݵ��ļ���
	//std::ofstream outFile;
	//���ļ�
	outFile.open("../../toppra_matlab/reCubic.txt");//ios::app 
	double xx = 0;
	for (int i = 0; i < size; i++)
	{
		xx = i*1.0 / (size - 1)*vec_time[size - 1];
		////д������
		//if (i==0)
		//	outFile << acc[i] << "  " << vel[i] << "  " << 0 <<" "<<  0 << "\n";
		//else
		//	outFile << acc[i] << "  " << vel[i] << "  " << i*delta << "  " << r_time[i] << "\n";

		outFile << xx << "  " << q1_trj(xx) << "  " << q1_trj.deriv(1,xx) << "  " << q1_trj.deriv(2, xx) <<
			"  " << q2_trj(xx) << "  " << q2_trj.deriv(1, xx) << "  " << q2_trj.deriv(2, xx) << "\n";

	}
	//�ر��ļ�
	outFile.close();

	

	return 0;
}