#include "scara_joint_torque.h"
#include "piecewise_poly_path.hpp"
#include "qpOASES-wrapper.hpp"

#include "toppra_alg.hpp"
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

	toppra::Matrix coeff{ 3, 2 };
	coeff(0, 0) = 0;
	coeff(0, 1) = 0;
	coeff(1, 0) = 14;
	coeff(1, 1) = 44;
	coeff(2, 0) = -132;
	coeff(2, 1) = -79;


	toppra::Matrices coefficents = { coeff, coeff };
	toppra::PiecewisePolyPath p =
		toppra::PiecewisePolyPath(coefficents, std::vector<double>{0, 1,2});

	toppra::Vector pos1 = p.eval_single(0.5, 0);
	toppra::Vector pos2 = p.eval_single(0.5, 1);

	std::shared_ptr<toppra::PiecewisePolyPath> path;
	path = std::make_shared<toppra::PiecewisePolyPath>(coefficents, std::vector<double>{0, 1,2});
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
	
	Vector parametrization=m_data.parametrization;

	Matrix controllable_sets= m_data.controllable_sets;
	double test = 0;
	for (int i = 0; i < 20; i++) {
		test=controllable_sets(i);
	}

		
	return 0;
}