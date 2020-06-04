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

	toppra::Matrix coeff{ 4, 2 };
	coeff(0, 0) = 0;
	coeff(0, 1) = 0;
	coeff(1, 0) = 1;
	coeff(1, 1) = 1;
	coeff(2, 0) = 2;
	coeff(2, 1) = 2;
	coeff(3, 0) = 3;
	coeff(3, 1) = 3;
	toppra::Matrices coefficents = { coeff, coeff };

	std::shared_ptr<toppra::PiecewisePolyPath> path;
	path = std::make_shared<toppra::PiecewisePolyPath>(coefficents, std::vector<double>{0, 1, 2});
 	//toppra::PiecewisePolyPath path =
		//toppra::PiecewisePolyPath(coefficents, std::vector<double>{0, 1, 2});

	toppra::LinearConstraintPtrs v;

	v = toppra::LinearConstraintPtrs{ std::make_shared<toppra::constraint::scara_joint_torque>(scara_lowerTlimit, scara_uperTlimit, scara_frictionCoeffs) };

	algorithm::TOPPRA ta(v, path);

	std::shared_ptr<toppra::solver::qpOASESWrapper> qp_solver(new toppra::solver::qpOASESWrapper);

	ta.solver(qp_solver);
	ta.setN(50);
	ta.computePathParametrization();

	


	return 0;
}