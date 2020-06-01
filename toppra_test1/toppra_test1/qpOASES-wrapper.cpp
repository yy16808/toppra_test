#include "qpOASES-wrapper.hpp"

#include <qpOASES.hpp>
#include "toppra.hpp"

namespace toppra {
namespace solver {
struct qpOASESWrapper::Impl {
  qpOASES::SQProblem qp;

  Impl(Eigen::Index nV, Eigen::Index nC)
    : qp (nV, nC)
  {
    qpOASES::Options options;
    options.printLevel = qpOASES::PL_NONE;

    qp.setOptions( options );
  }
};

value_type qpOASESWrapper::m_defaultBoundary = 1e16;

void qpOASESWrapper::setDefaultBoundary (const value_type& v)
{
  m_defaultBoundary = v;
}

qpOASESWrapper::qpOASESWrapper () {}

void qpOASESWrapper::initialize (const LinearConstraintPtrs& constraints, const GeometricPathPtr& path,
        const Vector& times)
{
  Solver::initialize (constraints, path, times);
  m_boundary = m_defaultBoundary;

  // Currently only support Canonical Linear Constraint
  Eigen::Index nC = 2; // First constraint is x + 2 D u <= xnext_max, second is xnext_min <= x + 2D u
  for (const Solver::LinearConstraintParams& linParam : m_constraintsParams.lin)
    nC += linParam.F[0].rows();

  Eigen::Index nV (nbVars());
  assert(nV == 2);
  m_A  = RMatrix::Zero(nC, nV);
  m_lA = -Vector::Ones(nC);
  m_hA = -Vector::Ones(nC);

  m_impl = std::unique_ptr<Impl>(new Impl(nV, nC));
}

qpOASESWrapper::~qpOASESWrapper ()
{
}

bool qpOASESWrapper::solveStagewiseOptim(std::size_t i,
        const Matrix& H, const Vector& g,
        const Bound& x, const Bound& xNext,
        Vector& solution)
{
  TOPPRA_LOG_DEBUG("stage: i="<<i);
  Eigen::Index N (nbStages());
  assert (i <= N);

  Bound l (Bound::Constant(-m_boundary)),
        h (Bound::Constant( m_boundary));

  l[1] = std::max(l[1], x[0]);
  h[1] = std::min(h[1], x[1]);

  if (i < N) {
    value_type delta = deltas()[i];
    m_A.row(0) << -2 * delta, -1;
    m_hA[0] = - xNext[0];
    m_lA[0] = - m_boundary;

    m_A.row(1) << 2 * delta, 1;
    m_hA[1] = xNext[1];
    m_lA[1] = -m_boundary;
  } else {
    m_A.topRows<2>().setZero();
    m_lA.head<2>().setConstant(-1);
    m_hA.head<2>().setOnes();
  }
  Eigen::Index cur_index = 2;
  for (const Solver::LinearConstraintParams& lin : m_constraintsParams.lin)
  {
    std::size_t j (lin.F.size() == 1 ? 0 : i);
    const Matrix& _F (lin.F[j]);
    const Vector& _g (lin.g[j]);
    Eigen::Index nC (_F.rows());

    m_A.block(cur_index, 0, nC, 1) = _F * lin.a[i];
    m_A.block(cur_index, 1, nC, 1) = _F * lin.b[i];
    m_hA.segment(cur_index, nC) = _g - _F * lin.c[i];
    m_lA.segment(cur_index, nC).setConstant(-m_boundary);
    cur_index += nC;
  }
  for (const Solver::BoxConstraintParams& box : m_constraintsParams.box)
  {
    if (!box.u.empty()) {
      l[0] = std::max(l[0], box.u[i][0]);
      h[0] = std::min(h[0], box.u[i][1]);
    }
    if (!box.x.empty()) {
      l[1] = std::max(l[1], box.x[i][0]);
      h[1] = std::min(h[1], box.x[i][1]);
    }
  }

  TOPPRA_LOG_DEBUG("lA: " << std::endl << m_lA);
  TOPPRA_LOG_DEBUG("hA: " << std::endl << m_hA);
  TOPPRA_LOG_DEBUG(" A: " << std::endl << m_A);
  TOPPRA_LOG_DEBUG("l : " << std::endl << l);
  TOPPRA_LOG_DEBUG("h : " << std::endl << h);

  qpOASES::returnValue res;
  // TODO I assumed 1000 is the argument nWSR of the SQProblem.init function.
  //res = self.solver.init(
  //    H, g, self._A, l, h, self._lA, self._hA, np.array([1000])
  //)
  int nWSR = 1000;
  if (H.size() == 0) {
    m_impl->qp.setHessianType(qpOASES::HST_ZERO);
    res = m_impl->qp.init (NULL, g.data(),
        m_A.data(),
        l.data(), h.data(),
        m_lA.data(), m_hA.data(),
        nWSR);
  } else {
    m_H = H; // Convert to row-major
    res = m_impl->qp.init (m_H.data(), g.data(),
        m_A.data(),
        l.data(), h.data(),
        m_lA.data(), m_hA.data(),
        nWSR);
  }

  if (res == qpOASES::SUCCESSFUL_RETURN) {
    solution.resize(nbVars());
    m_impl->qp.getPrimalSolution(solution.data());
    return true;
  }
  return false;
}

} // namespace solver
} // namespace toppra
