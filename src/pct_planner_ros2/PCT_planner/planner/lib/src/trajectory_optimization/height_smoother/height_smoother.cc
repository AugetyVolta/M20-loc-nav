#include "trajectory_optimization/height_smoother/height_smoother.h"

#include <algorithm>
#include <iostream>

#include "common/smoothing/osqp_spline1d_solver.h"

Eigen::VectorXd HeightSmoother::Smooth(const Eigen::VectorXd& coarse_height,
                                       const Eigen::VectorXd& upper_bound,
                                       const double dt, const int N,
                                       const double knot_interval) {
  std::vector<double> lbs;
  std::vector<double> ubs;
  std::vector<double> refs;
  std::vector<double> ts;
  std::vector<double> knots;

  for (int i = 0; i < N; ++i) {
    knots.emplace_back(i * knot_interval);
  }

  for (int i = 0; i < coarse_height.size(); ++i) {
    // Smooth the terrain profile without allowing the trajectory to cut below
    // the surface or float far above it. coarse_height already includes the
    // configured path-to-ground offset.
    const double lower = coarse_height(i) - 0.05;
    const double terrain_upper = coarse_height(i) + 0.20;
    const double ceiling_upper = upper_bound(i) - 0.3;
    lbs.emplace_back(lower);
    ubs.emplace_back(std::max(lower, std::min(terrain_upper, ceiling_upper)));
    refs.emplace_back(coarse_height(i));
    ts.emplace_back(i * dt);
  }

  common::OsqpSpline1dSolver solver(knots, 5);
  auto kernel = solver.mutable_kernel();
  kernel->AddRegularization(1e-5);
  // kernel->AddSecondOrderDerivativeMatrix(5);
  kernel->AddThirdOrderDerivativeMatrix(30);
  kernel->AddReferenceLineKernelMatrix(ts, refs, 1);
  auto constraint = solver.mutable_constraint();
  constraint->AddThirdDerivativeSmoothConstraint();
  constraint->AddPointConstraint(ts.front(), coarse_height(0));
  // constraint->AddPointDerivativeConstraint(t_knots_.front(), init_s_[1]);
  // constraint->AddPointSecondDerivativeConstraint(t_knots_.front(),
  // init_s_[2]);
  constraint->AddBoundary(ts, lbs, ubs);
  // constraint->AddDerivativeBoundary(t_samples, v_min, v_max);
  // constraint->AddSecondDerivativeBoundary(t_samples, a_min, a_max);

  if (!solver.Solve()) {
    // std::cout << "Fail to solve the spline" << std::endl;
    return coarse_height;
  }

  auto spline = solver.spline();
  Eigen::VectorXd smooth_height(coarse_height.size());
  for (int i = 0; i < coarse_height.size(); ++i) {
    smooth_height(i) = spline(ts[i]);
  }
  return smooth_height;
}
