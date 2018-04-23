#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

size_t N = 10;
double dt = 0.2;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

static CppAD::AD<double> polyEvalAd(Eigen::VectorXd coeffs, CppAD::AD<double> x) {
  CppAD::AD<double> rv = 0;
  for (int i = 0; i < coeffs.size(); i++) {
    rv += coeffs[i] * CppAD::pow(x, i);
  }

  return rv;
}

static CppAD::AD<double> polyDerivEvalAd(Eigen::VectorXd coeffs, CppAD::AD<double> x) {
  // evaluate derivative of polynomial
  CppAD::AD<double> rv = 0;
  for (int i = 1; i < coeffs.size(); i++) {
    rv += coeffs[i] * CppAD::pow(x, i - 1) * i;
  }

  return rv;
}

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // helper functions for getting variables
    auto x = [&](int i) -> auto& { return vars[i]; };
    auto y = [&](int i) -> auto& { return vars[N + i]; };
    auto psi = [&](int i) -> auto& { return vars[2 * N + i]; };
    auto v = [&](int i) -> auto& { return vars[3 * N + i]; };
    auto cte = [&](int i) -> auto& { return vars[4 * N + i]; };
    auto epsi = [&](int i) -> auto& { return vars[5 * N + i]; };
    auto delta = [&](int i) -> auto& { return vars[6 * N + i]; };
    auto a = [&](int i) -> auto& { return vars[7 * N - 1 + i]; };

    // helper functions for getting constraints
    auto cx = [&](int i) -> auto& { return fg[i + 1]; };
    auto cy = [&](int i) -> auto& { return fg[N + i + 1]; };
    auto cpsi = [&](int i) -> auto& { return fg[2 * N + i + 1]; };
    auto cv = [&](int i) -> auto& { return fg[3 * N + i + 1]; };
    auto ccte = [&](int i) -> auto& { return fg[4 * N + i + 1]; };
    auto cepsi = [&](int i) -> auto& { return fg[5 * N + i + 1]; };

    // define cost function
    // add cost for cross-track error and heading error
    for (int i = 0; i < N; i++) {
      fg[0] += CppAD::pow(cte(i), 2);
      fg[0] += CppAD::pow(epsi(i), 2);
    }

    // add cost for not moving at 20 kmph
    for (int i = 0; i < N; i++) {
      fg[0] += CppAD::pow(v(i) - 20, 2);
    }

    // add cost for high steering angle
    for (int i = 0; i < N - 1; i++) {
      fg[0] += CppAD::pow(delta(i), 2);
    }

    // add cost for high acceleration
    for (int i = 0; i < N - 1; i++) {
      fg[0] += 10 * CppAD::pow(a(i), 2);
    }

    // define constraints
    // initial constraints
    cx(0) = x(0);
    cy(0) = y(0);
    cpsi(0) = psi(0);
    cv(0) = v(0);
    ccte(0) = cte(0);
    cepsi(0) = epsi(0);

    // subsequent constraints
    for (int i = 0; i < N - 1; i++) {
      cx(i + 1) = x(i + 1) - (
          x(i) + v(i) * CppAD::cos(psi(i)) * dt);
      cy(i + 1) = y(i + 1) - (
          y(i) + v(i) * CppAD::sin(psi(i)) * dt);
      // note minus instead of plus in this expression
      // because positive steering input *decreases*
      // the bearing angle
      cpsi(i + 1) = psi(i + 1) - (
          psi(i) - v(i) / Lf * delta(i) * dt);
      cv(i + 1) = v(i + 1) - (
          v(i) + a(i) * dt);
      // TODO this is different from what's given in the
      // lesson, but it should still work right?
      ccte(i + 1) = cte(i + 1) - (
          y(i + 1) - polyEvalAd(coeffs, x(i + 1)));
      cepsi(i + 1) = epsi(i + 1) - (
          psi(i + 1) - CppAD::atan(polyDerivEvalAd(coeffs, x(i + 1))));
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  const size_t n_state_vars = 6;  // x, y, psi, v, cte, epsi
  const size_t n_actuator_vars = 2;
  size_t n_vars = n_state_vars * N + n_actuator_vars * (N - 1);

  // one constraint per timestep for each state_var
  size_t n_constraints = n_state_vars * N;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  // initial x
  vars[0] = state[0];
  // initial y
  vars[N] = state[1];
  // initial psi
  vars[2 * N] = state[2];
  // initial v
  vars[3 * N] = state[3];
  // initial cte
  vars[4 * N] = state[4];
  // initial epsi
  vars[5 * N] = state[5];

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  for (int i = 0; i < N; i++) {
    // minimum x = -infinity
    vars_lowerbound[i] = -1e10;
    // maximum x = infinity
    vars_upperbound[i] = 1e10;
    // minimum y = -infinity
    vars_lowerbound[N + i] = -1e10;
    // maximum y = infinity
    vars_upperbound[N + i] = 1e10;
    // minimum psi = -infinity
    vars_lowerbound[2 * N + i] = -1e10;
    // maximum psi = infinity
    vars_upperbound[2 * N + i] = 1e10;
    // minimum v = 0
    vars_lowerbound[3 * N + i] = 0;
    // maximum v = 100
    vars_upperbound[3 * N + i] = 100;
    // minimum cte = -infinity
    vars_lowerbound[4 * N + i] = -1e10;
    // maximum cte = infinity
    vars_upperbound[4 * N + i] = 1e10;
    // minimum epsi = -infinity
    vars_lowerbound[5 * N + i] = -1e10;
    // maximum epsi = infinity
    vars_upperbound[5 * N + i] = 1e10;
    if (i < N - 1) {
      // minimum delta = -25 degrees
      vars_lowerbound[6 * N + i] = -25.0 / 180 * M_PI;
      // maximum delta = 25 degrees
      vars_upperbound[6 * N + i] = 25.0 / 180 * M_PI;
      // minimum a = -1
      vars_lowerbound[7 * N - 1 + i] = -1;
      // maximum a = 1
      vars_upperbound[7 * N - 1 + i] = 1;
    }
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  // initial x
  constraints_lowerbound[0] = constraints_upperbound[0] = state[0];
  // initial y
  constraints_lowerbound[N] = constraints_upperbound[N] = state[1];
  // initial psi
  constraints_lowerbound[2 * N] = constraints_upperbound[2 * N] = state[2];
  // initial v
  constraints_lowerbound[3 * N] = constraints_upperbound[3 * N] = state[3];
  // initial cte
  constraints_lowerbound[4 * N] = constraints_upperbound[4 * N] = state[4];
  // initial epsi
  constraints_lowerbound[5 * N] = constraints_upperbound[5 * N] = state[5];

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  if (solution.status != CppAD::ipopt::solve_result<Dvector>::success) {
    std::cerr << "Error! CppAD::ipopt::solve failed to find solution!\n";
  }

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  std::cerr << "Optimal delta: " << solution.x[6 * N] << "; optimal a: " << solution.x[7 * N - 1] << '\n';
  return {solution.x[6 * N], solution.x[7 * N - 1]};
}
