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

    // define constraints
    // initial constraints
    // TODO is this correct? I feel like we should be subtracting the `vars` value from the true current value
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

    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
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

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

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

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  return {};
}
