/****************************************************************************/
/// @file    optimizeSpeedAcc.cc
 
 
/// @date    December 2023
///
/****************************************************************************/
#include "optimizeSpeedAcc.h"


FG_eval::FG_eval(double lambda1, double lambda2, double lambda3, double D, double v_0, double t_g)
    : lambda1(lambda1), lambda2(lambda2), lambda3(lambda3), D(D), v_0(v_0), t_g(t_g) {}


void FG_eval::operator()(ADvector& fg, const ADvector& x) const {
    assert(fg.size() == 3);  // Updated based on new constraint counts
    assert(x.size() == 2);

    // Extract variables from the input vector x
    CppAD::AD<double> a = x[0];
    CppAD::AD<double> v = x[1];


    CppAD::AD<double> d = (v * v - v_0 * v_0) / (2.0 * a) + v * (t_g - (v - v_0) / a);

    // VT-micro
    double VT_micro_M[4][4] = {VT_MICRO_M_VALUES};

    // Placeholder for sumMoe
    CppAD::AD<double> sumMOE = 0., MOE0 = 0., MOE1 = 0.;
    CppAD::AD<double> timeStepForT0 = 10 * (v - v_0) / a,
                      timeStepForT1 = 10 * (t_g - (v - v_0) / a);

    // Note from Speed: km/h; acceleration: km/h/s; CO2 emission rate: mg/s.
    for (int ts = 0; ts < timeStepForT0; ts++){
        for (int m = 0; m <= 3; ++m) {
            for (int n = 0; n <= 3; ++n) {
                MOE0 += VT_micro_M[m][n] * CppAD::pow((v_0 + a * ts)*3.6, m) * CppAD::pow(a*3.6, n);
            }
        }
    }
    MOE0 = 0.1 * CppAD::exp(MOE0);

    for (int i = 0; i <= 3; i++) {
        MOE1 += VT_micro_M[i][0] * CppAD::pow(v*3.6, i);
    }
    MOE1 = (t_g - (v - v_0) / a) * CppAD::exp(MOE1);

    sumMOE = MOE0 + MOE1;

    // Objective function
    fg[0] = lambda1 * sumMOE - lambda2 * v + lambda3 * CppAD::pow(D - d, 2);

    // Constraint 3: d < D
    fg[1] = D - d;

    // Constraint 4: (v - v_0) / a <= t_g
    fg[2] = v - v_0 - a * t_g;
}


void solveOptimizationProblem(double lambda1, double lambda2, double lambda3, double D, double v_0, double t_g, double& optimal_a, double& optimal_v) {
    // Initialize variables and set bounds
    size_t nx = 2;  // Number of variables (a and v)
    size_t ng = 2;  // Number of constraints
    CppAD::vector<double> x0(nx), xl(nx), xu(nx);
    CppAD::vector<double> gl(ng), gu(ng);

    // Initial guess
    x0[0] = -0.08;  // Initial guess for a
    x0[1] = 4.0;  // Initial guess for v

    // Variable bounds
    xl[0] = A_LOWER_BOUND;  // Lower bound for a
    xu[0] = A_UPPER_BOUND;  // Upper bound for a
    xl[1] = V_LOWER_BOUND;  // Lower bound for v
    xu[1] = V_UPPER_BOUND; // Upper bound for v

    // Constraint bounds (most of them are set to be non-negative)
    for(int i = 0; i < ng; ++i) {
        gl[i] = 0.0;
        gu[i] = 1.0e19;
    }

    // Create the FG_eval object
    FG_eval fg_eval(lambda1, lambda2, lambda3, D, v_0, t_g);

    // IPOPT options
    std::string options;
    options += "Integer print_level  0\n";
    options += "String sb            yes\n";
    options += "Integer max_iter     10\n";
    options += "Numeric tol          1e-6\n";
    options += "String derivative_test   second-order\n";
    options += "Numeric point_perturbation_radius   0.\n";

    // Solve the problem
    CppAD::ipopt::solve_result<CppAD::vector<double>> solution;
    CppAD::ipopt::solve<CppAD::vector<double>, FG_eval>(options, x0, xl, xu, gl, gu, fg_eval, solution);

    optimal_a = solution.x[0];
    optimal_v = solution.x[1];
    double d = (solution.x[1] * solution.x[1] - v_0 * v_0) / (2.0 * solution.x[0]) + solution.x[1] * (t_g - (solution.x[1] - v_0) / solution.x[0]);

    // Output the solution
    std::cout << "OptimalSpeedTrajectory result:\noptAcc: " << solution.x[0] << ", optSpeed: " << solution.x[1] << ", actual d: " << d << std::endl;

}
