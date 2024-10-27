/****************************************************************************/
/// @file    optimizeSpeedAcc.h
 
/// @date    December 2023
///
/****************************************************************************/
#ifndef OPTIMIZESPEEDACC_H
#define OPTIMIZESPEEDACC_H

#include <cppad/ipopt/solve.hpp>
#include <cassert>


#define A_UPPER_BOUND       3.0
#define A_LOWER_BOUND       -5.0
#define V_UPPER_BOUND       16.66
#define V_LOWER_BOUND       0.0

#define VT_MICRO_M_VALUES \
    {-7.73452, -0.01799, -0.00427, 0.00018829}, \
    {0.02804, 0.00772, 0.00083744, -0.00003387}, \
    {-0.00021988, -0.00005219, -7.44E-06, 2.77E-07}, \
    {1.08E-06, 2.47E-07, 4.87E-08, 3.79E-10}



// FG_eval class declaration
class FG_eval {
public:
    using ADvector = CppAD::vector<CppAD::AD<double>>;

    // Constructor
    FG_eval(double lambda1, double lambda2, double lambda3, double D, double v_0, double t_g);

    // Operator overload for problem evaluation
    void operator()(ADvector& fg, const ADvector& x) const;

private:
    double lambda1, lambda2, lambda3, D, v_0, t_g;
};

void solveOptimizationProblem(double lambda1, double lambda2, double lambda3, double D, double v_0, double t_g, double& optimal_a, double& optimal_v);

#endif // OPTIMIZESPEEDACC_H
