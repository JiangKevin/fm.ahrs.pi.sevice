
#pragma once
// /
#include <Eigen/Dense>
//
static Eigen::VectorXf computeVelocity( float dt, const Eigen::VectorXf& a_prev, const Eigen::VectorXf& a_next )
{
    Eigen::VectorXf v = 0.5 * ( a_prev + a_next ) * dt;
    return v;
}
