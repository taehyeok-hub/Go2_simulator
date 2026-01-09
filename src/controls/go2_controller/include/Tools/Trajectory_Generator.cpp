#include "Trajectory_Generator.hpp"

TrajectoryGenerator::TrajectoryGenerator()
{
}

TrajectoryGenerator::~TrajectoryGenerator() {}

double TrajectoryGenerator::CubicBezier(double p0, double p1, double t)
{
    const double u = 1.0 - t;
    const double m = 0.5 * (p0 + p1); 
    return u*u*u*p0 + 3.0*u*u*t*m + 3.0*u*t*t*m + t*t*t*p1;
}

double TrajectoryGenerator::CubicBezierFirst(double p0, double p1, double t)
{
    const double u = 1.0 - t;
    const double m = 0.5 * (p0 + p1);
    return 3.0*u*u*(m - p0) + 3.0*t*t*(p1 - m);
}

double TrajectoryGenerator::CubicBezierSecond(double p0, double p1, double t)
{
    (void)t;
    return 3.0 * (p1 - p0);
}

double TrajectoryGenerator::QuadraticBezier(double p0, double p1, double p2, double t)
{
    const double u = 1.0 - t;
    return u*u*p0 + 2.0*u*t*p1 + t*t*p2;
}

double TrajectoryGenerator::QuadraticBezierFirst(double p0, double p1, double p2, double t)
{
    return 2.0*(1.0 - t)*(p1 - p0) + 2.0*t*(p2 - p1);
}

double TrajectoryGenerator::QuadraticBezierSecond(double p0, double p1, double p2, double t)
{
    (void)t;
    return 2.0 * (p2 - 2.0*p1 + p0);
}

double TrajectoryGenerator::Sinusoidal(double p0, double p1, double t)
{
    (void)t;
    return p0 + (p1 - p0) * 0.5 * (1 - cos(M_PI * t));
}

double TrajectoryGenerator::SinusoidalFirst(double p0, double p1, double t)
{
    (void)t;
    return (p1 - p0) * 0.5 * M_PI * sin(M_PI * t);
}

double TrajectoryGenerator::SinusoidalZ(double p0, double h, double t)
{
    (void)t;
    return p0 + h * 0.5 * (1 - cos(M_PI * t));
}

double TrajectoryGenerator::SinusoidalFirstZ(double h, double t)
{
    return 0.5 * h * M_PI * std::sin(M_PI * t);
}

double TrajectoryGenerator::Linear(double p0, double v, double hz)
{
    return p0 + v*(1/hz); 
}