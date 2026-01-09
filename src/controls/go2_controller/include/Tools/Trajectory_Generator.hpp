#include <vector>
#include <functional>
#include <iostream>
#include <stdio.h>
#include <cmath>
#include "Eigen/Dense"
#include "Enum_Shared.hpp"

class TrajectoryGenerator
{
public:
    TrajectoryGenerator();
    ~TrajectoryGenerator();

    double CubicBezier(double p0, double p1, double t);
    double CubicBezierFirst(double p0, double p1, double t);
    double CubicBezierSecond(double p0, double p1, double t);

    double QuadraticBezier(double p0, double p1, double p2, double t);
    double QuadraticBezierFirst(double p0, double p1, double p2, double t);
    double QuadraticBezierSecond(double p0, double p1, double p2, double t);

    double Sinusoidal(double p0, double p1, double phase);
    double SinusoidalFirst(double p0, double p1, double t);

    double SinusoidalZ(double p0, double h, double phase);
    double SinusoidalFirstZ(double h, double t);

    double Linear(double p0, double v, double hz);

private:

};