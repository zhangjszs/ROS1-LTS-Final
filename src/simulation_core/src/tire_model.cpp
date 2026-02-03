#include "simulation_core/tire_model.hpp"
#include <cmath>
#include <algorithm>

namespace simulation_core {

TireModel::TireModel() : params_() {}

TireModel::TireModel(const TireParams& params) : params_(params) {}

void TireModel::setParams(const TireParams& params) {
    params_ = params;
}

void TireModel::calculateForces(double slip_ratio, double slip_angle,
                                 double Fz, double& Fx, double& Fy) const {
    // Ensure positive normal load
    Fz = std::max(0.0, Fz);
    if (Fz < 1.0) {
        Fx = 0.0;
        Fy = 0.0;
        return;
    }

    // Calculate individual forces
    Fx = calculateFx(slip_ratio, Fz);
    Fy = calculateFy(slip_angle, Fz);

    // Combined slip friction circle limitation
    double Fmax = params_.friction * Fz * params_.D;
    double F_total = std::hypot(Fx, Fy);

    if (F_total > Fmax && F_total > 1e-6) {
        double scale = Fmax / F_total;
        Fx *= scale;
        Fy *= scale;
    }
}

double TireModel::calculateFx(double slip_ratio, double Fz) const {
    // Clamp slip ratio to reasonable range
    slip_ratio = std::clamp(slip_ratio, -1.0, 1.0);

    // Peak force scales with normal load and friction
    double D = params_.friction * Fz * params_.D;

    return magicFormula(slip_ratio, params_.B, params_.C, D, params_.E);
}

double TireModel::calculateFy(double slip_angle, double Fz) const {
    // Clamp slip angle to reasonable range
    slip_angle = std::clamp(slip_angle, -M_PI / 2.0, M_PI / 2.0);

    // Peak force scales with normal load and friction
    double D = params_.friction * Fz * params_.D;

    return magicFormula(slip_angle, params_.B, params_.C, D, params_.E);
}

double TireModel::magicFormula(double slip, double B, double C,
                                double D, double E) const {
    // Pacejka Magic Formula:
    // F = D * sin(C * atan(B * x - E * (B * x - atan(B * x))))
    double Bx = B * slip;
    double inner = Bx - E * (Bx - std::atan(Bx));
    return D * std::sin(C * std::atan(inner));
}

}  // namespace simulation_core
