#pragma once

#include "simulation_core/types.hpp"

namespace simulation_core {

/**
 * @brief Pacejka Magic Formula tire model
 *
 * Calculates tire forces based on slip ratio and slip angle using
 * the simplified Magic Formula: F = D * sin(C * atan(B * x - E * (B * x - atan(B * x))))
 */
class TireModel {
public:
    TireModel();
    explicit TireModel(const TireParams& params);

    /**
     * @brief Set tire parameters
     */
    void setParams(const TireParams& params);

    /**
     * @brief Get current parameters
     */
    const TireParams& params() const { return params_; }

    /**
     * @brief Calculate longitudinal and lateral tire forces
     * @param slip_ratio Longitudinal slip ratio [-]
     * @param slip_angle Lateral slip angle [rad]
     * @param Fz Normal load [N]
     * @param Fx Output longitudinal force [N]
     * @param Fy Output lateral force [N]
     */
    void calculateForces(double slip_ratio, double slip_angle,
                         double Fz, double& Fx, double& Fy) const;

    /**
     * @brief Calculate longitudinal force only
     * @param slip_ratio Longitudinal slip ratio [-]
     * @param Fz Normal load [N]
     * @return Longitudinal force [N]
     */
    double calculateFx(double slip_ratio, double Fz) const;

    /**
     * @brief Calculate lateral force only
     * @param slip_angle Lateral slip angle [rad]
     * @param Fz Normal load [N]
     * @return Lateral force [N]
     */
    double calculateFy(double slip_angle, double Fz) const;

private:
    /**
     * @brief Magic Formula calculation
     * @param slip Slip value (ratio or angle)
     * @param B Stiffness factor
     * @param C Shape factor
     * @param D Peak factor (scaled by Fz)
     * @param E Curvature factor
     * @return Force [N]
     */
    double magicFormula(double slip, double B, double C, double D, double E) const;

    TireParams params_;
};

}  // namespace simulation_core
