#include "physics/Tire.h"
#include <cmath>
#include <algorithm>

namespace CarPhysics {

// Pacejka Magic Formula
double Tire::magicFormula(double B, double C, double D, double E, double x) {
    return D * std::sin(C * std::atan(B*x - E*(B*x - std::atan(B*x))));
}

double Tire::tempGripFactor(double tempC) {
    // Grip peaks near 80 °C; falls off sharply below 40 or above 120
    constexpr double peakTemp  = 80.0;
    constexpr double sharpness = 0.0015;
    double delta = tempC - peakTemp;
    return 0.6 + 0.4 * std::exp(-sharpness * delta * delta);
}

// Slip computation
TireState Tire::computeSlip(double forwardVel, double lateralVel,
                             double angularVel) const {
    TireState state;
    state.angularVelocity = angularVel;

    constexpr double eps = 0.1; // m/s  avoid division by zero at rest
    double wheelSpeed = angularVel * m_params.radius;
    double refSpeed   = std::max(std::abs(forwardVel), eps);

    // κ = (V_wheel - V_forward) / V_ref
    state.slipRatio = (wheelSpeed - forwardVel) / refSpeed;
    state.slipRatio = std::clamp(state.slipRatio, -1.0, 1.0);

    // α = atan2(lateral, forward)  [opposite sign convention: +left = +α]
    if (std::abs(forwardVel) > eps)
        state.slipAngle = -std::atan2(lateralVel, std::abs(forwardVel));
    else
        state.slipAngle = 0.0;

    return state;
}

// Force computation
Tire::Forces Tire::computeForces(const TireContactPatch& contact,
                                  const TireState& state) const {
    if (!contact.inContact || contact.normalForce <= 0.0) return {};

    double Fz  = contact.normalForce;
    double tgf = tempGripFactor(state.temperature);

    // Longitudinal force (Fx)
    double Fx = magicFormula(m_params.Bx, m_params.Cx,
                              m_params.Dx * Fz * tgf,
                              m_params.Ex,
                              state.slipRatio);

    // Lateral force (Fy) — slip angle in radians
    double Fy = magicFormula(m_params.By, m_params.Cy,
                              m_params.Dy * Fz * tgf,
                              m_params.Ey,
                              std::tan(state.slipAngle));

    // Self-aligning torque (simplified pneumatic trail model)
    double trailM = 0.04 * m_params.radius; // pneumatic trail ≈ 4 % of radius
    double Mz     = -trailM * Fy;

    // Friction-circle coupling: reduce lateral if longitudinal is saturating
    double frictionLimit = m_params.Dx * Fz * tgf;
    double combined = std::sqrt(Fx*Fx + Fy*Fy);
    if (combined > frictionLimit && combined > 1e-6) {
        double scale = frictionLimit / combined;
        Fx *= scale;
        Fy *= scale;
    }

    return {Fx, Fy, Mz};
}

double Tire::computeRollingResistance(const TireContactPatch& contact) const {
    return -m_params.rollingResistCoeff * contact.normalForce;
}

}