#pragma once
#include "physics/Vector3.h"

namespace CarPhysics {

/// Pacejka "Magic Formula" tire model.
/// Computes longitudinal (Fx) and lateral (Fy) forces from slip.
struct TireParams {
    // Pacejka coefficients (longitudinal)
    double Bx{10.0};   // stiffness factor
    double Cx{1.9};    // shape factor
    double Dx{1.0};    // peak factor  (multiplied by normal load)
    double Ex{0.97};   // curvature factor

    // Pacejka coefficients (lateral)
    double By{8.0};
    double Cy{1.3};
    double Dy{0.9};    // multiplied by normal load
    double Ey{-1.5};

    // Physical properties
    double radius{0.33};          // m
    double rollingResistCoeff{0.015};
    double maxSlipRatio{0.20};    // slip ratio at which peak Fx occurs
};

struct TireContactPatch {
    Vector3 position{};   // world-space contact point
    Vector3 normal{Vector3::UP};
    double  normalForce{0.0};     // N  (positive = ground pushes up)
    bool    inContact{true};
};

struct TireState {
    double slipRatio{0.0};        // κ: (ωr - v) / max(|v|, ε)
    double slipAngle{0.0};        // α: atan2(lateral_velocity, forward_velocity) rad
    double angularVelocity{0.0};  // ω: rad/s (spin)
    double temperature{25.0};     // °C  (affects grip multiplier)
};

class Tire {
public:
    explicit Tire(TireParams params = {}) : m_params(params) {}

    /// Returns [Fx (forward), Fy (lateral), Mz (aligning torque)]
    /// given contact patch normal force and current slip state.
    struct Forces {
        double Fx{0.0};   // N longitudinal  (+forward)
        double Fy{0.0};   // N lateral       (+left)
        double Mz{0.0};   // N·m self-aligning torque
    };

    Forces computeForces(const TireContactPatch& contact,
                         const TireState& state) const;

    double computeRollingResistance(const TireContactPatch& contact) const;

    /// Update slip state given vehicle velocities and wheel spin
    TireState computeSlip(double forwardVel, double lateralVel,
                          double angularVel) const;

    const TireParams& params() const { return m_params; }
    void  setParams(const TireParams& p) { m_params = p; }

private:
    TireParams m_params;

    /// Pacejka Magic Formula: y = D·sin(C·atan(B·x - E·(B·x - atan(B·x))))
    static double magicFormula(double B, double C, double D, double E, double x);

    /// Temperature grip multiplier (peak at ~80 °C)
    static double tempGripFactor(double tempC);
};

}