#pragma once
#include "physics/Vector3.h"

namespace CarPhysics {

struct AeroParams {
    double frontalArea{2.2};          // m²
    double dragCoeff{0.30};           // Cd
    double liftCoeff{-0.40};          // Cl  (negative = downforce)
    double airDensity{1.225};         // kg/m³  (sea level, 15 °C)

    // Front/rear aero balance: fraction of downforce on front axle
    double frontDownforceBalance{0.45};

    // Pitch/roll aero moments
    double pitchMomentCoeff{0.05};    // Cm (positive = nose-up)
    double rollMomentCoeff{0.02};     // Cn (positive = roll right)

    // Ground effect: downforce multiplier that grows as ride height drops
    double groundEffectMin{1.0};      // at full ride height
    double groundEffectMax{1.6};      // at minimum ride height
    double groundEffectRideHeight{0.08}; // m reference ride height
};

struct AeroForces {
    Vector3 drag{};        // world-frame, opposes velocity
    Vector3 lift{};        // world-frame, vertical  (downforce if -Y)
    Vector3 frontLift{};   // front axle portion
    Vector3 rearLift{};    // rear  axle portion
    double  pitchMoment{}; // N·m (about lateral axis)
    double  rollMoment{};  // N·m (about longitudinal axis)
};

class Aerodynamics {
public:
    explicit Aerodynamics(AeroParams params = {}) : m_params(params) {}

    /// Compute all aerodynamic forces/moments for the current state.
    /// @param velocity  world-frame velocity  (m/s)
    /// @param rideHeight  average ride height (m)
    /// @param pitchAngle  body pitch angle    (rad, positive = nose up)
    AeroForces compute(const Vector3& velocity,
                       double rideHeight,
                       double pitchAngle) const;

    double dynamicPressure(double speed) const;
    double groundEffectMultiplier(double rideHeight) const;

    const AeroParams& params() const { return m_params; }
    void  setParams(const AeroParams& p) { m_params = p; }

private:
    AeroParams m_params;
};

}