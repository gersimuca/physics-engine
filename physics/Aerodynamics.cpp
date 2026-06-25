#include "physics/Aerodynamics.h"
#include <cmath>
#include <algorithm>

namespace CarPhysics {

double Aerodynamics::dynamicPressure(double speed) const {
    return 0.5 * m_params.airDensity * speed * speed;
}

double Aerodynamics::groundEffectMultiplier(double rideHeight) const {
    // Linearly interpolate between groundEffectMin and groundEffectMax
    // as rideHeight drops toward zero
    double t = std::clamp(1.0 - rideHeight / m_params.groundEffectRideHeight, 0.0, 1.0);
    return m_params.groundEffectMin +
           t * (m_params.groundEffectMax - m_params.groundEffectMin);
}

AeroForces Aerodynamics::compute(const Vector3& velocity,
                                  double rideHeight,
                                  double pitchAngle) const {
    AeroForces result;
    double speed = velocity.length();
    if (speed < 0.01) return result;

    double q      = dynamicPressure(speed);
    double A      = m_params.frontalArea;
    double geMult = groundEffectMultiplier(rideHeight);

    // Drag (opposes velocity direction)
    double dragMag = m_params.dragCoeff * q * A;
    result.drag    = velocity.normalized() * (-dragMag);

    // Lift / Downforce
    // Positive Cl = lift; negative Cl = downforce (pushes car down onto road)
    double liftMag    = m_params.liftCoeff * q * A * geMult;
    // Simple pitch coupling: downforce increases slightly when nose dips
    liftMag          *= (1.0 - 0.1 * pitchAngle);
    result.lift        = Vector3{0, liftMag, 0};

    double frontShare = m_params.frontDownforceBalance;
    result.frontLift  = Vector3{0, liftMag * frontShare,       0};
    result.rearLift   = Vector3{0, liftMag * (1.0-frontShare), 0};

    // Pitching moment
    result.pitchMoment = m_params.pitchMomentCoeff * q * A * 2.7; // 2.7m = wheelbase

    // Rolling moment (yaw-induced, e.g. crosswind)
    // Approximate crosswind from lateral velocity component
    double lateralSpeed = std::sqrt(velocity.x*velocity.x + velocity.z*velocity.z);
    result.rollMoment   = m_params.rollMomentCoeff * 0.5 * m_params.airDensity
                          * lateralSpeed * lateralSpeed * A;

    return result;
}

}