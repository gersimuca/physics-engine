#include "physics/Drivetrain.h"
#include <cmath>
#include <algorithm>

namespace CarPhysics {

// Torque curve: Gaussian-ish bell centred at peakTorqueRPM
double Drivetrain::torqueCurve(double rpm) const {
    rpm = clampRPM(rpm);
    // Simple bell shape: Tmax at peak, drops symmetrically
    double spread = (m_engine.redlineRPM - m_engine.idleRPM) * 0.35;
    double delta  = rpm - m_engine.peakTorqueRPM;
    double scale  = std::exp(-(delta * delta) / (2.0 * spread * spread));
    // Idle floor: small torque below 1500 RPM to simulate idle enrichment
    double idleBoost = 0.1 * m_engine.maxTorque *
                       std::max(0.0, 1.0 - (rpm - m_engine.idleRPM) / 700.0);
    return scale * m_engine.maxTorque + idleBoost;
}

double Drivetrain::totalGearRatio() const {
    int g = m_state.currentGear;
    // gear index: reverse=0, neutral → return 0, 1st=1 …
    if (g == 0) return 0.0;
    // negative gear = reverse
    if (g < 0)  return m_gearbox.gearRatios[0] * m_gearbox.finalDriveRatio;
    int idx = std::clamp(g, 1, (int)m_gearbox.gearRatios.size()-1);
    return m_gearbox.gearRatios[idx] * m_gearbox.finalDriveRatio;
}

void Drivetrain::update(double throttle, double wheelAngularVel, double dt) {
    m_state.throttle = std::clamp(throttle, 0.0, 1.0);

    updateShift(dt);

    if (m_state.isShifting || !m_state.clutchEngaged) {
        // Torque interruption during shift
        m_state.wheelTorque  = 0.0;
        // Engine freewheels: light deceleration from friction
        m_state.engineAngularVel = std::max(
            m_engine.idleRPM * M_PI / 30.0,
            m_state.engineAngularVel - m_engine.frictionCoeff * dt * 100.0
        );
        m_state.engineRPM = m_state.engineAngularVel * 30.0 / M_PI;
        m_state.engineTorque = 0.0;
        return;
    }

    double ratio = totalGearRatio();
    if (std::abs(ratio) < 1e-6) {
        // Neutral
        m_state.engineRPM    = clampRPM(m_state.engineRPM);
        m_state.wheelTorque  = 0.0;
        m_state.engineTorque = 0.0;
        return;
    }

    // Back-calculate engine RPM from wheel speed
    double engineOmega = std::abs(wheelAngularVel) * std::abs(ratio);
    m_state.engineAngularVel = std::max(
        m_engine.idleRPM * M_PI / 30.0, engineOmega
    );
    m_state.engineRPM = clampRPM(m_state.engineAngularVel * 30.0 / M_PI);

    // Engine torque from curve × throttle
    m_state.engineTorque = torqueCurve(m_state.engineRPM) * m_state.throttle;

    // Internal friction loss
    double frictionTorque = m_engine.frictionCoeff * m_state.engineAngularVel;
    double netTorque      = m_state.engineTorque - frictionTorque;

    // Wheel torque after gearbox efficiency
    m_state.wheelTorque = netTorque * std::abs(ratio) * m_gearbox.efficiency;

    // Reverse gear: negate
    if (m_state.currentGear < 0)
        m_state.wheelTorque = -std::abs(m_state.wheelTorque);
}

void Drivetrain::shiftUp() {
    if (m_state.isShifting) return;
    if (m_state.currentGear < maxGear()) {
        m_state.currentGear++;
        m_state.isShifting = true;
        m_state.shiftTimer  = m_gearbox.shiftTime;
    }
}

void Drivetrain::shiftDown() {
    if (m_state.isShifting) return;
    if (m_state.currentGear > -1) {
        m_state.currentGear--;
        m_state.isShifting = true;
        m_state.shiftTimer  = m_gearbox.shiftTime;
    }
}

void Drivetrain::setGear(int gear) {
    int clamped = std::clamp(gear, -1, maxGear());
    m_state.currentGear = clamped;
    m_state.isShifting  = false;
    m_state.shiftTimer   = 0.0;
}

void Drivetrain::updateShift(double dt) {
    if (!m_state.isShifting) return;
    m_state.shiftTimer -= dt;
    if (m_state.shiftTimer <= 0.0) {
        m_state.isShifting = false;
        m_state.shiftTimer  = 0.0;
    }
}

double Drivetrain::clampRPM(double rpm) const {
    return std::clamp(rpm, m_engine.idleRPM, m_engine.redlineRPM);
}

}