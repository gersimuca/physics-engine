#include "physics/Chassis.h"
#include <cmath>
#include <numeric>

namespace CarPhysics {

WeightDistribution Chassis::computeWeightDistribution(
    const Vector3& accel,
    const std::array<double,4>& /*rideHeights*/) const
{
    WeightDistribution wd;
    const double W    = m_params.mass * G;
    const double wb   = m_params.wheelbase;
    const double twF  = m_params.trackWidthFront;
    const double twR  = m_params.trackWidthRear;
    const double hcg  = m_params.cgHeight;

    // Static corner loads
    double staticFront = W * m_params.frontWeightBias;
    double staticRear  = W * (1.0 - m_params.frontWeightBias);

    // Longitudinal weight transfer (braking/acceleration): dW = m·ax·h/wb
    double dWlong = m_params.mass * accel.z * hcg / wb;
    // Lateral weight transfer (cornering): split by roll stiffness front/rear
    double totalRollStiffness = m_params.rollStiffnessFront + m_params.rollStiffnessRear;
    double dWlatF = 0.0, dWlatR = 0.0;
    if (totalRollStiffness > 0.0) {
        double rollMoment = m_params.mass * accel.x * hcg;
        dWlatF = rollMoment * (m_params.rollStiffnessFront / totalRollStiffness) / twF;
        dWlatR = rollMoment * (m_params.rollStiffnessRear  / totalRollStiffness) / twR;
    }

    wd.longitudinalTransfer = dWlong;
    wd.lateralTransfer      = (dWlatF + dWlatR) * 0.5;
    wd.rollAngle            = computeRollAngle(accel.x);
    wd.pitchAngle           = computePitchAngle(accel.z);

    // Per-corner: FL, FR, RL, RR
    double baseFront = staticFront * 0.5;
    double baseRear  = staticRear  * 0.5;

    wd.cornerLoad[0] = baseFront - dWlong * 0.5 - dWlatF; // FL
    wd.cornerLoad[1] = baseFront - dWlong * 0.5 + dWlatF; // FR
    wd.cornerLoad[2] = baseRear  + dWlong * 0.5 - dWlatR; // RL
    wd.cornerLoad[3] = baseRear  + dWlong * 0.5 + dWlatR; // RR

    // Clamp to non-negative (wheel lift-off)
    for (auto& cl : wd.cornerLoad)
        if (cl < 0.0) cl = 0.0;

    return wd;
}

double Chassis::computeTorsionAngle(double diagonalLoadDiff) const {
    if (m_params.torsionalStiffness <= 0.0) return 0.0;
    // Convert stiffness from N·m/deg to N·m/rad, then compute angle
    double stiffnessRad = m_params.torsionalStiffness * (180.0 / M_PI);
    return diagonalLoadDiff / stiffnessRad;
}

double Chassis::computeRollAngle(double lateralAccel) const {
    double totalStiffness = m_params.rollStiffnessFront + m_params.rollStiffnessRear;
    if (totalStiffness <= 0.0) return 0.0;
    double rollMoment = m_params.mass * lateralAccel * m_params.cgHeight;
    return rollMoment / totalStiffness;
}

double Chassis::computePitchAngle(double longitudinalAccel) const {
    if (m_params.pitchStiffness <= 0.0) return 0.0;
    double pitchMoment = m_params.mass * longitudinalAccel * m_params.cgHeight;
    return pitchMoment / m_params.pitchStiffness;
}

std::array<Vector3,4> Chassis::wheelCentres(const Vector3& bodyPos,
                                              const Vector3& orientation) const
{
    // Local offsets from body centre [FL, FR, RL, RR]
    double halfWB  = m_params.wheelbase * 0.5;
    double halfTwF = m_params.trackWidthFront * 0.5;
    double halfTwR = m_params.trackWidthRear  * 0.5;

    std::array<Vector3,4> local = {{
        {-halfTwF,  0.0,  halfWB},  // FL
        { halfTwF,  0.0,  halfWB},  // FR
        {-halfTwR,  0.0, -halfWB},  // RL
        { halfTwR,  0.0, -halfWB}   // RR
    }};

    // Rotate each offset by body yaw (simplified: only yaw matters for position)
    double cy = std::cos(orientation.z);
    double sy = std::sin(orientation.z);
    std::array<Vector3,4> world;
    for (int i = 0; i < 4; ++i) {
        world[i] = {
            bodyPos.x + local[i].x * cy - local[i].z * sy,
            bodyPos.y + local[i].y,
            bodyPos.z + local[i].x * sy + local[i].z * cy
        };
    }
    return world;
}

}