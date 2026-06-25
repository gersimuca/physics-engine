#pragma once
#include "physics/Vector3.h"
#include <array>

namespace CarPhysics {

struct ChassisParams {
    double mass{1500.0};               // kg  (sprung mass)
    double wheelbase{2.70};            // m
    double trackWidthFront{1.52};      // m
    double trackWidthRear{1.50};       // m
    double cgHeight{0.45};             // m  (centre of gravity above ground)
    double frontWeightBias{0.52};      // fraction of static weight on front axle

    // Torsional flex (chassis twist under lateral load)
    double torsionalStiffness{18000.0}; // N·m/deg

    // Roll stiffness split front/rear (affects handling balance)
    double rollStiffnessFront{12000.0}; // N·m/rad
    double rollStiffnessRear{10000.0};  // N·m/rad

    // Pitch stiffness
    double pitchStiffness{15000.0};     // N·m/rad

    // Inertia tensor components (kg·m²) — approximate for a sedan
    Vector3 inertiaTensor{1850.0, 3100.0, 2600.0}; // Ixx, Iyy, Izz
};

struct WeightDistribution {
    std::array<double,4> cornerLoad{};  // N  [FL, FR, RL, RR]
    double lateralTransfer{0.0};        // N (positive = transfer to right)
    double longitudinalTransfer{0.0};   // N (positive = transfer to rear)
    double rollAngle{0.0};              // rad
    double pitchAngle{0.0};             // rad
};

class Chassis {
public:
    explicit Chassis(ChassisParams params = {}) : m_params(params) {}

    /// Compute per-corner load given acceleration vector (m/s²) and ride heights.
    /// @param accel  body-frame acceleration (x=lateral, y=vertical, z=longitudinal)
    /// @param rideHeights  [FL, FR, RL, RR] in metres
    WeightDistribution computeWeightDistribution(
        const Vector3& accel,
        const std::array<double,4>& rideHeights) const;

    /// Torsion angle (rad) given difference in vertical loads on a diagonal pair
    double computeTorsionAngle(double diagonalLoadDiff) const;

    /// Roll angle (rad) given lateral acceleration
    double computeRollAngle(double lateralAccel) const;

    /// Pitch angle (rad) given longitudinal acceleration
    double computePitchAngle(double longitudinalAccel) const;

    /// World positions of the four wheel centres given body position/orientation
    std::array<Vector3,4> wheelCentres(const Vector3& bodyPos,
                                        const Vector3& orientation) const;

    const ChassisParams& params() const { return m_params; }
    void  setParams(const ChassisParams& p) { m_params = p; }

private:
    ChassisParams m_params;

    static constexpr double G = 9.81;
};

}