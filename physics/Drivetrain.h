#pragma once
#include <array>
#include <vector>

namespace CarPhysics {

struct EngineParams {
    double idleRPM{800.0};
    double redlineRPM{7500.0};
    double maxTorque{420.0};          // N·m (at peak RPM)
    double peakTorqueRPM{4500.0};
    double inertia{0.25};             // kg·m²
    double frictionCoeff{0.02};       // internal friction (N·m·s)
};

struct GearboxParams {
    std::vector<double> gearRatios{-3.5, 3.9, 2.2, 1.5, 1.1, 0.85, 0.68};
    // index 0 = reverse, 1 = 1st, 2 = 2nd … 6 = 6th
    double finalDriveRatio{3.73};
    double efficiency{0.92};          // power transmission loss
    double shiftTime{0.08};           // s  (torque interruption window)
};

enum class DriveType { FWD, RWD, AWD };

struct DifferentialParams {
    DriveType driveType{DriveType::RWD};
    double    lockingCoeff{0.0};      // 0 = open, 1 = fully locked
    double    frontTorqueSplit{0.40}; // AWD only: fraction to front axle
};

struct DrivetrainState {
    double engineRPM{800.0};
    double engineTorque{0.0};         // N·m output (after throttle)
    double wheelTorque{0.0};          // N·m at wheel after gearbox
    int    currentGear{1};            // 1-based, 0 = neutral, -1 = reverse
    bool   clutchEngaged{true};
    bool   isShifting{false};
    double shiftTimer{0.0};
    double throttle{0.0};             // [0,1]
    double engineAngularVel{0.0};     // rad/s
};

class Drivetrain {
public:
    explicit Drivetrain(EngineParams ep = {}, GearboxParams gp = {},
                        DifferentialParams dp = {})
        : m_engine(ep), m_gearbox(gp), m_diff(dp) {}

    /// Update drivetrain for one timestep.
    /// @param throttle [0,1]
    /// @param wheelAngularVel driven wheel angular velocity (rad/s)
    /// @param dt timestep (s)
    void update(double throttle, double wheelAngularVel, double dt);

    /// Request a gear change
    void shiftUp();
    void shiftDown();
    void setGear(int gear);     // 0 = neutral

    /// Compute engine torque from torque curve at given RPM
    double torqueCurve(double rpm) const;

    /// Effective total gear ratio for current gear (0 if neutral)
    double totalGearRatio() const;

    const DrivetrainState& state()    const { return m_state;   }
    const EngineParams&    engine()   const { return m_engine;  }
    const GearboxParams&   gearbox()  const { return m_gearbox; }
    const DifferentialParams& diff()  const { return m_diff;    }

    int maxGear() const { return static_cast<int>(m_gearbox.gearRatios.size()) - 1; }

private:
    EngineParams       m_engine;
    GearboxParams      m_gearbox;
    DifferentialParams m_diff;
    DrivetrainState    m_state;

    void updateShift(double dt);
    double clampRPM(double rpm) const;
};

}