#pragma once
#include "physics/RigidBody.h"
#include "physics/Tire.h"
#include "physics/Suspension.h"
#include "physics/Aerodynamics.h"
#include "physics/Drivetrain.h"
#include "physics/Chassis.h"
#include <array>
#include <string>

namespace CarPhysics {

struct VehicleInput {
    double throttle{0.0};    // [0, 1]
    double brake{0.0};       // [0, 1]
    double steer{0.0};       // [-1, 1]  left negative, right positive
    bool   shiftUp{false};
    bool   shiftDown{false};
    bool   handbrake{false};
};

struct VehicleState {
    // from RigidBody
    Vector3 position{};
    Vector3 velocity{};
    Vector3 orientation{};    // roll, pitch, yaw (rad)
    Vector3 angularVelocity{};
    double  speed{0.0};       // m/s
    double  speedKph{0.0};

    // per-corner [FL, FR, RL, RR]
    std::array<double,4> cornerLoad{};
    std::array<double,4> suspensionTravel{};
    std::array<TireState,4> tireStates{};
    std::array<Tire::Forces,4> tireForces{};

    // from subsystems
    DrivetrainState drivetrain{};
    AeroForces      aero{};
    double          rideHeight{0.08};  // m  average
    double          rollAngle{0.0};    // rad
    double          pitchAngle{0.0};   // rad
    double          lateralG{0.0};
    double          longitudinalG{0.0};
    double          verticalG{1.0};
};

class Vehicle {
public:
    Vehicle();

    /// Advance simulation by dt seconds given driver inputs
    void update(const VehicleInput& input, double dt);

    /// Reset to initial state
    void reset();

    const VehicleState&   state()       const { return m_state; }
    const RigidBody&      rigidBody()   const { return m_body; }
    RigidBody&            rigidBody()         { return m_body; }
    Drivetrain&           drivetrain()        { return m_drivetrain; }
    const Drivetrain&     drivetrain()  const { return m_drivetrain; }
    Chassis&              chassis()          { return m_chassis; }
    const Chassis&        chassis()    const { return m_chassis; }
    Aerodynamics&         aero()             { return m_aero; }
    const Aerodynamics&   aero()       const { return m_aero; }
    std::array<Suspension,4>& suspensions()       { return m_suspensions; }
    std::array<Tire,4>&       tires()             { return m_tires; }

private:
    RigidBody                m_body;
    Drivetrain               m_drivetrain;
    Chassis                  m_chassis;
    Aerodynamics             m_aero;
    std::array<Suspension,4> m_suspensions;
    std::array<Tire,4>       m_tires;
    VehicleState             m_state;

    static constexpr double G         = 9.81;
    static constexpr double BRAKE_MAX = 8000.0;  // N·m at wheel

    void applyDrivetrain(const VehicleInput& input, double dt);
    void applySuspensionAndTires(const VehicleInput& input);
    void applyAerodynamics();
    void applyGravity();
    void updateState();
};

}