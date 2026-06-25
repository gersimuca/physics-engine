#pragma once
#include "physics/Vector3.h"

namespace CarPhysics {

    /// 6-DOF rigid body. Integrates Newton-Euler equations at each timestep.
    struct RigidBody {
        // Intrinsic properties
        double mass{1500.0};           // kg
        Vector3 inertiaTensor{2000, 3000, 2500}; // kg·m² (Ixx, Iyy, Izz, diagonal approx)

        // State
        Vector3 position{};
        Vector3 velocity{};            // world-frame linear velocity  (m/s)
        Vector3 acceleration{};

        Vector3 orientation{};         // Euler angles (roll, pitch, yaw) in radians
        Vector3 angularVelocity{};     // world-frame angular velocity  (rad/s)
        Vector3 angularAcceleration{};

        // ── Accumulated forces / torques for current step
        Vector3 forceAccum{};
        Vector3 torqueAccum{};

        // ── Methods
        void applyForce(const Vector3& force);
        void applyTorque(const Vector3& torque);
        /// Force applied at a body-space offset (also generates torque)
        void applyForceAtPoint(const Vector3& force, const Vector3& worldPoint);

        /// Advance state by dt seconds (semi-implicit Euler)
        void integrate(double dt);

        /// Clear per-step accumulators
        void clearAccumulators();

        // ── Derived helpers
        double speed() const { return velocity.length(); }

        /// Speed along the forward (z) axis — positive = forward
        double forwardSpeed() const;

        /// Convert a world-frame direction to body frame
        Vector3 worldToBody(const Vector3& v) const;

        /// Convert a body-frame direction to world frame
        Vector3 bodyToWorld(const Vector3& v) const;
    };

}