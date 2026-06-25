#include "physics/RigidBody.h"
#include <cmath>

namespace CarPhysics {

void RigidBody::applyForce(const Vector3& force) {
    forceAccum += force;
}

void RigidBody::applyTorque(const Vector3& torque) {
    torqueAccum += torque;
}

void RigidBody::applyForceAtPoint(const Vector3& force, const Vector3& worldPoint) {
    forceAccum += force;
    Vector3 r = worldPoint - position;
    torqueAccum += r.cross(force);
}

void RigidBody::integrate(double dt) {
    if (dt <= 0.0 || mass <= 0.0) return;

    // ── Linear dynamics
    acceleration   = forceAccum * (1.0 / mass);
    velocity      += acceleration * dt;
    position      += velocity * dt;

    // Angular dynamics (diagonal inertia tensor, body frame)
    // α = I⁻¹ · τ  (simplified: diagonal tensor, body-frame torque ≈ world-frame)
    angularAcceleration = Vector3{
        torqueAccum.x / inertiaTensor.x,
        torqueAccum.y / inertiaTensor.y,
        torqueAccum.z / inertiaTensor.z
    };
    angularVelocity += angularAcceleration * dt;
    orientation     += angularVelocity     * dt;

    // ── Clamp orientation to [-π, π]
    auto wrap = [](double a) -> double {
        while (a >  M_PI) a -= 2.0*M_PI;
        while (a < -M_PI) a += 2.0*M_PI;
        return a;
    };
    orientation.x = wrap(orientation.x);
    orientation.y = wrap(orientation.y);
    orientation.z = wrap(orientation.z);

    clearAccumulators();
}

void RigidBody::clearAccumulators() {
    forceAccum  = Vector3::ZERO;
    torqueAccum = Vector3::ZERO;
}

double RigidBody::forwardSpeed() const {
    // Forward = +Z in body frame; project world velocity onto body forward axis
    Vector3 fwd = bodyToWorld(Vector3::FORWARD);
    return velocity.dot(fwd);
}

Vector3 RigidBody::worldToBody(const Vector3& v) const {
    double cy = std::cos(orientation.z), sy = std::sin(orientation.z); // yaw
    double cp = std::cos(orientation.y), sp = std::sin(orientation.y); // pitch
    double cr = std::cos(orientation.x), sr = std::sin(orientation.x); // roll

    // Full ZYX rotation matrix transpose (world→body)
    return {
         cy*cp*v.x + sy*cp*v.y - sp*v.z,
        (cy*sp*sr - sy*cr)*v.x + (sy*sp*sr + cy*cr)*v.y + cp*sr*v.z,
        (cy*sp*cr + sy*sr)*v.x + (sy*sp*cr - cy*sr)*v.y + cp*cr*v.z
    };
}

Vector3 RigidBody::bodyToWorld(const Vector3& v) const {
    double cy = std::cos(orientation.z), sy = std::sin(orientation.z);
    double cp = std::cos(orientation.y), sp = std::sin(orientation.y);
    double cr = std::cos(orientation.x), sr = std::sin(orientation.x);

    // Full ZYX rotation matrix (body→world)
    return {
        (cy*cp)*v.x + (cy*sp*sr - sy*cr)*v.y + (cy*sp*cr + sy*sr)*v.z,
        (sy*cp)*v.x + (sy*sp*sr + cy*cr)*v.y + (sy*sp*cr - cy*sr)*v.z,
        (-sp  )*v.x + (cp*sr            )*v.y + (cp*cr            )*v.z
    };
}

}