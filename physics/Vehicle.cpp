#include "physics/Vehicle.h"
#include <cmath>
#include <algorithm>

namespace CarPhysics {

Vehicle::Vehicle() {
    // Configure default body
    m_body.mass           = m_chassis.params().mass;
    m_body.inertiaTensor  = m_chassis.params().inertiaTensor;

    // Stagger suspension rest lengths slightly front/rear
    SuspensionParams spF, spR;
    spF.springRate = 28000.0; spF.damperRate = 3200.0;
    spR.springRate = 24000.0; spR.damperRate = 2800.0;
    m_suspensions[0].setParams(spF);
    m_suspensions[1].setParams(spF);
    m_suspensions[2].setParams(spR);
    m_suspensions[3].setParams(spR);

    reset();
}

void Vehicle::reset() {
    m_body      = RigidBody{};
    m_body.mass = m_chassis.params().mass;
    m_body.inertiaTensor = m_chassis.params().inertiaTensor;
    m_body.position      = {0, 0.5, 0};
    m_drivetrain.setGear(1);
    m_state = VehicleState{};
}

void Vehicle::update(const VehicleInput& input, double dt) {
    // Handle shift requests (one-shot)
    if (input.shiftUp)   m_drivetrain.shiftUp();
    if (input.shiftDown) m_drivetrain.shiftDown();

    applyGravity();
    applyDrivetrain(input, dt);
    applySuspensionAndTires(input);
    applyAerodynamics();

    m_body.integrate(dt);
    updateState();
}

void Vehicle::applyGravity() {
    m_body.applyForce({0, -m_body.mass * G, 0});
}

void Vehicle::applyDrivetrain(const VehicleInput& input, double dt) {
    // Driven wheel speed — use rear axle average for RWD default
    double wheelRadius   = m_tires[2].params().radius;
    double driveWheelOmega = m_body.forwardSpeed() / std::max(wheelRadius, 1e-4);

    m_drivetrain.update(input.throttle, driveWheelOmega, dt);

    double wheelTorque = m_drivetrain.state().wheelTorque;

    // Brake torque (opposes motion)
    double brakeForce = input.brake * BRAKE_MAX / wheelRadius;

    // Net traction force along forward axis
    double tractionForce = (wheelTorque / wheelRadius) - brakeForce;

    Vector3 forward = m_body.bodyToWorld(Vector3::FORWARD);
    m_body.applyForce(forward * tractionForce);

    // Handbrake locks rear wheels → large rear longitudinal force reversal
    if (input.handbrake) {
        Vector3 vel   = m_body.velocity;
        double  speed = vel.length();
        if (speed > 0.1)
            m_body.applyForce(-vel.normalized() * m_body.mass * G * 0.6);
    }
}

void Vehicle::applySuspensionAndTires(const VehicleInput& input) {
    // Compute per-corner normal forces from weight distribution
    Vector3 accel = m_body.acceleration;
    // Body-frame acceleration
    Vector3 bodyAccel = m_body.worldToBody(accel);

    std::array<double,4> rideH = {
        m_state.rideHeight, m_state.rideHeight,
        m_state.rideHeight, m_state.rideHeight
    };
    WeightDistribution wd = m_chassis.computeWeightDistribution(bodyAccel, rideH);

    m_state.rollAngle  = wd.rollAngle;
    m_state.pitchAngle = wd.pitchAngle;

    // Steering angle (simplified Ackermann)
    double steerAngle = input.steer * 0.45;   // max ≈ 25.8 deg

    Vector3 totalForce{};
    for (int i = 0; i < 4; ++i) {
        double Fz = wd.cornerLoad[i];
        m_state.cornerLoad[i] = Fz;

        // Suspension force pushes car upward
        double suspF = m_suspensions[i].computeForce(
            m_suspensions[i].params().restLength, 0.0);
        m_body.applyForce({0, suspF * 0.25, 0});

        // Tire contact patch
        TireContactPatch cp;
        cp.normalForce = Fz;
        cp.inContact   = (Fz > 0.0);

        // Wheel velocities in body frame
        double fwdVel = m_body.forwardSpeed();
        double latVel = m_body.velocity.dot(m_body.bodyToWorld(Vector3::RIGHT));

        // Steering affects front wheels (indices 0,1)
        double effectiveSteer = (i < 2) ? steerAngle : 0.0;
        double wheelFwdVel = fwdVel * std::cos(effectiveSteer)
                           + latVel * std::sin(effectiveSteer);
        double wheelLatVel = -fwdVel * std::sin(effectiveSteer)
                            + latVel * std::cos(effectiveSteer);

        // Wheel angular velocity approximation
        double wheelOmega = fwdVel / std::max(m_tires[i].params().radius, 1e-4);

        TireState ts = m_tires[i].computeSlip(wheelFwdVel, wheelLatVel, wheelOmega);
        Tire::Forces tf = m_tires[i].computeForces(cp, ts);

        m_state.tireStates[i] = ts;
        m_state.tireForces[i] = tf;

        // Transform tire forces to world frame
        Vector3 fwd   = m_body.bodyToWorld(Vector3::FORWARD);
        Vector3 right = m_body.bodyToWorld(Vector3::RIGHT);

        // Rotate by steer angle for front tires
        if (i < 2) {
            double cs = std::cos(effectiveSteer), ss = std::sin(effectiveSteer);
            Vector3 steerFwd   = fwd * cs + right * ss;
            Vector3 steerRight = -fwd * ss + right * cs;
            m_body.applyForce(steerFwd   * tf.Fx);
            m_body.applyForce(steerRight * tf.Fy);
        } else {
            m_body.applyForce(fwd   * tf.Fx);
            m_body.applyForce(right * tf.Fy);
        }

        // Yaw torque from lateral tire forces
        auto centres = m_chassis.wheelCentres(m_body.position, m_body.orientation);
        Vector3 r = centres[i] - m_body.position;
        Vector3 lat = (i < 2)
            ? m_body.bodyToWorld(Vector3{std::cos(effectiveSteer),0,-std::sin(effectiveSteer)})
            : m_body.bodyToWorld(Vector3::RIGHT);
        m_body.applyTorque(r.cross(lat * tf.Fy));

        // Rolling resistance
        double rr = m_tires[i].computeRollingResistance(cp);
        m_body.applyForce(fwd * rr);
    }
}

void Vehicle::applyAerodynamics() {
    AeroForces af = m_aero.compute(
        m_body.velocity,
        m_state.rideHeight,
        m_body.orientation.y);   // pitch

    m_body.applyForce(af.drag);
    m_body.applyForce(af.lift);
    m_body.applyTorque({af.rollMoment, 0, 0});
    m_body.applyTorque({0, 0, af.pitchMoment});

    m_state.aero = af;
}

void Vehicle::updateState() {
    m_state.position        = m_body.position;
    m_state.velocity        = m_body.velocity;
    m_state.orientation     = m_body.orientation;
    m_state.angularVelocity = m_body.angularVelocity;
    m_state.speed           = m_body.speed();
    m_state.speedKph        = m_state.speed * 3.6;
    m_state.drivetrain      = m_drivetrain.state();

    // G-forces
    constexpr double G = 9.81;
    Vector3 bodyAccel       = m_body.worldToBody(m_body.acceleration);
    m_state.lateralG        = bodyAccel.x / G;
    m_state.longitudinalG   = bodyAccel.z / G;
    m_state.verticalG       = bodyAccel.y / G + 1.0;
}

} 