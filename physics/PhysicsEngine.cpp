#include "physics/PhysicsEngine.h"
#include <stdexcept>
#include <algorithm>
#include <chrono>

namespace CarPhysics {

PhysicsEngine::PhysicsEngine(SimulationConfig config)
    : m_config(config) {}

size_t PhysicsEngine::addVehicle(std::unique_ptr<Vehicle> vehicle) {
    size_t idx = m_vehicles.size();
    m_vehicles.push_back(std::move(vehicle));
    m_inputs.emplace_back();
    m_callbacks.emplace_back();
    ++m_stats.activeVehicles;
    return idx;
}

void PhysicsEngine::removeVehicle(size_t index) {
    if (index >= m_vehicles.size())
        throw std::out_of_range("PhysicsEngine::removeVehicle: invalid index");
    m_vehicles.erase(m_vehicles.begin() + index);
    m_inputs.erase(m_inputs.begin() + index);
    m_callbacks.erase(m_callbacks.begin() + index);
    --m_stats.activeVehicles;
}

void PhysicsEngine::setInput(size_t vehicleIndex, const VehicleInput& input) {
    if (vehicleIndex >= m_inputs.size())
        throw std::out_of_range("PhysicsEngine::setInput: invalid index");
    m_inputs[vehicleIndex] = input;
}

int PhysicsEngine::step(double realDeltaTime) {
    auto wallStart = std::chrono::high_resolution_clock::now();

    m_accumulator += realDeltaTime;
    int steps = 0;
    const double dt = m_config.fixedTimestep;

    while (m_accumulator >= dt && steps < static_cast<int>(m_config.maxSubSteps)) {
        subStep(dt);
        m_accumulator -= dt;
        ++steps;
        ++m_stats.totalSteps;
        m_stats.simTimeSeconds += dt;
    }

    auto wallEnd = std::chrono::high_resolution_clock::now();
    m_stats.lastStepDurationMs =
        std::chrono::duration<double, std::milli>(wallEnd - wallStart).count();

    return steps;
}

void PhysicsEngine::subStep(double dt) {
    for (size_t i = 0; i < m_vehicles.size(); ++i) {
        if (!m_vehicles[i]) continue;

        // Optional sleep: skip nearly-static vehicles
        if (m_config.enableSleeping &&
            m_vehicles[i]->state().speed < m_config.sleepVelocityThreshold &&
            m_inputs[i].throttle < 0.001 &&
            m_inputs[i].brake    < 0.001)
        {
            // still fire callbacks so telemetry stays alive
            for (auto& cb : m_callbacks[i])
                cb(*m_vehicles[i], m_stats.simTimeSeconds);
            continue;
        }

        m_vehicles[i]->update(m_inputs[i], dt);

        for (auto& cb : m_callbacks[i])
            cb(*m_vehicles[i], m_stats.simTimeSeconds);

        // Clear one-shot inputs
        m_inputs[i].shiftUp   = false;
        m_inputs[i].shiftDown = false;
    }
}

void PhysicsEngine::onStep(size_t vehicleIndex, StepCallback cb) {
    if (vehicleIndex >= m_callbacks.size())
        throw std::out_of_range("PhysicsEngine::onStep: invalid index");
    m_callbacks[vehicleIndex].push_back(std::move(cb));
}

const Vehicle* PhysicsEngine::vehicle(size_t index) const {
    if (index >= m_vehicles.size()) return nullptr;
    return m_vehicles[index].get();
}

Vehicle* PhysicsEngine::vehicle(size_t index) {
    if (index >= m_vehicles.size()) return nullptr;
    return m_vehicles[index].get();
}

void PhysicsEngine::reset() {
    for (auto& v : m_vehicles) if (v) v->reset();
    for (auto& inp : m_inputs) inp = {};
    m_accumulator          = 0.0;
    m_stats.simTimeSeconds = 0.0;
    m_stats.totalSteps     = 0;
}

}