#pragma once
#include "physics/Vehicle.h"
#include <vector>
#include <functional>
#include <memory>
#include <chrono>

namespace CarPhysics {

struct SimulationConfig {
    double fixedTimestep{1.0 / 500.0};   // s  (500 Hz physics)
    double maxSubSteps{10};              // safety cap on catch-up steps
    bool   enableSleeping{true};         // skip static vehicles
    double sleepVelocityThreshold{0.01}; // m/s
};

struct SimulationStats {
    double simTimeSeconds{0.0};
    uint64_t totalSteps{0};
    double lastStepDurationMs{0.0};
    int    activeVehicles{0};
};

using StepCallback = std::function<void(const Vehicle&, double /*simTime*/)>;

class PhysicsEngine {
public:
    explicit PhysicsEngine(SimulationConfig config = {});

    /// Add a vehicle to the simulation; returns its index
    size_t addVehicle(std::unique_ptr<Vehicle> vehicle);

    /// Remove vehicle by index
    void removeVehicle(size_t index);

    /// Set driver input for a vehicle
    void setInput(size_t vehicleIndex, const VehicleInput& input);

    /// Advance simulation by realDeltaTime (s), using fixed sub-steps.
    /// Returns number of sub-steps executed.
    int step(double realDeltaTime);

    /// Register a callback fired once per sub-step for a vehicle
    void onStep(size_t vehicleIndex, StepCallback cb);

    const Vehicle* vehicle(size_t index) const;
    Vehicle*       vehicle(size_t index);

    size_t vehicleCount() const { return m_vehicles.size(); }

    const SimulationStats& stats()  const { return m_stats;  }
    const SimulationConfig& config() const { return m_config; }
    void  setConfig(const SimulationConfig& c) { m_config = c; }

    void  reset();

    double simTime() const { return m_stats.simTimeSeconds; }

private:
    SimulationConfig m_config;
    SimulationStats  m_stats;

    std::vector<std::unique_ptr<Vehicle>>  m_vehicles;
    std::vector<VehicleInput>              m_inputs;
    std::vector<std::vector<StepCallback>> m_callbacks;

    double m_accumulator{0.0};

    void subStep(double dt);
};

}