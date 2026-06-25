#pragma once
#include "physics/Vector3.h"

namespace CarPhysics {

    struct SuspensionParams {
        double springRate{25000.0};       // N/m
        double damperRate{3000.0};        // N·s/m
        double restLength{0.35};          // m
        double minLength{0.20};           // m (jounce limit)
        double maxLength{0.50};           // m (rebound limit)
        double antiRollBarRate{5000.0};   // N/m (shared between axle pair)
        double bumpStopRate{80000.0};     // N/m (engages within 1 cm of limit)
        double bumpStopZone{0.01};        // m
    };

    struct SuspensionState {
        double length{0.35};              // m current strut length
        double velocity{0.0};            // m/s compression velocity (+ = compression)
        double travelNorm{0.0};          // [0,1]: 0 = full rebound, 1 = full jounce
        bool   inJounce{false};
        bool   inRebound{false};
    };

    class Suspension {
    public:
        explicit Suspension(SuspensionParams params = {})
            : m_params(params), m_state{params.restLength} {}

        /// Compute suspension force (positive = push wheel down, i.e. extend strut)
        /// @param compressionVel positive = compressing
        double computeForce(double currentLength, double compressionVel) const;

        /// Compute anti-roll bar contribution for one wheel of an axle pair.
        /// rolLDiff = difference in roll angle (rad) between left and right sides.
        double computeAntiRollForce(double rollDiff) const;

        void   update(double currentLength, double compressionVel, double dt);

        const SuspensionState& state()  const { return m_state; }
        const SuspensionParams& params() const { return m_params; }
        void  setParams(const SuspensionParams& p) { m_params = p; }

    private:
        SuspensionParams m_params;
        SuspensionState  m_state;

        double bumpStopForce(double currentLength) const;
    };

}