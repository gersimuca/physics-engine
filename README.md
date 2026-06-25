# Physics Engine

Simulation car physics engine with Qt6 UI and Google Test suite.

## Architecture

```
physics_engine/
├── physics/
│   ├── Vector3         – 3D math (dot, cross, normalize, lerp)
│   ├── RigidBody       – 6-DOF Newton-Euler integration
│   ├── Tire            – Pacejka Magic Formula (Fx, Fy, Mz, friction circle)
│   ├── Suspension      – Spring-damper + bump stops + anti-roll bar
│   ├── Aerodynamics    – Drag, downforce, ground effect, pitch/roll moments
│   ├── Drivetrain      – Engine torque curve, 6-speed gearbox, differential
│   ├── Chassis         – Weight transfer, torsional flex, roll/pitch stiffness
│   ├── Vehicle         – Assembles all subsystems per-timestep
│   └── PhysicsEngine   – Fixed-timestep loop, multi-vehicle, callbacks
```

## Prerequisites (Ubuntu 24.04)

```bash
sudo apt-get install -y \
    build-essential cmake \
    qt6-base-dev qt6-charts-dev \
    libgtest-dev
```

## Build

```bash
cd physics_engine
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --parallel
```

### Debug build

```bash
cmake .. -DCMAKE_BUILD_TYPE=Debug
cmake --build . --parallel
```

## Run the UI

```bash
./build/car_physics_ui
```

## Run the tests

```bash
cd build
ctest --output-on-failure
# or run directly:
./run_tests
```

## Keyboard Controls

| Key | Action |
|-----|--------|
| W / ↑ | Throttle |
| S / ↓ | Brake |
| A / ← | Steer left |
| D / → | Steer right |
| Q | Shift up |
| E | Shift down |
| Space | Handbrake |
| R | Reset simulation |
| P | Pause / Resume |
| Scroll | Zoom simulation view |
| Left-drag | Pan simulation view |
| Right-click | Re-centre on car |

## Physics Details

### Tire model (Pacejka Magic Formula)
```
F = D · sin(C · atan(B·x − E·(B·x − atan(B·x))))
```
- Longitudinal force from slip ratio κ = (ωr − v) / |v|
- Lateral force from slip angle α = atan(vy / vx)
- Friction circle coupling: √(Fx² + Fy²) ≤ μ·Fz

### Suspension
- Spring-damper: F = k·x + c·ẋ
- Progressive bump stops within 10 mm of travel limits
- Anti-roll bar torque: k_arb · Δroll

### Aerodynamics
- Drag:      Fd = ½ρv²·Cd·A
- Downforce: Fl = ½ρv²·Cl·A·Kge (Kge = ground-effect multiplier)
- Ground effect: up to 1.6× downforce at minimum ride height

### Drivetrain
- Engine torque curve: Gaussian bell centred at peakTorqueRPM
- 6-speed gearbox with 80 ms shift-torque-cut interruption
- Wheel torque: T_wheel = T_engine · ratio · η_gearbox

### Weight Transfer
- Longitudinal: ΔW = m·ax·h/wb
- Lateral (roll stiffness distribution): ΔWf = rollMoment · (Kf / Ktotal) / twf
