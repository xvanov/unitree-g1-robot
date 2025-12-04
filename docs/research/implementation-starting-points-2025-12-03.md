# Implementation Starting Points Research

**Date:** 2025-12-03
**Author:** PM Research Session
**Purpose:** Identify existing repos, frameworks, and starting points for unitree-g1-robot MVP implementation

---

## Executive Summary

This document captures research into what already exists that can be leveraged for the construction site inspection robot MVP. Key finding: **The G1 has built-in walking via SDK high-level commands on the real robot, but these are NOT available in simulation.** This creates a critical architectural decision point.

---

## 1. Locomotion Options

### 1.1 Built-in SDK High-Level Commands (Real Robot Only)

The Unitree G1 ships with working locomotion out of the box via `unitree_sdk2`.

**Available Commands:**
| Command | Description |
|---------|-------------|
| `StandUp()` | Robot stands from sitting |
| `Sit()` | Robot sits down |
| `Move(vx, vy, vyaw)` | Walk with velocity commands |
| `SetVelocity(vx, vy, omega, duration)` | Timed velocity movement |
| `BalanceStand()` | Balance in place |
| `HighStand()` / `LowStand()` | Adjust standing height |
| `ContinuousGait()` | Continuous walking mode |

**Example Usage:**
```python
from unitree_sdk2 import LocoClient

client = LocoClient()
client.StandUp()
client.SetVelocity(vx=0.3, vy=0.0, omega=0.0, duration=2.0)  # Walk forward
client.SetVelocity(vx=0.0, vy=0.0, omega=0.5, duration=1.0)  # Turn
```

**Critical Limitation:** These commands only work on the physical robot. They are NOT available in any simulator (MuJoCo, Isaac Sim, Gazebo).

**Source:** [unitree_sdk2 G1 Documentation](https://deepwiki.com/unitreerobotics/unitree_sdk2/3-g1-humanoid-robot)

---

### 1.2 RL-Based Locomotion (Works in Sim and Real)

For simulation-based development, RL policies provide locomotion that works in both sim and real.

**Key Repository:** [unitree_rl_gym](https://github.com/unitreerobotics/unitree_rl_gym)
- **Pre-trained checkpoint:** `deploy/pre_train/g1/motion.pt`
- **Supports:** Go2, H1, H1_2, G1
- **Training time:** Hours on good GPU (not hundreds of hours)
- **Workflow:** Train → Play → Sim2Sim (MuJoCo) → Sim2Real

**Other Locomotion Frameworks:**
| Framework | Link | Notes |
|-----------|------|-------|
| unitree_rl_lab | [GitHub](https://github.com/unitreerobotics/unitree_rl_lab) | IsaacLab-based, native G1-29dof support |
| unitree_sim_isaaclab | [GitHub](https://github.com/unitreerobotics/unitree_sim_isaaclab) | Official Unitree simulation for Isaac Lab |
| PBHC/KungfuBot | [GitHub](https://github.com/TeleHuman/PBHC) | Motion imitation, not velocity-command walking |

---

### 1.3 PBHC Assessment (Already in ~/PBHC)

**What PBHC provides:**
- Complete motion tracking framework (human video → SMPL → G1 motion)
- IsaacGym training pipeline
- MuJoCo deployment (sim2sim)
- Real robot deployment interface (`URCIRobot` class)
- G1 23-DOF support
- NeurIPS 2025 accepted

**What PBHC does NOT provide:**
- Velocity command walking (it's motion imitation, not locomotion control)
- Navigation integration
- Sensor-based obstacle avoidance
- High-level task abstraction

**Recommendation:** PBHC is wrong abstraction for "walk from A to B" navigation. Better for complex whole-body motions like dancing/kung-fu. Use unitree_rl_gym for locomotion instead.

---

## 2. Simulation Environment Options

### 2.1 Simulator Comparison

| Simulator | High-Level Commands | Low-Level Control | G1 Support |
|-----------|--------------------|--------------------|------------|
| Real Robot | ✅ Yes | ✅ Yes | ✅ Native |
| MuJoCo | ❌ No | ✅ Yes | ✅ Yes |
| Isaac Sim/Lab | ❌ No | ✅ Yes | ✅ Yes |
| Gazebo | ❌ No | ✅ Yes | ✅ Yes |

**Key Insight:** All simulators only support low-level joint control. High-level SDK commands (LocoClient, Move, SetVelocity) only work on real hardware.

**Source:** [unitree_mujoco Issue #29](https://github.com/unitreerobotics/unitree_mujoco/issues/29)

---

### 2.2 Development Path Options

#### Path 1: RL-Based (Recommended for Robust Sim Testing)
```
Simulation:
├── Use unitree_rl_gym pre-trained G1 policy
├── Nav2 velocity commands → RL policy wrapper
├── Same code works in sim AND real
└── Pre-trained: deploy/pre_train/g1/motion.pt

Real Robot:
└── Deploy same RL policy (sim2real)
```

**Pros:** Proper sim testing, smaller sim-to-real gap
**Cons:** More setup complexity

#### Path 2: Fake Locomotion Abstraction (Faster Prototyping)
```
Simulation:
├── "Fake" locomotion - teleport robot base directly
├── No actual walking physics simulated
├── Test navigation, perception, SLAM only
└── Robot glides perfectly (always succeeds)

Real Robot:
└── Switch to SDK LocoClient commands
```

**Pros:** Simple setup, fast iteration on nav/perception
**Cons:** No locomotion testing in sim, larger sim-to-real gap

**When Path 2 Makes Sense:**
- Prototyping navigation/perception quickly
- Trust SDK walking for flat floors
- Real robot testing is accessible
- Not doing extreme terrain initially

---

## 3. Navigation & SLAM Stack

### 3.1 Core Components

| Component | Recommended | Link |
|-----------|-------------|------|
| 2D SLAM | slam_toolbox | [GitHub](https://github.com/SteveMacenski/slam_toolbox) |
| Path Planning | Nav2 | ROS2 Navigation Stack |
| ROS2 Interface | unitree_ros2 | [GitHub](https://github.com/unitreerobotics/unitree_ros2) |

### 3.2 Plan-Based Localization

**Relevant Project:** [Ogm2Pgbm](https://github.com/MigVega/Ogm2Pgbm)
- Robust BIM-based 2D-LiDAR localization
- Uses Cartographer/SLAM toolbox for local mapping
- Matches against pre-built occupancy grid from BIM

**Challenges for Construction Plans:**
| Challenge | Why It's Hard |
|-----------|--------------|
| Plan ≠ Reality | Active site has temporary walls, materials blocking spaces |
| 2D Plan → 3D World | LiDAR sees 3D, plan is 2D |
| Dynamic Environment | Site changes daily |
| Plan Accuracy | As-built differs from as-designed |
| Feature Sparsity | Empty rooms have identical signatures |

**Recommendation:** Start with pure SLAM (build map live). Add plan overlay for report correlation later. Reduces Day 1 complexity.

---

## 4. Defect Detection / Computer Vision

### 4.1 Available Datasets

| Dataset | Link | Contents |
|---------|------|----------|
| BD3 Dataset | [GitHub](https://github.com/Praveenkottari/BD3-Dataset) | 3,965+ annotated building defect images (structural) |
| awesome-industrial-anomaly-detection | [GitHub](https://github.com/M-3LAB/awesome-industrial-anomaly-detection) | Curated papers and methods |

### 4.2 Gap Analysis

**What exists:** Structural defect detection (cracks, spalls, damage)
**What doesn't exist:** Finishes-trade specific detection (wrong tile position, scratched surfaces, location errors vs. plan)

**Recommendation:**
- Use foundation models (SAM, DINO) with fine-tuning
- Collect own training data for finishes-specific defects
- BD3 can inform approach but won't transfer directly

---

## 5. Recommended Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     MVP STACK                                    │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  LOCOMOTION:                                                     │
│  ├── Sim: unitree_rl_lab pre-trained G1 policy                  │
│  │        OR fake locomotion abstraction                        │
│  └── Real: SDK LocoClient OR same RL policy                     │
│                                                                  │
│  ROS2 BRIDGE: unitree_ros2 + CycloneDDS                         │
│                                                                  │
│  SLAM: slam_toolbox (2D) + LiDAR (MID-360)                      │
│                                                                  │
│  NAV STACK: Nav2 for path planning                              │
│                                                                  │
│  PERCEPTION: Intel RealSense D435i → custom defect detection    │
│                                                                  │
│  REPORTING: PDF generation                                       │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 6. Critical Gaps (What Doesn't Exist)

1. **Plan-to-LiDAR direct matching for construction PDFs** - Ogm2Pgbm uses BIM, not 2D PDFs
2. **Finishes trade defect detection model** - BD3 is structural defects only
3. **Stair-climbing integration with Nav2** - Locomotion exists, nav stack integration is custom
4. **End-to-end G1 inspection pipeline** - No one has built exactly this
5. **High-level SDK commands in simulation** - Must use RL policies or fake locomotion

---

## 7. Key Repositories Summary

### Official Unitree
| Repo | Purpose | Link |
|------|---------|------|
| unitree_sdk2 | Robot control SDK | [GitHub](https://github.com/unitreerobotics/unitree_sdk2) |
| unitree_sdk2_python | Python bindings | [GitHub](https://github.com/unitreerobotics/unitree_sdk2_python) |
| unitree_ros2 | ROS2 interface | [GitHub](https://github.com/unitreerobotics/unitree_ros2) |
| unitree_rl_gym | RL locomotion (IsaacGym) | [GitHub](https://github.com/unitreerobotics/unitree_rl_gym) |
| unitree_rl_lab | RL locomotion (IsaacLab) | [GitHub](https://github.com/unitreerobotics/unitree_rl_lab) |
| unitree_mujoco | MuJoCo simulation | [GitHub](https://github.com/unitreerobotics/unitree_mujoco) |
| unitree_sim_isaaclab | Isaac Lab simulation | [GitHub](https://github.com/unitreerobotics/unitree_sim_isaaclab) |

### Third-Party / Research
| Repo | Purpose | Link |
|------|---------|------|
| PBHC/KungfuBot | Motion imitation | [GitHub](https://github.com/TeleHuman/PBHC) |
| PHC | Perpetual humanoid control | [GitHub](https://github.com/ZhengyiLuo/PHC) |
| ASAP | Sim2Real refinement | [GitHub](https://github.com/LeCAR-Lab/ASAP) |
| slam_toolbox | ROS2 SLAM | [GitHub](https://github.com/SteveMacenski/slam_toolbox) |
| Ogm2Pgbm | Plan-based localization | [GitHub](https://github.com/MigVega/Ogm2Pgbm) |
| HouseExpo | 2D floor plan dataset | [GitHub](https://github.com/TeaganLi/HouseExpo) |

---

## 8. Next Steps / Decisions Needed

1. **Choose simulation strategy:** RL-based locomotion vs. fake locomotion abstraction
2. **Choose locomotion for real robot:** SDK high-level commands vs. RL policy deployment
3. **Validate SDK walking:** Test on flat floor and stairs with real G1
4. **Plan parsing approach:** How to convert construction PDFs to usable format
5. **Defect detection strategy:** Fine-tune existing models vs. train from scratch

---

## 9. Risk-Ordered Validation Sequence

Per PRD, validate in this order:
1. **Self-localization from plans** (highest risk) - May need to defer, start with pure SLAM
2. **Autonomous navigation including stairs** - Test SDK walking capability first
3. **Defect detection accuracy** - Start with obvious defects
4. **End-to-end inspection cycle** - Integration testing
5. **Blue tape placement** (deferred from MVP)

---

*Document generated from PM research session exploring implementation starting points for unitree-g1-robot project.*
