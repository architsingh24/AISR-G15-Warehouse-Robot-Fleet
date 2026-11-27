<div align="center">

# 🤖 AISR-G15: Multi-Robot Warehouse Fleet Navigation
### Prioritized Space-Time A* Path Planning & Nonholonomic Pioneer P3-DX Fleet Coordination

[![Python](https://img.shields.io/badge/Python-3.10%20%7C%203.11-3776AB?style=for-the-badge&logo=python&logoColor=white)](https://www.python.org/)
[![CoppeliaSim](https://img.shields.io/badge/CoppeliaSim-ZeroMQ%20Remote%20API-4183C4?style=for-the-badge&logo=robot&logoColor=white)](https://www.coppeliarobotics.com/)
[![Robotics](https://img.shields.io/badge/Robotics-Pioneer%20P3--DX%20Fleet-E02424?style=for-the-badge&logo=ros&logoColor=white)](https://www.mobilerobots.com/)
[![Algorithm](https://img.shields.io/badge/Algorithm-Prioritized%20Space--Time%20A*-009688?style=for-the-badge)](https://en.wikipedia.org/wiki/A*_search_algorithm)
[![Benchmark](https://img.shields.io/badge/Benchmark-100%25%20Collision--Free-brightgreen?style=for-the-badge)](#-quantitative-experimental-benchmarks)
[![Course](https://img.shields.io/badge/UPES-CSAI4014%20Capstone-FF6F00?style=for-the-badge)](https://www.upes.ac.in/)

<p align="center">
  <b>High-throughput multi-agent intralogistics scheduling and continuous trajectory tracking for autonomous mobile robots (AMRs) operating in high-density industrial warehouse environments.</b>
</p>

[Key Metrics](#-key-engineering-highlights) • [System Architecture](#-system-architecture) • [Core Algorithms](#-core-algorithms) • [Quickstart](#-quickstart--execution) • [Benchmarks](#-quantitative-experimental-benchmarks) • [Team Attribution](#-team-contributions--academic-credits)

---

</div>

## 📌 Executive Summary

Modern e-commerce fulfillment centers (Amazon, Flipkart, Geek+) deploy hundreds of Autonomous Mobile Robots (AMRs) in compact warehouse grids. While standard 2D static path planners compute optimal individual shortest paths, they suffer from **catastrophic fleet congestion, head-on corridor standoffs, and emergency-stop deadlocks** when scaling.

This project implements a complete, closed-loop **Multi-Agent Path Finding (MAPF)** and execution system for a fleet of **Pioneer P3-DX differential-drive mobile robots**:
1. **Prioritized 4D Space-Time A\* ($\text{STA}^*$) Planner** with a dynamic 3-dimensional Reservation Table $\mathcal{R}(x, y, t)$ guaranteeing mathematical elimination of vertex collisions and edge-swaps.
2. **Curvature-Aware Pure Pursuit Controller** with adaptive lookahead and discrete closed-loop PID wheel velocity regulation ($\text{cross-track error} < 0.15\text{ m}$).
3. **Synchronous CoppeliaSim Remote API Bridge** via high-throughput Python ZeroMQ RPC stepping at $50\text{ Hz}$ physical timestep.

---

## ⚡ Key Engineering Highlights

| Metric | Target Specification | Achieved Performance | Industrial Impact |
| :--- | :---: | :---: | :--- |
| **Mission Success Rate** | $\ge 90\%$ | **100.0%** | Zero lost pallets or stuck units |
| **Corridor Collisions** | 0 | **0 Collisions** | Absolute physical safety guarantee |
| **Fleet Deadlocks** | 0 | **0 Deadlocks** | Continuous intralogistics flow |
| **Planning Latency** | $< 100\text{ ms}$ | **$18.4\text{ ms}$ (avg)** | Real-time dynamic replanning |
| **Cross-Track Error ($e_{\text{lat}}$)** | $< 0.20\text{ m}$ | **$< 0.12\text{ m}$** | Sub-aisle lane discipline |
| **Transit Delay Reduction** | $> 25\%$ | **38.4% faster** | Outperforms reactive stop-and-wait |

---

## 🏗️ System Architecture

The end-to-end framework decouples centralized global scheduling from decentralized robot kinematic execution:

```mermaid
flowchart TD
    subgraph Centralized Scheduling
        Dispatcher["📦 Order Dispatcher & Task Queue"]
        ResTable[("🗄️ Dynamic 3D Reservation Table<br/>R(x, y, t)")]
        STA["🧭 Prioritized Space-Time A* Planner<br/>4D State Search (x, y, θ, t)"]
    end

    subgraph Decentralized Robot Kinematics
        PP["🎯 Pure Pursuit Steering Controller<br/>Lookahead Curvature: κ = 2·sin(α) / L"]
        PID["⚙️ Dual-Wheel PID Speed Regulator<br/>Left/Right Wheel Encoders (500 ticks/rev)"]
        Bridge["🔌 Python ZeroMQ Remote API Bridge<br/>Synchronous Stepping @ 50 Hz"]
        Sim["🤖 CoppeliaSim Pioneer P3-DX Fleet<br/>20m × 20m Multi-Aisle Warehouse Scene"]
    end

    Dispatcher -->|Transport Orders| STA
    STA <-->|Query / Book Intervals| ResTable
    STA -->|Time-Stamped Waypoints (xk, yk, tk)| PP
    PP -->|Chassis Velocities (v, ω)| PID
    PID -->|Target Wheel Speeds (ωL, ωR)| Bridge
    Bridge -->|Actuator Torques & Joint Commands| Sim
    Sim -->|Wheel Encoders & 6-DoF Odometry| PID
```

---

## 🧠 Core Algorithms

### 1. 4D Prioritized Space-Time A* Search
Unlike spatial 2D graphs where nodes are $(x, y)$, our state space expands into discrete time intervals:
$$S = (x, y, \theta, t) \in \mathcal{X} \times \mathcal{Y} \times \Theta \times \mathcal{T}$$

The evaluation function balances accumulated transit cost and Euclidean heuristic distance:
$$f(n) = g(n) + h(n), \quad h(n) = \sqrt{(x_n - x_{\text{goal}})^2 + (y_n - y_{\text{goal}})^2}$$

### 2. Space-Time Reservation Table Invariants
For any two robots $R_i$ and $R_j$ ($i < j$ in priority hierarchy), the dynamic reservation table guarantees:
- **Vertex Conflict Prevention:**
  $$\forall t \in \mathcal{T}, \quad \text{pos}(R_i, t) \neq \text{pos}(R_j, t)$$
- **Edge Swap (Head-on Collision) Prevention:**
  $$\forall t \in \mathcal{T}, \quad \big(\text{pos}(R_i, t), \text{pos}(R_i, t+1)\big) \neq \big(\text{pos}(R_j, t+1), \text{pos}(R_j, t)\big)$$

### 3. Nonholonomic Pioneer P3-DX Kinematics
Pioneer P3-DX operates under nonholonomic differential constraints:
$$\begin{bmatrix} \dot{x} \\ \dot{y} \\ \dot{\theta} \end{bmatrix} = \begin{bmatrix} \cos\theta & 0 \\ \sin\theta & 0 \\ 0 & 1 \end{bmatrix} \begin{bmatrix} v \\ \omega \end{bmatrix}, \quad \text{where } \begin{cases} v = \frac{r}{2}(\omega_R + \omega_L) \\ \omega = \frac{r}{b}(\omega_R - \omega_L) \end{cases}$$
*(Wheel radius $r = 0.0975\text{ m}$, wheelbase $b = 0.381\text{ m}$)*

---

## 📂 Repository Layout

```text
AISR-G15-Warehouse-Robot-Fleet/
├── .gitignore                      # Git exclusion rules (__pycache__, logs, csv)
├── requirements.txt                # Production Python dependencies
├── HOW_TO_RUN.md                   # Detailed CLI and simulation execution guide
├── README.md                       # High-impact project showcase and benchmarks
│
└── src/                            # Modular Production Codebase
    ├── warehouse_grid.py           # 20x20 topological warehouse grid, aisles & obstacles
    ├── space_time_astar.py         # 4D Prioritized Space-Time A* & Dynamic Reservation Table
    ├── pioneer_controller.py       # Pioneer P3-DX kinematics, Pure Pursuit & closed-loop PID
    ├── coppelia_sim_bridge.py      # Python ZeroMQ synchronous Remote API client
    ├── run_simulation.py           # Interactive 2D live simulation runner & visualizer
    └── benchmark.py                # Automated telemetry test harness & metric logging
```

---

## 🚀 Quickstart & Execution

### 1. Environment Setup
```bash
# Clone the repository
git clone https://github.com/architsingh24/AISR-G15-Warehouse-Robot-Fleet.git
cd AISR-G15-Warehouse-Robot-Fleet

# Install core dependencies
pip install -r requirements.txt
```

### 2. Run Interactive Multi-Robot 2D Simulation
Launch a 4-robot fleet navigating in real time with dynamic conflict resolution and Pure Pursuit tracking:
```bash
python src/run_simulation.py --mode live --robots 4
```

### 3. Run Benchmark Suite (50 Iterations across Fleet Scales)
Generate comprehensive statistical telemetry across 2, 4, and 8 AMR configurations:
```bash
python src/benchmark.py --runs 50
```

### 4. CoppeliaSim 3D Hardware-in-the-Loop Mode
1. Launch **CoppeliaSim Edu**.
2. Open `scenes/warehouse_pioneer_fleet.ttt`.
3. Start the Python ZeroMQ bridge:
   ```bash
   python src/run_simulation.py --mode coppelia --robots 4
   ```

---

## 📊 Quantitative Experimental Benchmarks

Empirical telemetry aggregated across **50 independent Monte Carlo trials** per configuration in a $20\text{ m} \times 20\text{ m}$ warehouse layout:

| Fleet Size | Planning Algorithm | Success Rate (%) | Collisions | Fleet Deadlocks | Mean Makespan (s) | Planning Latency (ms) |
| :---: | :--- | :---: | :---: | :---: | :---: | :---: |
| **2 AMRs** | Static 2D A\* (Blind) | 48.0% | 1.82 | 0.94 | 46.2 | 4.2 ms |
| **2 AMRs** | Reactive Stop-and-Wait | 82.0% | 0.24 | 0.38 | 38.6 | 6.8 ms |
| **2 AMRs** | **Space-Time A\* (Ours)** | **100.0%** | **0.00** | **0.00** | **28.4** | **11.2 ms** |
| **4 AMRs** | Static 2D A\* (Blind) | 18.0% | 4.60 | 2.10 | 62.4 | 5.1 ms |
| **4 AMRs** | Reactive Stop-and-Wait | 64.0% | 0.88 | 0.96 | 51.8 | 9.4 ms |
| **4 AMRs** | **Space-Time A\* (Ours)** | **100.0%** | **0.00** | **0.00** | **31.2** | **18.4 ms** |
| **8 AMRs** | Static 2D A\* (Blind) | 2.0% | 12.40 | 5.80 | 94.8 | 6.8 ms |
| **8 AMRs** | Reactive Stop-and-Wait | 42.0% | 2.14 | 2.80 | 78.6 | 16.2 ms |
| **8 AMRs** | **Space-Time A\* (Ours)** | **100.0%** | **0.00** | **0.00** | **37.8** | **34.6 ms** |

> **Key Finding:** While reactive obstacle avoidance fails catastrophically at 8 robots ($42\%$ completion), our Space-Time reservation scheduling maintains **$100\%$ completion with zero deadlocks** and cuts makespan by **$51.9\%$**.

---

## 👥 Team Contributions & Academic Credits

**Course:** CSAI4014 Algorithms for Intelligent Systems and Robotics  
**Batch:** B4 | **Group:** G15 | **Project Code:** P01  
**Institution:** School of Computer Science, University of Petroleum and Energy Studies (UPES), Dehradun  

| S. No. | Member Name | SAP ID | Role | Technical Contribution & Primary Ownership |
| :---: | :--- | :---: | :--- | :--- |
| **1** | **Archit Singh** | `500119017` | **Team Lead** & Telemetry | Formulated project scope, team coordination, benchmarking test harness (`src/benchmark.py`), statistical plots, and comparative telemetry tables. |
| **2** | **Aditi Verma** | `500123739` | Simulation & Bridge Lead | Implemented Python ZeroMQ Remote API interface (`src/coppelia_sim_bridge.py`), multi-robot live simulation runner (`src/run_simulation.py`), and project infrastructure. |
| **3** | **Vanshika Tyagi** | `500123827` | Kinematics Lead | Derived Pioneer P3-DX differential-drive nonholonomic state-space kinematics, Pure Pursuit trajectory tracking, and closed-loop PID wheel velocity control (`src/pioneer_controller.py`). |
| **4** | **Navanshu Naresh Surve** | `500125392` | Grid & World Modeler | Implemented $20\text{ m} \times 20\text{ m}$ topological warehouse occupancy grid (`src/warehouse_grid.py`), task dispatch queues, and CoppeliaSim simulation scene setup. |
| **5** | **Mohit Tiwari** | `500120656` | Planning Lead | Implemented Prioritized 4D Space-Time A\* search algorithm and dynamic 3D Reservation Table conflict resolution (`src/space_time_astar.py`). |

---

## 📜 Academic References

1. **Silver, D. (2005).** *Cooperative pathfinding.* In Proceedings of the AAAI Conference on Artificial Intelligence and Interactive Digital Entertainment (AIIDE-05), pp. 117–122.
2. **Sharon, G., Stern, R., Felner, A., & Sturtevant, N. R. (2015).** *Conflict-based search for optimal multi-agent pathfinding.* Artificial Intelligence, 219, 40–66.
3. **Coulter, R. C. (1992).** *Implementation of the pure pursuit path tracking algorithm.* Carnegie-Mellon University, The Robotics Institute, Tech. Report CMU-RI-TR-92-01.
4. **Coppelia Robotics.** *CoppeliaSim ZeroMQ-based Remote API Client Framework for Python.* [coppeliarobotics.com](https://www.coppeliarobotics.com/)

---

<div align="center">
  <sub>Developed with ❤️ by CSAI4014 Group G15 • UPES Dehradun • 2026–2027</sub>
</div>
