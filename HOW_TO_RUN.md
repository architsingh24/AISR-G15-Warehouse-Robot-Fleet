# Quick Start Guide: Warehouse Mobile Robot Fleet (CSAI4014 AISR)
**Group G15 | Batch B4 | UPES Dehradun**

Follow these 2 simple steps to run the simulation on any computer (Windows / Mac / Linux):

---

### Step 1: Install Python Libraries
Open terminal / command prompt in this project folder and run:
```bash
pip install -r requirements.txt
```
*(Only `numpy` and `matplotlib` are required for the live 2D visualizer).*

---

### Step 2: Run the Live Interactive Simulation
To launch the real-time animated warehouse simulation with 4 Pioneer P3-DX robots:
```bash
python src/run_simulation.py --mode live --robots 4
```
*(or run: `python run_simulation.py --mode live --robots 4`)*

A live simulation window will pop up showing the 20m x 20m warehouse grid, storage racks, dispatch stations, and 4 Pioneer robots dynamically coordinating and avoiding collisions in real time.

---

### Step 3: Run the Benchmarks & Generate Report Figures
To run the automated performance comparison (Baseline vs Space-Time A*) and regenerate all 4 project figures:
```bash
python benchmark.py
```
Output figures will be saved in `report/figures/`.

---

### Step 4 (Optional): Run with 3D CoppeliaSim Physics Simulator
If you have **CoppeliaSim Edu** installed:
1. Open CoppeliaSim.
2. Run:
```bash
python src/run_simulation.py --mode coppelia --robots 4
```
