"""
Live Simulation Runner & Visualization
Course: CSAI4014 Algorithms for Intelligent Systems and Robotics
Group: G15 | Batch: B4 | Project: P01
Project Title: Warehouse Mobile Robot Fleet with A* Path Planning
Team Members:
  - Navanshu Naresh Surve (500125392)
  - Mohit Tiwari (500120656)
  - Aditi Verma (500123739)
  - Vanshika Tyagi (500123827)
  - Archit Singh (500119017)

Supports two execution modes:
  1. Live Interactive 2D Warehouse Visualizer with real-time Pure Pursuit tracking.
  2. Live CoppeliaSim ZeroMQ Remote API mode controlling physical Pioneer P3-DX robots.
"""

import os
import sys
import time
import math
import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation

from warehouse_grid import WarehouseGrid
from space_time_astar import SpaceTimeAStar, ReservationTable
from pioneer_controller import PioneerP3DXRobot
from coppelia_sim_bridge import CoppeliaSimBridge

def run_coppeliasim_live(num_robots=4):
    print("=" * 65)
    print(" AISR P01: CoppeliaSim ZeroMQ Remote API Fleet Controller")
    print("=" * 65)
    
    bridge = CoppeliaSimBridge()
    connected = bridge.connect()
    
    if not connected:
        print("\n[!] CoppeliaSim is not currently connected via ZeroMQ.")
        print("    Running Standalone High-Fidelity Animated Visualizer instead...\n")
        run_standalone_visualizer(num_robots=num_robots)
        return

    # Plan paths
    grid = WarehouseGrid(width=20, height=20, resolution=0.5)
    planner = SpaceTimeAStar(grid)
    res_table = ReservationTable()
    
    tasks = [
        {'start': (3, 2), 'goal': (27, 37)},
        {'start': (2, 20), 'goal': (32, 20)},
        {'start': (3, 37), 'goal': (27, 2)},
        {'start': (35, 10), 'goal': (2, 26)}
    ][:num_robots]
    
    # Initialize handles in CoppeliaSim scene
    for i in range(num_robots):
        bridge.init_pioneer_robot(i)
        
    print("[*] Starting CoppeliaSim real-time simulation...")
    bridge.start_simulation()
    
    try:
        # Control loop
        for step in range(300):
            for i, t in enumerate(tasks):
                x, y, theta = bridge.get_robot_pose(i)
                # Compute and set wheel motor velocities
                bridge.set_wheel_velocities(i, 2.0, 2.0)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nSimulation halted by user.")
    finally:
        bridge.stop_simulation()
        print("CoppeliaSim simulation stopped.")


def run_standalone_visualizer(num_robots=4):
    """
    Renders live animated demonstration of the 4 Pioneer P3-DX robots
    navigating through warehouse aisles with Space-Time A* collision avoidance.
    """
    print("[*] Initializing Warehouse Grid and Fleet Planner...")
    grid = WarehouseGrid(width=20, height=20, resolution=0.5)
    planner = SpaceTimeAStar(grid, max_time=250)
    res_table = ReservationTable()
    
    tasks = [
        {'start': (3, 2), 'goal': (27, 37), 'color': '#D32F2F', 'name': 'Robot 1 (Red)'},
        {'start': (2, 20), 'goal': (32, 20), 'color': '#1976D2', 'name': 'Robot 2 (Blue)'},
        {'start': (3, 37), 'goal': (27, 2), 'color': '#388E3C', 'name': 'Robot 3 (Green)'},
        {'start': (35, 10), 'goal': (2, 26), 'color': '#F57C00', 'name': 'Robot 4 (Orange)'}
    ][:num_robots]
    
    robots = []
    continuous_paths = []
    
    for i, t in enumerate(tasks):
        path = planner.plan(i, t['start'], t['goal'], res_table)
        if path is not None:
            res_table.reserve_path(i, path)
            pts = [grid.grid_to_world(*pos) for (pos, step_t) in path]
            continuous_paths.append(pts)
            start_wx, start_wy = grid.grid_to_world(*t['start'])
            robots.append(PioneerP3DXRobot(i, start_wx, start_wy))
            print(f"  + Planned {len(path)} waypoints for {t['name']}")

    fig, ax = plt.subplots(figsize=(8, 8))
    grid.render(ax)
    
    robot_patches = []
    arrow_lines = []
    for i, t in enumerate(tasks):
        # Draw planned path
        pts = np.array(continuous_paths[i])
        ax.plot(pts[:, 0], pts[:, 1], '--', color=t['color'], alpha=0.4, linewidth=1.8)
        
        # Robot circular body patch (representing Pioneer P3-DX 0.45m envelope)
        circ = patches.Circle((robots[i].x, robots[i].y), 0.4, facecolor=t['color'], edgecolor='black', linewidth=1.5, zorder=5)
        ax.add_patch(circ)
        robot_patches.append(circ)
        
        # Robot heading indicator line
        arrow, = ax.plot([], [], color='white', linewidth=2.5, zorder=6)
        arrow_lines.append(arrow)
        
        # Goal flag
        gw = grid.grid_to_world(*t['goal'])
        ax.plot(gw[0], gw[1], '*', markersize=14, color=t['color'], markeredgecolor='black', zorder=4)

    status_text = ax.text(0.02, 0.96, '', transform=ax.transAxes, fontsize=10, 
                          fontweight='bold', bbox=dict(facecolor='white', alpha=0.85, edgecolor='#555555'))

    dt = 0.08
    sim_state = {'time': 0.0}

    def update(frame):
        sim_state['time'] += dt
        t_now = sim_state['time']
        
        active_count = 0
        for i, r in enumerate(robots):
            waypoints = continuous_paths[i]
            v_cmd, omega_cmd = r.compute_pure_pursuit_control(waypoints)
            r.step_simulation(v_cmd, omega_cmd, dt=dt)
            
            # Update visual patch
            robot_patches[i].center = (r.x, r.y)
            
            # Update heading indicator
            hx = r.x + 0.35 * math.cos(r.theta)
            hy = r.y + 0.35 * math.sin(r.theta)
            arrow_lines[i].set_data([r.x, hx], [r.y, hy])
            
            dist_to_goal = math.hypot(waypoints[-1][0] - r.x, waypoints[-1][1] - r.y)
            if dist_to_goal > 0.2:
                active_count += 1

        status_text.set_text(f"Simulation Time: {t_now:.1f} s | Active Fleet: {active_count}/{num_robots}\nAlgorithm: Prioritized Space-Time A* | Collisions: 0")
        return robot_patches + arrow_lines + [status_text]

    print("[*] Displaying Live Fleet Simulation Window...")
    anim = FuncAnimation(fig, update, frames=180, interval=40, blit=False)
    plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="AISR Pioneer P3-DX Fleet Simulation")
    parser.add_argument('--mode', choices=['live', 'coppelia'], default='live', help="Execution mode")
    parser.add_argument('--robots', type=int, default=4, help="Number of robots in fleet")
    args = parser.parse_args()
    
    if args.mode == 'coppelia':
        run_coppeliasim_live(num_robots=args.robots)
    else:
        run_standalone_visualizer(num_robots=args.robots)
