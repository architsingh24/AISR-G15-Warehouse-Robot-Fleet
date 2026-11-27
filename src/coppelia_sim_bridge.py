"""
CoppeliaSim Python ZeroMQ Remote API Bridge for Pioneer P3-DX Fleet
Course: CSAI4014 Algorithms for Intelligent Systems and Robotics
Group: G15 | Batch: B4 | Project: P01
Author: Aditi Verma (SAP ID: 500123739)

This script interfaces Python with CoppeliaSim via the ZeroMQ remote API.
Scene requirement:
  - Pioneer P3-DX models named '/PioneerP3DX', '/PioneerP3DX[0]', '/PioneerP3DX[1]', etc.
  - Left motor: 'Pioneer_p3dx_leftMotor'
  - Right motor: 'Pioneer_p3dx_rightMotor'
"""

import sys
import time
import math

class CoppeliaSimBridge:
    def __init__(self, host='localhost', port=23000):
        self.host = host
        self.port = port
        self.client = None
        self.sim = None
        self.is_connected = False
        self.robot_handles = {}

    def connect(self):
        """Attempts connection to CoppeliaSim via ZeroMQ remote API"""
        try:
            from coppeliasim_zmqremoteapi_client import RemoteAPIClient
            print(f"[CoppeliaSimBridge] Connecting to ZeroMQ API on {self.host}:{self.port}...")
            self.client = RemoteAPIClient(host=self.host, port=self.port)
            self.sim = self.client.require('sim')
            self.is_connected = True
            print("[CoppeliaSimBridge] Successfully connected to CoppeliaSim!")
            return True
        except ImportError:
            print("[CoppeliaSimBridge] Note: 'coppeliasim_zmqremoteapi_client' not installed.")
            print("  To install: pip install coppeliasim-zmqremoteapi-client")
            self.is_connected = False
            return False
        except Exception as e:
            print(f"[CoppeliaSimBridge] Could not connect to CoppeliaSim: {e}")
            self.is_connected = False
            return False

    def init_pioneer_robot(self, robot_id, object_name=None):
        """
        Discovers Pioneer P3-DX robot base and motor handles in the scene.
        """
        if not self.is_connected:
            return False

        if object_name is None:
            object_name = f"/PioneerP3DX[{robot_id}]" if robot_id > 0 else "/PioneerP3DX"

        try:
            base_handle = self.sim.getObject(object_name)
            left_motor = self.sim.getObject(f"{object_name}/Pioneer_p3dx_leftMotor")
            right_motor = self.sim.getObject(f"{object_name}/Pioneer_p3dx_rightMotor")
            
            self.robot_handles[robot_id] = {
                'base': base_handle,
                'left_motor': left_motor,
                'right_motor': right_motor,
                'name': object_name
            }
            print(f"[CoppeliaSimBridge] Initialized Robot {robot_id} ({object_name}) successfully.")
            return True
        except Exception as e:
            print(f"[CoppeliaSimBridge] Error initializing {object_name}: {e}")
            return False

    def set_wheel_velocities(self, robot_id, w_left, w_right):
        """Sets target angular velocities (rad/s) on left and right wheel joints"""
        if not self.is_connected or robot_id not in self.robot_handles:
            return
        
        handles = self.robot_handles[robot_id]
        self.sim.setJointTargetVelocity(handles['left_motor'], float(w_left))
        self.sim.setJointTargetVelocity(handles['right_motor'], float(w_right))

    def get_robot_pose(self, robot_id):
        """Returns (x, y, theta) of the robot in CoppeliaSim world frame"""
        if not self.is_connected or robot_id not in self.robot_handles:
            return 0.0, 0.0, 0.0

        handles = self.robot_handles[robot_id]
        pos = self.sim.getObjectPosition(handles['base'], self.sim.handle_world)
        ori = self.sim.getObjectOrientation(handles['base'], self.sim.handle_world)
        return pos[0], pos[1], ori[2]

    def start_simulation(self):
        if self.is_connected:
            self.sim.startSimulation()
            print("[CoppeliaSimBridge] Simulation started.")

    def stop_simulation(self):
        if self.is_connected:
            self.sim.stopSimulation()
            print("[CoppeliaSimBridge] Simulation stopped.")

    def step(self):
        if self.is_connected:
            self.client.step()
