#!/usr/bin/env python3
"""
Interactive Motion Planning Demo with OMPL, Pinocchio, and Viser

This demo provides an interactive 3D visualization for motion planning of a UR5 robot arm.
"""

import time
import os
import numpy as np

from ompl import base as ob
from ompl import geometric as og

import pinocchio

from ViserGUI import ViserGUI

# ============================================================================
# Configuration
# ============================================================================

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
URDF_PATH = os.path.join(SCRIPT_DIR, "../resources/ur5/ur5_spherized.urdf")
MESH_DIR = os.path.join(SCRIPT_DIR, "../resources/ur5")

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

NUM_JOINTS = len(JOINT_NAMES)
DEFAULT_CONFIG = np.array([-np.pi/2, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0.0])
PATH_INTERPOLATION_STEPS = 50


# ============================================================================
# Collision Checker
# ============================================================================

class CollisionChecker:
    """Collision checker using Pinocchio for FK and collision detection."""
    
    def __init__(self, urdf_path: str, mesh_dir: str, obstacles: dict = None):
        """
        Initialize collision checker.
        
        Args:
            urdf_path: Path to robot URDF
            mesh_dir: Directory containing mesh files
            obstacles: Dict of obstacles - {obs_id: {'position': np.ndarray, 'radius': float}}
        """
        self.model, self.collision_model, self.visual_model = \
            pinocchio.buildModelsFromUrdf(urdf_path, package_dirs=[mesh_dir])
        self.data = self.model.createData()
        
        # Add obstacles to collision model
        num_robot_geoms = self.collision_model.ngeoms
        if obstacles:
            for obs_id, obs_data in obstacles.items():
                sphere = pinocchio.GeometryObject(
                    f"obstacle_{obs_id}",
                    0,
                    pinocchio.SE3(np.eye(3), obs_data['position']),
                    pinocchio.hppfcl.Sphere(obs_data['radius'])
                )
                sphere.parentJoint = 0
                geom_id = self.collision_model.addGeometryObject(sphere)
                
                # Add collision pairs with all robot geometries
                for i in range(num_robot_geoms):
                    self.collision_model.addCollisionPair(pinocchio.CollisionPair(i, geom_id))
        
        self.collision_data = self.collision_model.createData()
        
    def is_state_valid(self, q: np.ndarray) -> bool:
        """
        Check if a joint configuration is collision-free.
        
        Args:
            q: Joint configuration
        
        Returns:
            True if collision-free, False otherwise.
        """
        
        q_full = np.zeros(self.model.nq)
        q_full[:NUM_JOINTS] = q
        
        is_collision = pinocchio.computeCollisions(
            self.model, self.data,
            self.collision_model, self.collision_data,
            q_full, True
        )
        
        return not is_collision
    
    def get_ee_positions(self, waypoints: list) -> list:
        """Get end-effector positions for a list of configurations."""
        ee_positions = []
        for q in waypoints:
            q_full = np.zeros(self.model.nq)
            q_full[:NUM_JOINTS] = q
            pinocchio.forwardKinematics(self.model, self.data, q_full)
            ee_frame_id = self.model.nframes - 1
            pinocchio.updateFramePlacement(self.model, self.data, ee_frame_id)
            ee_positions.append(self.data.oMf[ee_frame_id].translation.copy())
        return ee_positions


# ============================================================================
# Motion Planner
# ============================================================================

class MotionPlanner:
    """RRTConnect motion planner using OMPL."""
    
    def __init__(self, urdf_path: str, mesh_dir: str):
        self.urdf_path = urdf_path
        self.mesh_dir = mesh_dir
        
        # Load model once to get joint limits
        model, _, _ = pinocchio.buildModelsFromUrdf(urdf_path, package_dirs=[mesh_dir])
        self.lower_limits = model.lowerPositionLimit[:NUM_JOINTS]
        self.upper_limits = model.upperPositionLimit[:NUM_JOINTS]
        
    def plan(self, start_config: np.ndarray, goal_config: np.ndarray, 
             timeout: float, obstacles: dict = None) -> tuple:
        """
        Plan a path from start to goal using RRTConnect.
        
        Args:
            start_config: Start joint configuration
            goal_config: Goal joint configuration
            timeout: Planning timeout in seconds
            obstacles: Dict of obstacles - {obs_id: {'position': np.ndarray, 'radius': float}}
        """
        # Create collision checker with current obstacles
        collision_checker = CollisionChecker(self.urdf_path, self.mesh_dir, obstacles)
        
        space = ob.RealVectorStateSpace(NUM_JOINTS)
        
        bounds = ob.RealVectorBounds(NUM_JOINTS)
        for i in range(NUM_JOINTS):
            bounds.setLow(i, float(self.lower_limits[i]))
            bounds.setHigh(i, float(self.upper_limits[i]))
        space.setBounds(bounds)
        
        ss = og.SimpleSetup(space)
        ss.setStateValidityChecker(lambda state: collision_checker.is_state_valid(
            np.array([state[i] for i in range(NUM_JOINTS)])
        ))
        
        start = space.allocState()
        goal = space.allocState()
        for i in range(NUM_JOINTS):
            start[i] = float(start_config[i])
            goal[i] = float(goal_config[i])
        
        si = ss.getSpaceInformation()
        if not si.isValid(start):
            return False, None, "Start configuration is in collision!"
        if not si.isValid(goal):
            return False, None, "Goal configuration is in collision!"
        
        ss.setStartAndGoalStates(start, goal)
        ss.setPlanner(og.RRTConnect(si))
        
        solved = ss.solve(timeout)
        
        if solved:
            ss.simplifySolution()
            path = ss.getSolutionPath()
            path.interpolate(PATH_INTERPOLATION_STEPS)
            
            waypoints = []
            for i in range(path.getStateCount()):
                state = path.getState(i)
                waypoints.append(np.array([state[j] for j in range(NUM_JOINTS)]))
            
            return True, waypoints, f"Path found with {len(waypoints)} waypoints!"
        
        return False, None, "No solution found within the time limit."


# ============================================================================
# Main
# ============================================================================

def main():
    print("=" * 60)
    print("Interactive Motion Planning Demo")
    print("=" * 60)
    print(f"Loading robot from: {URDF_PATH}")
    
    # Initialize planner
    planner = MotionPlanner(URDF_PATH, MESH_DIR)
    
    # Base collision checker (no obstacles) for joint limits and FK
    base_checker = CollisionChecker(URDF_PATH, MESH_DIR)
    
    print(f"Robot loaded with {NUM_JOINTS} joints")
    print("Joint limits:")
    for i, name in enumerate(JOINT_NAMES):
        print(f"  {name}: [{planner.lower_limits[i]:.2f}, {planner.upper_limits[i]:.2f}]")
    
    # Create GUI (handles visualization internally)
    gui = ViserGUI(
        urdf_path=URDF_PATH,
        mesh_dir=MESH_DIR,
        joint_names=JOINT_NAMES,
        joint_limits_lower=planner.lower_limits,
        joint_limits_upper=planner.upper_limits,
        initial_config=DEFAULT_CONFIG,
        port=8080,
    )
    
    # Connect callbacks
    gui.on_plan_requested = lambda start, goal, timeout: planner.plan(start, goal, timeout, gui.get_obstacles())
    gui.on_collision_check_requested = lambda config: CollisionChecker(URDF_PATH, MESH_DIR, gui.get_obstacles()).is_state_valid(config)
    gui.on_path_visualization_requested = lambda waypoints: base_checker.get_ee_positions(waypoints)
    
    # Main loop
    while True:
        gui.update()
        time.sleep(0.02)


if __name__ == "__main__":
    main()
