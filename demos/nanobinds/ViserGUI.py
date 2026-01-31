"""
Viser GUI Manager for Robot Visualization and Motion Planning

A pure visualization module that provides interactive GUI elements for:
- Robot joint configuration control
- Start/Goal configuration selection  
- Obstacle management
- Path animation

This module does NOT handle planning or collision checking internally.
Use callbacks to connect to external planners and collision checkers.
"""

import os
import numpy as np
import trimesh
import viser
from viser.extras import ViserUrdf
import yourdfpy
from typing import Callable, Optional, List


class ViserGUI:
    """
    Interactive Viser GUI for robot visualization and motion planning interface.
    
    This is a pure visualization class. Planning and collision checking
    should be handled externally via callbacks.
    """
    
    def __init__(
        self,
        urdf_path: str,
        mesh_dir: str,
        joint_names: List[str],
        joint_limits_lower: np.ndarray,
        joint_limits_upper: np.ndarray,
        initial_config: np.ndarray,
        port: int = 8080,
    ):
        """
        Initialize the Viser GUI.
        
        Args:
            urdf_path: Path to robot URDF file
            mesh_dir: Directory containing mesh files
            joint_names: List of joint names
            joint_limits_lower: Lower joint limits
            joint_limits_upper: Upper joint limits
            initial_config: Initial joint configuration
            port: Viser server port
        """
        self.urdf_path = urdf_path
        self.mesh_dir = mesh_dir
        self.joint_names = joint_names
        self.num_joints = len(joint_names)
        self.joint_limits_lower = joint_limits_lower
        self.joint_limits_upper = joint_limits_upper
        
        # State
        self.current_config = initial_config.copy()
        self.start_config: Optional[np.ndarray] = None
        self.goal_config: Optional[np.ndarray] = None
        self.planned_path: Optional[List[np.ndarray]] = None
        self.path_index = 0
        self.is_animating = False
        self.sphere_counter = 0
        self.animation_speed = 1.0
        self.planning_timeout = 5.0
        
        # Callbacks (set by user)
        self.on_plan_requested: Optional[Callable] = None  # (start, goal, timeout) -> (success, path, message)
        self.on_collision_check_requested: Optional[Callable] = None  # (config) -> bool
        self.on_path_visualization_requested: Optional[Callable] = None  # (waypoints) -> List[np.ndarray] (ee positions)
        
        # Start server and setup visualization
        self.server = viser.ViserServer(port=port)
        self._setup_visualization()
        self._setup_gui()
        
    def _setup_visualization(self):
        """Setup 3D visualization."""
        # Add ground grid
        self.server.scene.add_grid(
            "/ground", width=2.0, height=2.0, cell_size=0.1,
            cell_color=(100, 100, 100), plane="xy"
        )
        
        # Load robot URDF
        def filename_handler(fname):
            if fname.startswith("package://"):
                return os.path.join(self.mesh_dir, fname[len("package://"):])
            elif fname.startswith("meshes/"):
                return os.path.join(self.mesh_dir, fname)
            return fname
        
        urdf = yourdfpy.URDF.load(self.urdf_path, filename_handler=filename_handler)
        self.urdf_vis = ViserUrdf(self.server, urdf, root_node_name="/robot")
        
        # Visualization handles
        self.path_handle = None
        self.obstacle_handles = {}
        
        # Set initial config
        self.update_robot(self.current_config)
        
    def _setup_gui(self):
        """Setup all GUI elements."""
        self.joint_sliders = {}
        self._setup_joint_sliders()
        self._setup_planning_controls()
        self._setup_obstacle_controls()
        self._setup_utilities()
        
    def _setup_joint_sliders(self):
        """Setup joint configuration sliders."""
        with self.server.gui.add_folder("Joint Configuration"):
            for i, name in enumerate(self.joint_names):
                slider = self.server.gui.add_slider(
                    name,
                    min=float(self.joint_limits_lower[i]),
                    max=float(self.joint_limits_upper[i]),
                    step=0.01,
                    initial_value=float(self.current_config[i]),
                )
                self.joint_sliders[name] = slider
                
                def make_callback(idx, slider_ref):
                    @slider_ref.on_update
                    def _(_):
                        if not self.is_animating:
                            self.current_config[idx] = slider_ref.value
                            self.update_robot(self.current_config)
                    return _
                make_callback(i, slider)
                
    def _setup_planning_controls(self):
        """Setup planning controls."""
        with self.server.gui.add_folder("Planning"):
            self.status_text = self.server.gui.add_text("Status", initial_value="Ready", disabled=True)
            
            # Start/Goal buttons
            set_start_btn = self.server.gui.add_button("Set Start Configuration")
            self.start_display = self.server.gui.add_text("Start", initial_value="Not set", disabled=True)
            
            @set_start_btn.on_click
            def _(_):
                self.start_config = self.current_config.copy()
                self.start_display.value = f"[{', '.join([f'{v:.2f}' for v in self.start_config])}]"
                self.status_text.value = "Start configuration set"
            
            set_goal_btn = self.server.gui.add_button("Set Goal Configuration")
            self.goal_display = self.server.gui.add_text("Goal", initial_value="Not set", disabled=True)
            
            @set_goal_btn.on_click
            def _(_):
                self.goal_config = self.current_config.copy()
                self.goal_display.value = f"[{', '.join([f'{v:.2f}' for v in self.goal_config])}]"
                self.status_text.value = "Goal configuration set"
            
            # Planning controls
            self.server.gui.add_markdown("---")
            timeout_slider = self.server.gui.add_slider(
                "Planning Timeout (s)", min=1.0, max=30.0, step=1.0, initial_value=5.0
            )
            
            @timeout_slider.on_update
            def _(_):
                self.planning_timeout = timeout_slider.value
            
            plan_btn = self.server.gui.add_button("Plan Path (RRTConnect)", color="green")
            
            @plan_btn.on_click
            def _(_):
                if self.start_config is None:
                    self.status_text.value = "Error: Set start configuration first!"
                    return
                if self.goal_config is None:
                    self.status_text.value = "Error: Set goal configuration first!"
                    return
                
                if self.on_plan_requested is not None:
                    self.status_text.value = "Planning..."
                    success, path, message = self.on_plan_requested(
                        self.start_config, self.goal_config, self.planning_timeout
                    )
                    self.status_text.value = message
                    
                    if success:
                        self.planned_path = path
                        self.path_index = 0
                        self._show_path(path)
                    else:
                        self.planned_path = None
                else:
                    self.status_text.value = "Error: No planner callback set!"
            
            # Animation controls
            self.server.gui.add_markdown("---")
            
            animate_btn = self.server.gui.add_button("Animate Path", color="blue")
            stop_btn = self.server.gui.add_button("Stop Animation", color="red")
            reset_btn = self.server.gui.add_button("Reset to Start")
            
            speed_slider = self.server.gui.add_slider(
                "Animation Speed", min=0.1, max=5.0, step=0.1, initial_value=1.0
            )
            
            @speed_slider.on_update
            def _(_):
                self.animation_speed = speed_slider.value
            
            @animate_btn.on_click
            def _(_):
                if self.planned_path is None:
                    self.status_text.value = "No path to animate!"
                    return
                self.is_animating = True
                self.path_index = 0
                self.status_text.value = "Animating..."
            
            @stop_btn.on_click
            def _(_):
                self.is_animating = False
                self.status_text.value = "Animation stopped"
            
            @reset_btn.on_click
            def _(_):
                if self.start_config is not None:
                    self.is_animating = False
                    self.current_config[:] = self.start_config
                    self.update_robot(self.current_config)
                    self._update_sliders()
                    self.status_text.value = "Reset to start"
                    
    def _setup_obstacle_controls(self):
        """Setup obstacle controls."""
        with self.server.gui.add_folder("Obstacles"):
            radius_slider = self.server.gui.add_slider(
                "New Sphere Radius", min=0.02, max=0.2, step=0.01, initial_value=0.05
            )
            
            add_btn = self.server.gui.add_button("Add Sphere Obstacle", color="orange")
            
            @add_btn.on_click
            def _(_):
                obs_id = self.sphere_counter
                self.sphere_counter += 1
                
                radius = radius_slider.value
                pos = (0.4 + 0.15 * (obs_id % 3), 0.3 - 0.15 * (obs_id // 3), 0.5)
                
                self._add_obstacle_visual(obs_id, pos, radius)
                self.status_text.value = f"Added sphere obstacle {obs_id}"
            
            clear_btn = self.server.gui.add_button("Clear All Obstacles", color="red")
            
            @clear_btn.on_click
            def _(_):
                self._remove_all_obstacle_visuals()
                self._clear_path()
                self.sphere_counter = 0
                self.status_text.value = "Cleared all obstacles"
                
    def _setup_utilities(self):
        """Setup utility controls."""
        with self.server.gui.add_folder("Utilities"):
            check_btn = self.server.gui.add_button("Check Current Collision")
            collision_status = self.server.gui.add_text("Collision", initial_value="Unknown", disabled=True)
            
            @check_btn.on_click
            def _(_):
                if self.on_collision_check_requested is not None:
                    is_valid = self.on_collision_check_requested(self.current_config)
                    if is_valid:
                        collision_status.value = "Free"
                        self.status_text.value = "Current configuration is collision-free"
                    else:
                        collision_status.value = "Collision!"
                        self.status_text.value = "Current configuration is in collision!"
                else:
                    collision_status.value = "No checker"
                    self.status_text.value = "Error: No collision checker callback set!"
    
    # ========================================================================
    # Visualization Methods
    # ========================================================================
    
    def update_robot(self, config: np.ndarray):
        """Update robot configuration visualization."""
        joint_cfg = {name: config[i] for i, name in enumerate(self.joint_names)}
        self.urdf_vis.update_cfg(joint_cfg)
        
    def _update_sliders(self):
        """Update slider values to match current config."""
        for i, name in enumerate(self.joint_names):
            self.joint_sliders[name].value = self.current_config[i]
            
    def _add_obstacle_visual(self, obs_id: int, position: tuple, radius: float):
        """Add obstacle visualization with transform controls."""
        handle = self.server.scene.add_transform_controls(
            f"/obstacle_{obs_id}", scale=0.15, position=position
        )
        
        sphere = trimesh.creation.icosphere(subdivisions=2, radius=radius)
        sphere.visual.vertex_colors = np.array([[255, 80, 80, 200]] * len(sphere.vertices))
        mesh_handle = self.server.scene.add_mesh_trimesh(f"/obstacle_{obs_id}/mesh", mesh=sphere)
        
        self.obstacle_handles[obs_id] = {
            'handle': handle, 
            'mesh_handle': mesh_handle,
            'radius': radius
        }
        
    def _remove_all_obstacle_visuals(self):
        """Remove all obstacle visualizations."""
        for obs_id, handles in self.obstacle_handles.items():
            try:
                handles['handle'].remove()
            except Exception:
                pass
            try:
                handles['mesh_handle'].remove()
            except Exception:
                pass
        self.obstacle_handles.clear()
        
    def _show_path(self, waypoints: List[np.ndarray]):
        """Visualize path as end-effector trajectory."""
        if self.path_handle is not None:
            try:
                self.path_handle.remove()
            except Exception:
                pass
            
        if len(waypoints) < 2:
            return
        
        # Get end-effector positions via callback
        if self.on_path_visualization_requested is not None:
            ee_positions = self.on_path_visualization_requested(waypoints)
            if ee_positions is not None and len(ee_positions) >= 2:
                ee_positions = np.array(ee_positions, dtype=np.float32)
                line_segments = np.stack([ee_positions[:-1], ee_positions[1:]], axis=1)
                
                self.path_handle = self.server.scene.add_line_segments(
                    "/path", points=line_segments, colors=(0, 255, 0), line_width=3.0
                )
        
    def _clear_path(self):
        """Clear path visualization."""
        if self.path_handle is not None:
            try:
                self.path_handle.remove()
                self.path_handle = None
            except Exception:
                pass
                
    # ========================================================================
    # Public Methods
    # ========================================================================
    
    def update(self):
        """Update animation state. Call this in your main loop."""
        if self.is_animating and self.planned_path is not None:
            if self.path_index < len(self.planned_path):
                self.current_config[:] = self.planned_path[self.path_index]
                self.update_robot(self.current_config)
                self._update_sliders()
                self.path_index = int(self.path_index + self.animation_speed)
            else:
                self.is_animating = False
                self.status_text.value = "Animation complete"
                
    def get_obstacles(self) -> dict:
        """
        Get current obstacle data from transform controls.
        
        Returns:
            Dict mapping obs_id to {'position': np.ndarray, 'radius': float}
        """
        return {
            obs_id: {
                'position': np.array(handles['handle'].position),
                'radius': handles['radius']
            }
            for obs_id, handles in self.obstacle_handles.items()
        }
    
    def get_start_config(self) -> Optional[np.ndarray]:
        """
        Get the start configuration.
        
        Returns:
            Start configuration as numpy array, or None if not set.
        """
        return self.start_config.copy() if self.start_config is not None else None
    
    def get_goal_config(self) -> Optional[np.ndarray]:
        """
        Get the goal configuration.
        
        Returns:
            Goal configuration as numpy array, or None if not set.
        """
        return self.goal_config.copy() if self.goal_config is not None else None
    
    def get_current_config(self) -> np.ndarray:
        """
        Get the current robot configuration.
        
        Returns:
            Current configuration as numpy array.
        """
        return self.current_config.copy()
