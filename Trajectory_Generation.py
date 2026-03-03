#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import sympy as sp
import threading
import time
from quadrotor_msgs.msg import PositionCommand
from nav_msgs.msg import Odometry

# ================= State Enumeration =================
STATE_WAITING_ODOM = 0      # Waiting for drone power-on / localization
STATE_WAIT_TAKEOFF = 1      # Odom ready, hovering to wait for takeoff command
STATE_ASCENDING = 2         # Slowly ascending to designated height
STATE_WAIT_FIGURE8 = 3      # Ascend complete, hovering to wait for figure-8 command
STATE_FLYING = 4            # Executing figure-8 trajectory
STATE_FINISHED = 5          # [New] Finished state (stop sending commands)

class SingleDroneController: 
    def __init__(self, drone_name):
        self.drone_name = drone_name
        self.state = STATE_WAITING_ODOM
        
        # ================= Configuration Parameters =================
        self.cmd_topic = f'/{drone_name}/position_cmd'
        self.odom_topic = f'/{drone_name}/mavros/local_position/odom'
        
        # --- Figure-8 Trajectory Parameters ---
        self.A = 1.5      # Horizontal width (m)
        self.B = 1.0      # Forward depth (m)
        self.T = 5.0      # Period to complete one cycle (s)
        self.fixed_yaw = -np.pi / 2 # Heading (fixed)
        
        # [Config] Maximum number of flight cycles
        self.max_cycles = 6.0 
        
        # --- Altitude Control ---
        self.desired_flight_height = 1.1  
        self.ascend_speed = 0.2     
        
        # ================= Internal Variables =================
        self.trajectory_start_time = None 
        self.virtual_center = None 
        self.hover_pos = None     
        self.initial_height = 0.0 
        self.target_height = 0.0  
        self.ramp_z = 0.0         
        self.current_z_meas = 0.0 

        # [New] Track current cycle to prevent log spamming
        self.last_cycle_int = 0
        
        # Command Flags
        self.cmd_start_takeoff = False
        self.cmd_start_figure8 = False
        
        # Initialize symbolic derivation
        self._init_equations()
        
        # ROS Communication Setup
        self.pub = rospy.Publisher(self.cmd_topic, PositionCommand, queue_size=10)
        self.sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)
        
        rospy.loginfo(f"[{self.drone_name}] Controller initialized. Waiting for Odom...")

    def _init_equations(self):
        """ Use SymPy to pre-calculate derivatives for the figure-8 trajectory """
        t, A, B, w = sp.symbols('t A B w')
        sin_wt = sp.sin(w * t)
        cos_wt = sp.cos(w * t)
        denom = 1 + sin_wt**2
        
        # Lemniscate of Bernoulli equations
        px = B * sin_wt * cos_wt / denom
        py = A * cos_wt / denom
        pz = sp.sympify(0) 
        
        # Derivatives: Velocity, Acceleration, Jerk
        vx, vy, vz = sp.diff(px, t), sp.diff(py, t), sp.diff(pz, t)
        ax, ay, az = sp.diff(vx, t), sp.diff(vy, t), sp.diff(vz, t)
        jx, jy, jz = sp.diff(ax, t), sp.diff(ay, t), sp.diff(az, t)
        
        # Compile to numpy functions for performance
        self.f_pos = sp.lambdify((t, A, B, w), [px, py], 'numpy')
        self.f_vel = sp.lambdify((t, A, B, w), [vx, vy], 'numpy')
        self.f_acc = sp.lambdify((t, A, B, w), [ax, ay], 'numpy')
        self.f_jerk = sp.lambdify((t, A, B, w), [jx, jy], 'numpy')
        
        self.omega = 2 * np.pi / self.T
        self.rel_pos_at_t0 = self.f_pos(0, self.A, self.B, self.omega)

    def odom_cb(self, msg):
        self.current_z_meas = msg.pose.pose.position.z

        if self.state == STATE_WAITING_ODOM:
            self.hover_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
            self.initial_height = msg.pose.pose.position.z 
            
            # [Absolute Altitude Logic]
            if self.initial_height < self.desired_flight_height:
                self.target_height = self.desired_flight_height
            else:
                self.target_height = self.initial_height
            
            center_x = self.hover_pos[0] - self.rel_pos_at_t0[0]
            center_y = self.hover_pos[1] - self.rel_pos_at_t0[1]
            self.virtual_center = np.array([center_x, center_y])
            
            rospy.loginfo(f"[{self.drone_name}] Ready! CurrentH={self.initial_height:.2f}m. TargetH={self.target_height:.2f}m.")
            self.state = STATE_WAIT_TAKEOFF

    def update(self, event):
        """ Main control loop, called by Timer (100Hz) """
        now = rospy.Time.now()
        dt = 0.01 

        # --- 0. Wait for Connection ---
        if self.state == STATE_WAITING_ODOM:
            return

        # --- 1. Wait for Takeoff Command (Hovering in place) ---
        elif self.state == STATE_WAIT_TAKEOFF:
            self.publish_setpoint(self.hover_pos[0], self.hover_pos[1], self.initial_height,
                                  0,0,0, 0,0,0)
            
            if self.cmd_start_takeoff:
                if self.initial_height < self.target_height - 0.05:
                    rospy.loginfo(f"[{self.drone_name}] Ascending to {self.target_height:.2f}m...")
                    self.ramp_z = self.initial_height
                    self.state = STATE_ASCENDING
                else:
                    rospy.loginfo(f"[{self.drone_name}] Already at target height. Holding.")
                    self.ramp_z = self.target_height
                    self.state = STATE_WAIT_FIGURE8

        # --- 2. Slow Ascent ---
        elif self.state == STATE_ASCENDING:
            if self.ramp_z < self.target_height:
                self.ramp_z += self.ascend_speed * dt
            if self.ramp_z > self.target_height:
                self.ramp_z = self.target_height
            
            self.publish_setpoint(self.hover_pos[0], self.hover_pos[1], self.ramp_z,
                                  0, 0, self.ascend_speed, 
                                  0, 0, 0)
            
            if abs(self.ramp_z - self.target_height) < 0.001 and \
               abs(self.current_z_meas - self.target_height) < 0.15:
                rospy.loginfo(f"[{self.drone_name}] Altitude Reached. Waiting for FIGURE-8 cmd...")
                self.state = STATE_WAIT_FIGURE8

        # --- 3. Wait for Figure-8 Command ---
        elif self.state == STATE_WAIT_FIGURE8:
            self.publish_setpoint(self.hover_pos[0], self.hover_pos[1], self.target_height,
                                  0,0,0, 0,0,0)
            
            if self.cmd_start_figure8:
                rospy.loginfo(f"[{self.drone_name}] GO! Figure-8 Started ({self.max_cycles} cycles).")
                self.trajectory_start_time = now
                self.state = STATE_FLYING
                self.last_cycle_int = 0 # Reset cycle counter

        # --- 4. Figure-8 Flight (With cycle counting and termination) ---
        elif self.state == STATE_FLYING:
            t_elapsed = (now - self.trajectory_start_time).to_sec()
            
            # Calculate current progress (float)
            cycles_flown = t_elapsed / self.T
            
            # [New] Log cycle progress when starting a new lap
            current_cycle_int = int(cycles_flown) + 1
            if current_cycle_int > self.last_cycle_int and current_cycle_int <= self.max_cycles:
                rospy.loginfo(f"[{self.drone_name}] Starting Cycle {current_cycle_int}/{int(self.max_cycles)} ...")
                self.last_cycle_int = current_cycle_int

            # [New] Termination logic
            if cycles_flown >= self.max_cycles:
                rospy.logwarn(f"[{self.drone_name}] MISSION COMPLETE ({self.max_cycles} cycles finished). Stopping commands!")
                self.state = STATE_FINISHED
                return # Exit early without publishing this setpoint

            t_cycle = t_elapsed % self.T
            
            rel_pos = self.f_pos(t_cycle, self.A, self.B, self.omega)
            rel_vel = self.f_vel(t_cycle, self.A, self.B, self.omega)
            rel_acc = self.f_acc(t_cycle, self.A, self.B, self.omega)
            rel_jerk = self.f_jerk(t_cycle, self.A, self.B, self.omega)
            
            abs_x = self.virtual_center[0] + rel_pos[0]
            abs_y = self.virtual_center[1] + rel_pos[1]
            
            self.publish_setpoint(abs_x, abs_y, self.target_height,
                                  rel_vel[0], rel_vel[1], 0.0,
                                  rel_acc[0], rel_acc[1], 0.0,
                                  rel_jerk[0], rel_jerk[1], 0.0)

        # --- 5. Finished State (Silence) ---
        elif self.state == STATE_FINISHED:
            # [Key] Return immediately to stop publishing setpoints
            return

    def publish_setpoint(self, x, y, z, vx, vy, vz, ax, ay, az, jx=0, jy=0, jz=0):
        cmd = PositionCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "world"
        cmd.trajectory_id = 1
        cmd.position.x = x
        cmd.position.y = y
        cmd.position.z = z
        cmd.velocity.x, cmd.velocity.y, cmd.velocity.z = vx, vy, vz
        cmd.acceleration.x, cmd.acceleration.y, cmd.acceleration.z = ax, ay, az
        cmd.jerk.x, cmd.jerk.y, cmd.jerk.z = jx, jy, jz
        cmd.yaw = self.fixed_yaw
        cmd.yaw_dot = 0.0
        self.pub.publish(cmd)

class DualDroneManager:
    def __init__(self):
        rospy.init_node('dual_figure8_commander', anonymous=True)
        self.freq = 100.0
        self.controllers = [
            SingleDroneController('drone1'),
            SingleDroneController('drone2')
        ]
        
        rospy.loginfo("Dual Drone Manager Initialized.")
        
        # Start control loop (100Hz)
        rospy.Timer(rospy.Duration(1.0/self.freq), self.global_timer_callback)
        
        # Start keyboard listener thread
        self.input_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.input_thread.start()

    def global_timer_callback(self, event):
        """ Dispatch update calls to all drone controllers """
        for controller in self.controllers:
            controller.update(event)

    def keyboard_listener(self):
        """ Keyboard input handling logic """
        import time
        time.sleep(1) 
        
        print("\n" + "="*50)
        print(" [STEP 1] TAKEOFF CONTROL")
        print(f" Press [ENTER] to reach 1.0m (Absolute Height)")
        print("="*50 + "\n")
        
        input() 
        rospy.loginfo(">>> GLOBAL COMMAND: TAKEOFF <<<")
        for c in self.controllers:
            c.cmd_start_takeoff = True
            
        print("\n" + "="*50)
        print(" [STEP 2] TRAJECTORY CONTROL")
        print(" Press [ENTER] to start Figure-8 trajectory")
        print(" (Will stop sending commands after 6 cycles)")
        print("="*50 + "\n")
        
        input() 
        rospy.loginfo(">>> GLOBAL COMMAND: START FIGURE-8 <<<")
        for c in self.controllers:
            c.cmd_start_figure8 = True
            
        print("Mission Running... Press Ctrl+C to exit manually.")
        while not rospy.is_shutdown():
            time.sleep(1)

if __name__ == '__main__':
    try:
        DualDroneManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass