#!/usr/bin/python3

from pysimbotlib.core import PySimbotApp, Robot
from kivy.config import Config
# Force the program to show user's log only for "info" level or more. The info log will be disabled.
Config.set('kivy', 'log_level', 'info')

import random
import csv
import os


class RL_Robot(Robot):
    
    # Initialize Q-table: Q[action][FIR][LIR][RIR][LLIR][RRIR][SM]
    # 3 actions × 2 × 2 × 2 × 2 × 2 × 3 states = 96 states per action
    Q = [[[[[[[0.0 for _ in range(3)] for _ in range(2)]
              for _ in range(2)] for _ in range(2)]
              for _ in range(2)] for _ in range(2)] for _ in range(3)]
    
    # Previous state tracking
    prev_st0 = prev_st1 = prev_st2 = prev_st3 = prev_st4 = prev_sm = 0
    prev_action = 0
    
    # Q-learning parameters
    ALPHA = 0.5  # Learning rate
    GAMMA = 0.9  # Discount factor
    EPSILON = 0.1  # Exploration rate
    SENSOR_THRESHOLD = 20  # Distance threshold for binary sensor
    
    # Statistics tracking variables
    time_step = 0
    csv_file = 'robot_stats.csv'
    csv_header_written = False
    write_interval = 10  # Write to CSV every N steps to balance performance

    def update(self):
        # Track time step
        self.time_step += 1
        
        # Calculate cumulative rates over time (overall statistics)
        # Write to CSV periodically to avoid performance issues
        if self.time_step % self.write_interval == 0 or self.time_step == 1:
            eat_rate = self.eat_count / self.time_step  # Cumulative average eat rate
            hit_rate = self.collision_count / self.time_step  # Cumulative average hit rate
            
            # Write to CSV file
            file_exists = os.path.isfile(self.csv_file)
            write_header = not file_exists and not self.csv_header_written
            
            with open(self.csv_file, 'a', newline='') as f:
                writer = csv.writer(f)
                if write_header:
                    writer.writerow(['time_step', 'eat_rate', 'hit_rate'])
                    self.csv_header_written = True
                writer.writerow([self.time_step, eat_rate, hit_rate])
        
        # Read sensors
        distances = self.distance()  # Returns tuple of distances at angles [0, 45, 90, 135, 180, 225, 270, 315]
        target_angle = self.smell()
        
        # Normalize target angle
        if target_angle > 180:
            target_angle = target_angle - 360
        
        # Convert sensor readings to binary states (0=far, 1=near)
        FIR = 1 if distances[0] < self.SENSOR_THRESHOLD else 0  # Front (0°)
        LIR = 1 if distances[2] < self.SENSOR_THRESHOLD else 0  # Left (90°)
        RIR = 1 if distances[6] < self.SENSOR_THRESHOLD else 0  # Right (270°)
        LLIR = 1 if distances[1] < self.SENSOR_THRESHOLD else 0  # Left-left (45°)
        RRIR = 1 if distances[7] < self.SENSOR_THRESHOLD else 0  # Right-right (315°)
        
        # Convert target direction to SM state (0=left, 1=front, 2=right)
        if target_angle == -1:  # No target
            SM = 1  # Default to front
        elif target_angle < -30:
            SM = 0  # Left
        elif target_angle > 30:
            SM = 2  # Right
        else:
            SM = 1  # Front
        
        # Epsilon-greedy action selection
        if random.random() < self.EPSILON:
            # Explore: random action
            action = random.randint(0, 2)
        else:
            # Exploit: choose best action
            q0 = self.Q[0][FIR][LIR][RIR][LLIR][RRIR][SM]
            q1 = self.Q[1][FIR][LIR][RIR][LLIR][RRIR][SM]
            q2 = self.Q[2][FIR][LIR][RIR][LLIR][RRIR][SM]
            
            if q0 >= q1 and q0 >= q2:
                action = 0
            elif q1 >= q2:
                action = 1
            else:
                action = 2
        
        # Execute action and calculate reward
        reward = 0.0
        if action == 0:  # MoveForward
            self.move(5)
            if self.stuck:
                reward = -2.0  # Collision penalty
            else:
                reward = 1.0  # Successful move
        elif action == 1:  # TurnLeft
            self.turn(15)
            reward = -0.1 - abs(target_angle) / 180.0
        elif action == 2:  # TurnRight
            self.turn(-15)
            reward = -0.1 - abs(target_angle) / 180.0
        
        # Update Q-table using Q-learning formula
        # Q(s,a) ← Q(s,a) + α[r + γ·max(Q(s',a')) - Q(s,a)]
        # Current state: (FIR, LIR, RIR, LLIR, RRIR, SM)
        # Next state max Q-value
        max_next_q = max(
            self.Q[0][FIR][LIR][RIR][LLIR][RRIR][SM],
            self.Q[1][FIR][LIR][RIR][LLIR][RRIR][SM],
            self.Q[2][FIR][LIR][RIR][LLIR][RRIR][SM]
        )
        
        # Update Q-value for previous state and action
        old_q = self.Q[self.prev_action][self.prev_st0][self.prev_st1][self.prev_st2][self.prev_st3][self.prev_st4][self.prev_sm]
        new_q = old_q + self.ALPHA * (reward + self.GAMMA * max_next_q - old_q)
        self.Q[self.prev_action][self.prev_st0][self.prev_st1][self.prev_st2][self.prev_st3][self.prev_st4][self.prev_sm] = new_q
        
        # Store current state as previous state for next update
        self.prev_st0, self.prev_st1, self.prev_st2, self.prev_st3, self.prev_st4, self.prev_sm = FIR, LIR, RIR, LLIR, RRIR, SM
        self.prev_action = action

if __name__ == '__main__':
    app = PySimbotApp(robot_cls=RL_Robot, num_robots=1, max_tick=100000)
    app.run()