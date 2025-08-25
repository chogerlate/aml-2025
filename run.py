"""
Assignment 1 - Reactive Robot Control
IRP-AML-Assignment#1

Task: Implement reactive control for robot navigation through obstacles to reach food.
Strategy: Environment-aware navigation with strategic smell control.
"""

import os, platform
if platform.system() == "Linux" or platform.system() == "Darwin":
    os.environ["KIVY_VIDEO"] = "ffpyplayer"
    
from pysimbotlib.core import PySimbotApp, Robot
from kivy.logger import Logger
from kivy.config import Config

# Configure logging level
Config.set('kivy', 'log_level', 'info')

class RobotController(Robot):
    """
    Smart Reactive Control with Environment-Aware Navigation
    
    Core Strategy:
    1. Environment Detection: Narrow Path vs Open Area vs Moderate Path
    2. Strategic Smell Control: Enable/disable based on environment safety
    3. Priority-Based Actions: Escape first, then navigate efficiently
    """
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        
        # Navigation thresholds
        self.obstacle_threshold = 12    # Distance to consider as obstacle
        
        # Food seeking parameters
        self.food_seek_counter = 0      # Counter for periodic food seeking
        self.food_seek_interval = 10    # Seek food every 10 steps
        
        # Path width detection
        self.narrow_path_threshold = 10  # Consider path narrow if sides < 20 pixels
        self.open_area_threshold = 40    # Consider area open if sides > 40 pixels
        
    def is_in_narrow_path(self, IR):
        """Check if robot is in a narrow path - disable smell functionality"""
        left_space = min(IR[6], IR[7])   # Left and front-left sensors
        right_space = min(IR[1], IR[2])  # Front-right and right sensors
        return left_space < self.narrow_path_threshold and right_space < self.narrow_path_threshold
    
    def is_in_open_area(self, IR):
        """Check if robot is in an open area - enable full smell functionality"""
        left_space = min(IR[6], IR[7])   # Left and front-left sensors
        right_space = min(IR[1], IR[2])  # Front-right and right sensors
        return left_space > self.open_area_threshold and right_space > self.open_area_threshold
    
    def update(self):
        """Main control loop: environment-aware navigation with strategic smell control"""
        # Get sensor data
        IR = self.distance()
        
        # Log sensor values for debugging
        Logger.info(f"IR Sensors: {[round(x, 1) for x in IR]}")
        
        # Environment detection and strategy selection
        if self.is_in_narrow_path(IR):
            Logger.info("🚨 NARROW PATH - Smell DISABLED, focusing on escape")
            self.escape_narrow_path(IR)
        elif self.is_in_open_area(IR):
            Logger.info("🌤️ OPEN AREA - Smell ENABLED, full food seeking")
            self.seek_food_in_open_area(IR)
        else:
            Logger.info("🟡 MODERATE PATH - Balanced approach")
            self.balanced_navigation(IR)
    
    def escape_narrow_path(self, IR):
        """Priority 1: Escape narrow path - no smell, survival first"""
        front_clear = IR[0] > self.obstacle_threshold and IR[1] > self.obstacle_threshold and IR[7] > self.obstacle_threshold
        
        if front_clear:
            Logger.info("Moving forward to escape narrow path")
            self.move(5)
        else:
            # Find wider side and turn toward it
            left_space = min(IR[6], IR[7])
            right_space = min(IR[1], IR[2])
            
            if right_space > left_space:
                Logger.info(f"Turning RIGHT {20}° toward wider space")
                self.turn(20)
            else:
                Logger.info(f"Turning LEFT {20}° toward wider space")
                self.turn(-20)
    
    def seek_food_in_open_area(self, IR):
        """Priority 2: Full food seeking in safe open areas"""
        food_direction = self.smell()
        Logger.info(f"Food Direction: {round(food_direction, 1)}°")
        
        front_clear = IR[0] > self.obstacle_threshold and IR[1] > self.obstacle_threshold and IR[7] > self.obstacle_threshold
        
        if front_clear:
            self.food_seek_counter += 1
            
            # Periodic aggressive food seeking
            if self.food_seek_counter >= self.food_seek_interval:
                if abs(food_direction) > 10:
                    if food_direction > 0:
                        Logger.info(f"Periodic food seeking: Turning RIGHT {15}°")
                        self.move(-5)  # Back up before turn
                        self.turn(15)
                    else:
                        Logger.info(f"Periodic food seeking: Turning LEFT {15}°")
                        self.move(-5)  # Back up before turn
                        self.turn(-15)
                self.food_seek_counter = 0
            
            # Move forward
            Logger.info("Path clear - MOVING FORWARD")
            self.move(5)
            
            # Continuous course correction
            if abs(food_direction) > 5:
                if food_direction > 0:
                    Logger.info(f"Course correction: RIGHT {5}°")
                    self.turn(5)
                else:
                    Logger.info(f"Course correction: LEFT {5}°")
                    self.turn(-5)
        else:
            self.avoid_obstacle(IR)
    
    def balanced_navigation(self, IR):
        """Priority 3: Conservative navigation in moderate paths"""
        food_direction = self.smell()
        Logger.info(f"Food Direction: {round(food_direction, 1)}°")
        
        front_clear = IR[0] > self.obstacle_threshold and IR[1] > self.obstacle_threshold and IR[7] > self.obstacle_threshold
        
        if front_clear:
            Logger.info("Path clear - MOVING FORWARD")
            self.move(5)
            
            # Conservative food seeking
            if abs(food_direction) > 15:
                if food_direction > 0:
                    self.turn(8)
                else:
                    self.turn(-8)
        else:
            self.avoid_obstacle(IR)
    
    def avoid_obstacle(self, IR):
        """Common obstacle avoidance strategy"""
        left_space = min(IR[6], IR[7])
        right_space = min(IR[1], IR[2])
        
        Logger.info(f"Obstacle ahead! Right: {round(right_space, 1)}, Left: {round(left_space, 1)}")
        
        if right_space > left_space and right_space > self.obstacle_threshold:
            Logger.info(f"Turning RIGHT {20}° - more space on right")
            self.turn(20)
        elif left_space > right_space and left_space > self.obstacle_threshold:
            Logger.info(f"Turning LEFT {20}° - more space on left")
            self.turn(-20)
        else:
            Logger.info("Both sides blocked - BACKING UP and turning")
            self.move(-5)
            self.turn(25)

if __name__ == '__main__':
    app = PySimbotApp(
        robot_cls=RobotController, 
        interval=1/50.0,  # Fast simulation
        num_robots=1, 
        simulation_forever=True
    )
    app.run()



