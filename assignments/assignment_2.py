import os
import platform
from pysimbotlib.core import PySimbotApp, Robot
from kivy.logger import Logger
from kivy.config import Config

if platform.system() == "Linux" or platform.system() == "Darwin":
    os.environ["KIVY_VIDEO"] = "ffpyplayer"
Config.set('kivy', 'log_level', 'info')

ROBOT_NUM = 1
TIME_INTERVAL = 1 / 50

class FuzzyRobot(Robot):
    def __init__(self):
        super(FuzzyRobot, self).__init__()
        self.pos = [0, 0]

    def update(self):
        # Get sensor readings: IR distances and target smell direction
        self.ir_values = self.distance() 
        self.target = self.smell()
        
        # Initialize arrays for fuzzy logic rule evaluation
        rules = []      # Rule activation strengths (0.0 to 1.0)
        turns = []      # Turn angles for each rule
        moves = []      # Movement speeds for each rule

        # FUZZY LOGIC ALGORITHM RULES:
        # Each rule has a condition that returns a value between 0.0 and 1.0
        # Higher values mean the rule is more strongly activated
        # Final output is weighted average of all active rules

        # Fundamental rules
        rules.append(self.front_far())
        turns.append(0)
        moves.append(5)

        rules.append(self.right_near())
        turns.append(-45)
        moves.append(0)

        rules.append(self.left_near())
        turns.append(45)
        moves.append(0)

        rules.append(self.smell_left())
        turns.append(-30)
        moves.append(0)

        rules.append(self.smell_right())
        turns.append(30)
        moves.append(0)

        # Target pursuit rules
        rules.append(self.smell_center() * self.left_far() * self.right_far())
        turns.append(0)
        moves.append(5)

        rules.append(self.back_near() * self.smell_center())
        turns.append(0)
        moves.append(2)

        # Advanced rules: Avoidance rules
        # Corridor navigation
        rules.append(self.left_near() * self.right_near() * self.front_far())
        turns.append(0)
        moves.append(5)

        # Front-right obstacle
        rules.append(self.front_near() * self.right_near() * self.left_far())
        turns.append(-60)
        moves.append(0)

        # Front-left obstacle
        rules.append(self.front_near() * self.left_near() * self.right_far())
        turns.append(60)
        moves.append(0)

        # Three-sided trap
        rules.append(self.front_near() * self.left_near() * self.right_near() * self.back_far())
        turns.append(180)
        moves.append(1)

        # Three-sided trap
        rules.append(self.front_near() * self.left_near() * self.right_near() * self.back_near())
        turns.append(120)
        moves.append(0)

        # Three-sided trap
        rules.append(self.front_near() * self.left_far() * self.right_far() * self.back_far())
        turns.append(-120)
        moves.append(-1)

        # FUZZY LOGIC OUTPUT CALCULATION:
        # Weighted average of all rule outputs based on their activation strengths
        # Rules with higher activation (closer to 1.0) have more influence on final decision
        ans_turn = sum(t * r for t, r in zip(turns, rules))
        ans_move = sum(m * r for m, r in zip(moves, rules))
    
        print(self.ir_values)
        print(f"Turn: {ans_turn}, Move: {ans_move}")
        
        # Execute the calculated movement
        self.turn(ans_turn)
        self.move(ans_move)

    def front_far(self):
        irfront = self.ir_values[0]
        if irfront <= 15:
            return 0.0
        elif irfront >= 30:
            return 1.0
        else:
            return (irfront - 10.0) / 30.0

    def front_near(self):
        return 1 - self.front_far()

    def left_far(self):
        irleft = min(self.ir_values[6], self.ir_values[7])
        if irleft <= 8:
            return 0.0
        elif irleft >= 30:
            return 1.0
        else:
            return (irleft - 10.0) / 15.0

    def left_near(self):
        return 1 - self.left_far()

    def right_far(self):
        irright = min(self.ir_values[1], self.ir_values[2])
        if irright <= 8:
            return 0.0
        elif irright >= 30:
            return 1.0
        else:
            return (irright - 10.0) / 15.0

    def right_near(self):
        return 1 - self.right_far()

    def back_far(self):
        irback = min(self.ir_values[3], self.ir_values[4], self.ir_values[5])
        if irback <= 15:
            return 0.0
        elif irback >= 30:
            return 1.0
        else:
            return (irback - 10.0) / 15.0

    def back_near(self):
        return 1 - self.back_far()

    def smell_right(self):
        target = self.smell()
        if target >= 90:
            return 1.0
        elif target <= 0:
            return 0.0
        else:
            return target / 90.0

    def smell_center(self):
        target = abs(self.smell())
        if target >= 45:
            return 1.0
        elif target <= 0:
            return 0.0
        else:
            return target / 45.0

    def smell_left(self):
        target = self.smell()
        if target <= -90:
            return 1.0
        elif target >= 0:
            return 0.0
        else:
            return -target / 90.0

if __name__ == '__main__':
    app = PySimbotApp(FuzzyRobot, ROBOT_NUM, interval=TIME_INTERVAL, enable_wasd_control=False)
    app.run()
