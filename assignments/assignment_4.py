#!/usr/bin/python3

from pysimbotlib.core import PySimbotApp, Simbot, Robot, Util
from kivy.logger import Logger
from kivy.config import Config

from copy import deepcopy
import random
import csv
import os

# # Force the program to show user's log only for "info" level or more. The info log will be disabled.
# Config.set('kivy', 'log_level', 'debug')
Config.set('graphics', 'maxfps', 10)
dead_count = 0

def calculate_diversity(robots):
    if len(robots) < 2:
        return 1.0
    
    total_diff = 0
    comparisons = 0
    sample_size = min(10, len(robots))
    sample_robots = random.sample(robots, sample_size)
    
    for i in range(len(sample_robots)):
        for j in range(i + 1, len(sample_robots)):
            diff = 0
            for rule_idx in range(len(sample_robots[i].RULES)):
                for val_idx in range(len(sample_robots[i].RULES[rule_idx])):
                    diff += abs(sample_robots[i].RULES[rule_idx][val_idx] - 
                               sample_robots[j].RULES[rule_idx][val_idx])
            total_diff += diff
            comparisons += 1
    
    if comparisons == 0:
        return 1.0
    
    avg_diff = total_diff / comparisons
    max_possible_diff = 256 * len(sample_robots[0].RULES) * len(sample_robots[0].RULES[0])
    diversity = min(1.0, avg_diff / max_possible_diff)
    return diversity

def adaptive_mutation_rate(diversity, base_rate=0.02, min_rate=0.005, max_rate=0.1):
    # higher mutation when population is less diverse (converging)
    # lower mutation when population is diverse
    if diversity < 0.1:
        return min(max_rate, base_rate * 3)
    elif diversity < 0.3:
        return base_rate * 2
    elif diversity > 0.7:
        return max(min_rate, base_rate * 0.5)
    else:
        return base_rate

class StupidRobot(Robot):

    RULE_LENGTH = 11
    NUM_RULES = 10
    ELITE_COUNT = 2
    TOURNAMENT_SIZE = 3
    USE_TOURNAMENT_SELECTION = True

    def __init__(self, **kwarg):
        super(StupidRobot, self).__init__(**kwarg)
        self.RULES = [[0] * self.RULE_LENGTH for _ in range(self.NUM_RULES)]

        # initial list of rules
        self.rules = [0.] * self.NUM_RULES
        self.turns = [0.] * self.NUM_RULES
        self.moves = [0.] * self.NUM_RULES

        self.fitness = 0
        self.lazy_count = 0
        self.headache_count = 0
        self.energy = 400
        self.just_eat = False
        self.start = 0
        self.stop = 0
        

    def update(self):
        ''' Update method which will be called each frame
        '''   
        global dead_count
        global last_lazy
        global last_headache
        if self._sm.iteration % 1000 == 0 and self._sm.iteration > 0:
            if self is self._sm.robots[0]:
                self._sm.robots.sort(key=lambda r: r.fitness, reverse=True)
                Logger.info(f"Saving best robot at iteration: {self._sm.iteration}")
                Logger.info(f"Dead: {dead_count}")
                write_rule(self._sm.robots[0], "best_robot.csv", iteration=self._sm.iteration, dead=dead_count)
                dead_count = 0


        # Logger.info(self._sm.iteration)
        self.ir_values = self.distance()
        self.S0, self.S1, self.S2, self.S3, self.S4, self.S5, self.S6, self.S7 = self.ir_values
        self.target = self.smell_nearest()
        if self.energy < 100 :
            self.set_color(255,255,0,1)
        elif self.energy < 300 :
            self.set_color(255,0,0,1)
        elif self.energy < 500 :
            self.set_color(0,255,0,1)
        elif self.energy < 700 :
            self.set_color(0,0,255,1)
        elif self.energy < 900 :
            self.set_color(255,0,255,1)
        elif self.energy < 1200 :
            self.set_color(0,0,0,1)

        for i, RULE in enumerate(self.RULES):
            self.rules[i] = 1.0
            for k, RULE_VALUE in enumerate(RULE):
                if k < 8:
                    if RULE_VALUE % 5 == 1:
                        if k == 0: self.rules[i] *= self.S0_near()
                        elif k == 1: self.rules[i] *= self.S1_near()
                        elif k == 2: self.rules[i] *= self.S2_near()
                        elif k == 3: self.rules[i] *= self.S3_near()
                        elif k == 4: self.rules[i] *= self.S4_near()
                        elif k == 5: self.rules[i] *= self.S5_near()
                        elif k == 6: self.rules[i] *= self.S6_near()
                        elif k == 7: self.rules[i] *= self.S7_near()
                    elif RULE_VALUE % 5 == 2:
                        if k == 0: self.rules[i] *= self.S0_far()
                        elif k == 1: self.rules[i] *= self.S1_far()
                        elif k == 2: self.rules[i] *= self.S2_far()
                        elif k == 3: self.rules[i] *= self.S3_far()
                        elif k == 4: self.rules[i] *= self.S4_far()
                        elif k == 5: self.rules[i] *= self.S5_far()
                        elif k == 6: self.rules[i] *= self.S6_far()
                        elif k == 7: self.rules[i] *= self.S7_far()
                elif k == 8:
                    temp_val = RULE_VALUE % 6
                    if temp_val == 1: self.rules[i] *= self.smell_left()
                    elif temp_val == 2: self.rules[i] *= self.smell_center()
                    elif temp_val == 3: self.rules[i] *= self.smell_right()
                elif k==9: self.turns[i] = (RULE_VALUE % 181) - 90
                elif k==10: self.moves[i] = (RULE_VALUE % 21) - 10
        
        answerTurn = 0.0
        answerMove = 0.0

        for turn, move, rule in zip(self.turns, self.moves, self.rules):
            answerTurn += turn * rule
            answerMove += move * rule

        # Logger.info(f"turn{answerTurn} move{answerMove}")
        

        if int(answerMove) == 0 and int(answerTurn) == 0:
            self.lazy_count += 1
            self.energy -= 25

        
        if int(answerMove) < 0 or abs(answerTurn) > 30:
            self.headache_count += 1
            self.energy -= 20

        self.turn(answerTurn)
        self.move(answerMove)

        # every step the robot lost its energy
        self.energy -= 1
            
        # if the robot hit, it also lost energy
        if self.just_hit :
            self.energy -= 5
            self.just_hit = False
            
        
        # if the robot eat food, it gets some energy back
        if self.just_eat :
            self.energy += 500
            self.just_eat = False
            

        

        if self.energy < 0 :
            self.stop = self._sm.iteration
            dead_count += 1
            # Logger.info(f"eat_count: {self.eat_count}")
            # die and find a new robot
            # print (self._sm.iteration)
            
            # record that there is a dead one
            
            # generate new robot by usubg genetic operations
            temp = StupidRobot()
            temp = self.generate_new_robot()
            self.RULES = deepcopy(temp.RULES)

            # give this new born some energy
            self.energy += 300
            # self.lazy_count = 0
            # self.headache_count = 0
            # self.eat_count = 0
            self.start = self._sm.iteration
            
    def generate_new_robot(self):
        simbot = self._sm
        num_robots = len(simbot.robots)

        for robot in simbot.robots:
            robot.fitness += robot.eat_count * 300
            robot.fitness += robot.stop - robot.start  
            robot.fitness -= robot.lazy_count 
            robot.fitness -= robot.headache_count 
            robot.fitness -= robot.collision_count 

        simbot.robots.sort(key=lambda robot: robot.fitness, reverse=True)
        # Logger.info(f"Best Fitness:{simbot.robots[0].fitness}")
        def select(simbot) -> StupidRobot:
            if self.USE_TOURNAMENT_SELECTION:
                tournament = random.sample(simbot.robots, min(self.TOURNAMENT_SIZE, len(simbot.robots)))
                return max(tournament, key=lambda r: r.fitness)
            else:
                total_fitness = sum(r.fitness for r in simbot.robots)
                if total_fitness <= 0:
                    return random.choice(simbot.robots)
                pick = random.uniform(0, total_fitness)
                current = 0
                for r in simbot.robots:
                    current += r.fitness
                    if current >= pick:
                        return r

        # elitism: sometimes return elite robot directly
        if random.random() < 0.1 and len(simbot.robots) >= self.ELITE_COUNT:
            elite_index = random.randint(0, min(self.ELITE_COUNT - 1, len(simbot.robots) - 1))
            elite_robot = simbot.robots[elite_index]
            child = StupidRobot()
            child.RULES = deepcopy(elite_robot.RULES)
            mutate(child, mutation_rate=0.01)
            return child

        select1 = select(simbot)
        select2 = select(simbot)
        tryloop = 0
        while select1 == select2 and tryloop < 5:
            select2 = select(simbot)
            tryloop += 1

        # crossover + mutation
        diversity = calculate_diversity(simbot.robots)
        mutation_rate = adaptive_mutation_rate(diversity)
        child = crossover(select1, select2)
        mutate(child, mutation_rate=mutation_rate)
        return child

     

    def S0_near(self):
        if self.S0 <= 0: return 1.0
        elif self.S0 >= 100: return 0.0
        else: return 1 - (self.S0 / 100.0)

    def S0_far(self):
        if self.S0 <= 0: return 0.0
        elif self.S0 >= 100: return 1.0
        else: return self.S0 / 100.0
    
    def S1_near(self):
        if self.S1 <= 0: return 1.0
        elif self.S1 >= 100: return 0.0
        else: return 1 - (self.S1 / 100.0)
    
    def S1_far(self):
        if self.S1 <= 0: return 0.0
        elif self.S1 >= 100: return 1.0
        else: return self.S1 / 100.0
    
    def S2_near(self):
        if self.S2 <= 0: return 1.0
        elif self.S2 >= 100: return 0.0
        else: return 1 - (self.S2 / 100.0)
    
    def S2_far(self):
        if self.S2 <= 0: return 0.0
        elif self.S2 >= 100: return 1.0
        else: return self.S2 / 100.0
    
    def S3_near(self):
        if self.S3 <= 0: return 1.0
        elif self.S3 >= 100: return 0.0
        else: return 1 - (self.S3 / 100.0)
    
    def S3_far(self):
        if self.S3 <= 0: return 0.0
        elif self.S3 >= 100: return 1.0
        else: return self.S3 / 100.0
    
    def S4_near(self):
        if self.S4 <= 0: return 1.0
        elif self.S4 >= 100: return 0.0
        else: return 1 - (self.S4 / 100.0)
    
    def S4_far(self):
        if self.S4 <= 0: return 0.0
        elif self.S4 >= 100: return 1.0
        else: return self.S4 / 100.0
    
    def S5_near(self):
        if self.S5 <= 0: return 1.0
        elif self.S5 >= 100: return 0.0
        else: return 1 - (self.S5 / 100.0)
    
    def S5_far(self):
        if self.S5 <= 0: return 0.0
        elif self.S5 >= 100: return 1.0
        else: return self.S5 / 100.0
    
    def S6_near(self):
        if self.S6 <= 0: return 1.0
        elif self.S6 >= 100: return 0.0
        else: return 1 - (self.S6 / 100.0)
    
    def S6_far(self):
        if self.S6 <= 0: return 0.0
        elif self.S6 >= 100: return 1.0
        else: return self.S6 / 100.0
    
    def S7_near(self):
        if self.S7 <= 0: return 1.0
        elif self.S7 >= 100: return 0.0
        else: return 1 - (self.S7 / 100.0)
    
    def S7_far(self):
        if self.S7 <= 0: return 0.0
        elif self.S7 >= 100: return 1.0
        else: return self.S7 / 100.0
    
    def smell_right(self):
        if self.target >= 45: return 1.0
        elif self.target <= 0: return 0.0
        else: return self.target / 45.0
    
    def smell_left(self):
        if self.target <= -45: return 1.0
        elif self.target >= 0: return 0.0
        else: return 1-(-1*self.target)/45.0
    
    def smell_center(self):
        if self.target <= 45 and self.target >= 0: return self.target / 45.0
        if self.target <= -45 and self.target <= 0: return 1-(-1*self.target)/45.0
        else: return 0.0

def crossover(parent1: StupidRobot, parent2: StupidRobot) -> StupidRobot:
        child = StupidRobot()
        child.RULES = []
        crossover_point = random.randint(1, len(parent1.RULES) - 1)

        for i in range(crossover_point):
            child.RULES.append(parent1.RULES[i][:])  # copy

        for i in range(crossover_point, len(parent2.RULES)):
            child.RULES.append(parent2.RULES[i][:])  # copy

        return child

def mutate(robot: StupidRobot, mutation_rate=0.01):
    for i in range(len(robot.RULES)):
        for j in range(len(robot.RULES[i])):
            if random.random() < mutation_rate:
                robot.RULES[i][j] = random.randrange(256)

def write_rule(robot, filename, iteration=None, dead=None):
    # Append to CSV instead of overwriting
    file_exists = os.path.exists(filename)

    with open(filename, "a", newline="") as f:
        writer = csv.writer(f)

        # If file is new, write header first
        if not file_exists:
            writer.writerow(["iteration", "dead_count"])

        # Only write stats, not the robot RULES here
        if iteration is not None and dead is not None:
            writer.writerow([iteration, dead])


def before_simulation(simbot: Simbot):
    for robot in simbot.robots:
        # random RULES value for the first generation
        if simbot.simulation_count == 0:
            Logger.info("GA: initial population")
            for i, RULE in enumerate(robot.RULES):
                for k in range(len(RULE)):
                    robot.RULES[i][k] = random.randrange(256)
        
def after_simulation(simbot: Simbot):

    for robot in simbot.robots:
        robot.fitness = robot.energy

    # descending sort and rank: the best 10 will be on the list at index 0 to 9
    simbot.robots.sort(key=lambda robot: robot.fitness, reverse=True)

    write_rule(simbot.robots[0], "best_robot.csv")

if __name__ == '__main__':

    app = PySimbotApp(robot_cls=StupidRobot, 
                        num_robots=20,
                        num_objectives=4,
                        theme='default',
                        simulation_forever=False,
                        max_tick=100000,
                        interval=1/1000.0,
                        food_move_after_eat=True,
                        robot_see_each_other=True,
                        # map="no_wall",
                        customfn_before_simulation=before_simulation, 
                        customfn_after_simulation=after_simulation)
    app.run()