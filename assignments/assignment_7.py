#!/usr/bin/python3

from pysimbotlib.core import PySimbotApp, Simbot, Robot, Util
from kivy.logger import Logger
from kivy.config import Config

# # Force the program to show user's log only for "info" level or more. The info log will be disabled.
# Config.set('kivy', 'log_level', 'debug')
Config.set('graphics', 'maxfps', 10)

import pandas as pd
import numpy as np
import random

from typing import Tuple, Dict

def dist_to_label(dist: float) -> str:
    if dist < 10:
        return 'C'
    elif 10 <= dist < 25:
        return 'N'
    elif 25 <= dist < 50:
        return 'M'
    elif 50 <= dist < 75:
        return 'F'
    else:
        return 'L'

def angle_to_label(angle: float) -> str:
    # Normalize angle to [-90, 90] range
    angle = max(-90, min(90, angle))
    if -10 <= angle <= 10:
        return 'C'
    elif 10 < angle < 45:
        return 'R'
    elif 45 <= angle < 90:
        return 'Z'
    elif 90 <= angle <= 180 or -180 <= angle <= -90:
        return 'B'
    elif -90 < angle <= -45:
        return 'A'
    elif -45 < angle < -10:
        return 'L'

def learning_data_processing(data: pd.DataFrame) -> None:
    # Convert columns to object dtype first, then apply label conversion
    # Assign directly to new columns or use copy to avoid setting on incompatible dtype inplace
    for col_idx in range(8):
        col_name = data.columns[col_idx]
        data[col_name] = data[col_name].astype(object).map(dist_to_label)
    
    col_name_angle = data.columns[8]
    data[col_name_angle] = data[col_name_angle].astype(object).map(angle_to_label)

def inference_data_processing(data: pd.DataFrame) -> None:
    # Convert columns to object dtype first, then apply label conversion
    for col_idx in range(8):
        col_name = data.columns[col_idx]
        data[col_name] = data[col_name].astype(object).map(dist_to_label)
    
    col_name_angle = data.columns[8]
    data[col_name_angle] = data[col_name_angle].astype(object).map(angle_to_label)

def turnmove_to_class(row) -> str:
    turn = row['turn']
    move = row['move']
    
    # Determine turn component
    turn_active = turn <= -5 or turn >= 5
    turn_left = turn <= -5
    turn_right = turn >= 5
    
    # Determine move component
    move_forward = move >= 3
    move_backward = move <= -0.5
    
    # Handle combined cases first (both turn and move exceed thresholds)
    if turn_right and move_forward:
        return 'RF'  # Right + Forward
    elif turn_left and move_forward:
        return 'LF'  # Left + Forward
    elif turn_right and move_backward:
        return 'RB'  # Right + Backward
    elif turn_left and move_backward:
        return 'LB'  # Left + Backward
    # Handle single turn cases
    elif turn_right:
        return 'R'  # Right only
    elif turn_left:
        return 'L'  # Left only
    # Handle single move cases
    elif move_forward:
        return 'F'  # Forward only
    elif move_backward:
        return 'B'  # Backward only
    # Default: Stop/No action
    else:
        return 'S'  # Stop

class NaiveBayes:
    # Learning phase to build the CPT
    def __init__(self, filename: str, handle_imbalance: bool = True):
        # Load preprocessed data (should be processed using process_training_data.py)
        # The data should already have:
        # - Sensor values converted to labels (ir0-ir7, angle)
        # - Action column created
        # - All filtering already applied
        data = pd.read_csv(filename, sep=',')
        
        # Verify that data has been processed (check if action column exists and sensors are labels)
        if 'action' not in data.columns:
            raise ValueError("Data must be preprocessed. Use process_training_data.py first.")
        
        self.features = ['ir0', 'ir1', 'ir2', 'ir3', 'ir4', 'ir5', 'ir6', 'ir7', 'angle']
        self.actions = data['action'].unique()
        
        # 1. Calculate Prior Probabilities P(Action)
        self.prior = {}
        total_samples = len(data)
        action_counts = data['action'].value_counts()
        
        if handle_imbalance:
            # Use balanced priors: equal weight for each action class
            num_actions = len(self.actions)
            for action in self.actions:
                self.prior[action] = 1.0 / num_actions
        else:
            # Use empirical priors: based on actual data distribution
            for action in self.actions:
                self.prior[action] = action_counts[action] / total_samples
            
        # 2. Calculate Conditional Probabilities P(Feature | Action) with Laplace Smoothing
        self.cond_prob = {}
        
        for action in self.actions:
            self.cond_prob[action] = {}
            action_data = data[data['action'] == action]
            
            for feat in self.features:
                self.cond_prob[action][feat] = {}
                
                # Get counts of each value for this feature within this action class
                value_counts = action_data[feat].value_counts()
                
                # Get all possible values for this feature across the entire dataset (for Laplace smoothing)
                all_values = data[feat].unique()
                num_values = len(all_values)
                
                total_action_samples = len(action_data)
                alpha = 1.0 # Laplace smoothing parameter
                
                for val in all_values:
                    count = value_counts.get(val, 0)
                    # P(feat=val | action) = (count + alpha) / (total_samples_in_class + alpha * num_possible_values)
                    prob = (count + alpha) / (total_action_samples + alpha * num_values)
                    self.cond_prob[action][feat][val] = prob
        
    # find action that gives the highest conditional probability
    def classify(self, input_data: pd.DataFrame) -> str:
        # Calculate MAP (Maximum A Posteriori) estimate using log probabilities
        # log(P(Action|Features)) proportional to log(P(Action)) + sum(log(P(Feature_i|Action)))
        
        best_action = None
        max_log_prob = float('-inf')
        
        # Get the single row of features
        row = input_data.iloc[0]
        
        for action in self.actions:
            # Start with log prior
            current_log_prob = np.log(self.prior[action])
            
            # Add log likelihoods for each feature
            for feat in self.features:
                val = row[feat]
                
                if val in self.cond_prob[action][feat]:
                    prob = self.cond_prob[action][feat][val]
                    current_log_prob += np.log(prob)
                else:
                    # Handle unseen value case (should not happen if training data has all possible labels)
                    # Assign a penalty
                    current_log_prob += np.log(1e-6) 
            
            if current_log_prob > max_log_prob:
                max_log_prob = current_log_prob
                best_action = action
                
        return best_action


class NBRobot(Robot):

    def __init__(self, **kwarg):
        super(NBRobot, self).__init__(**kwarg)
        # Learning Phase
        self.nb = NaiveBayes('C:/Users/siwar/github/PyNBSimbot/history_cleaned.csv')

    def update(self):
        # read sensor value
        ir_values = self.distance()  # [0, 100]
        angle = self.smell()         # [-180, 180]
        
        # create 2D array of size 1 x 9
        sensor = np.zeros((1, 9))
        
        # set the values of the input sensor
        for i, ir in enumerate(ir_values):
            sensor[0][i] = ir

        ## use this line if the training data is from Jet
        sensor[0][8] = angle

        input_data = pd.DataFrame(sensor, columns=['ir0', 'ir1', 'ir2', 'ir3', 'ir4', 'ir5', 'ir6', 'ir7', 'angle'])
        inference_data_processing(input_data)

        # inference
        # Test Phase
        action = self.nb.classify(input_data)

        # Parse combined actions (RF, LF, RB, LB) and single actions (R, L, F, B, S)
        if action == 'RF':  # Right + Forward
            answerTurn = 10
            answerMove = 5
        elif action == 'LF':  # Left + Forward
            answerTurn = -15
            answerMove = 5
        elif action == 'RB':  # Right + Backward
            answerTurn = 10
            answerMove = -5
        elif action == 'LB':  # Left + Backward
            answerTurn = -15
            answerMove = -5
        elif action == 'R':  # Right only
            answerTurn = 10
            answerMove = 5
        elif action == 'L':  # Left only
            answerTurn = -15
            answerMove = 5  
        elif action == 'F':  # Forward only
            answerTurn = 0
            answerMove = 5
        elif action == 'B':  # Backward only
            answerTurn = 0
            answerMove = -5
        else:  # Stop (S) or unknown
            answerTurn = 0
            answerMove = 0

        # Log input data and action
        print(f"Step - Input data: {input_data.iloc[0].tolist()}, Action: {action}, Turn: {answerTurn}, Move: {answerMove}")

        # perform the robot movement
        self.turn(int(answerTurn))
        self.move(answerMove)

        if self.stuck or (answerTurn == 0 and answerMove == 0):
            Deg = random.randint(-45, 45)
            self.turn(Deg)
            self.move(-5)

if __name__ == '__main__':
    app = PySimbotApp(robot_cls=NBRobot, 
                        num_robots=1,
                        max_tick=5000,
                        theme='default',
                        interval = 1.0/60.0,
                        simulation_forever=False)
    app.run()