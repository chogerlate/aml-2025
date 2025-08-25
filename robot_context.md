Context:
The robot has only 8 infrared sensors (IR-0, IR-1, IR-2, …, IR-7) locating around
the robot. Each sensor reads the distance to the nearest object in its direction.
The possible value is 0 to 100 pixels. It means the sensors are only checked for
the 100 pixels ahead in its direction.
• The sensory information can be gathered by calling self.distance(). For
example IR = self.distance(). Then, the updated sensory information
are in a array variable of IR[8]. The index is used relating to sensor's
position on the robot. For example, we use IR[0] keeps the information
of sensor IR-0.
• The direction of the robot is referred to its head position which is referred to
0 degree. The other directions are then referred in clockwise direction.
• In addition, the robot can smell for direction of food. By calling
self.smell(), the robot can get the direction of food referred to the
heading position of the robot.
• For the actions, there are 3 commands available to use. There are
self.move(distance), self.turn(angle). Please use only these
commands.
o Note :
▪ positive angle is clockwise
▪ default distance = 1
▪ distance should not more than 10
• The robot program should be written in a file named run.py. You can
do or modify whatever you want to program it moving.
• The environment is shown below. The robot is a circle with a small circle
on it. The small circle indicates the heading position of the robot. There
are 26 obstacle boxes on the field. The robot cannot move pass it. If the
robot is forced to move into the obstacle it will force to return the
previous position. The green square on the right is the food and it can
be smelled by the robot. The robot will get a direction (reference to the
direction of its heading) to the food when it tries to smell.
