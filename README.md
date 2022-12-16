# Robot Autonomy (CS237A) Final Project

'''
roslaunch asl_turtlebot project.launch
rosrun asl_turtlebot supervisor.py
'''

**Gazebo Simulation Files:**
----------------------

`world/project_city.world`: Defines 3D model of rough, tentative
representation of the final project environment.

**Turtlebot Files:**
----------------------
**Launch Files:**

`launch/root.launch`: The main configurable launch file on top of
which the remaining launch files are built. Can be launched with a simulator or
on top of the running hardware turtlebot.

`launch/project/project.launch`: Launches gazebo with a (rough, tentative)
model of the final project environment, as well as the core SLAM and detector
nodes.

**Scripts/Nodes:**

`scripts/goal_commander.py`: Translates Rviz nav goal clicks
(/move_simple_base/goal) to the /cmd_nav topic.

`scripts/detector.py`: Gazebo stop sign detector from HW2. Publishes to
/detector/* where * is the detected object label.

`scripts/detector_mobilenet.py`: Runs tensorflow mobilenet model for image
classification.

`scripts/detector_viz.py`: Visualizes camera feed, bounding boxes and
confidence for detected objects.

`scripts/utils/grids.py`: Used for motion planning. Performs collision checking on
occupancy grids. grids.py functions/classes are used by scripts/navigator.py.

`scripts/navigator.py`: Node that manages point to point robot navigation, uses
your A\* implementation (HW2) in an MPC framework along with cubic spline
interpolation and the differential flatness controller (from HW1), switching to
the pose controller from HW1 when close to the goal.

`scripts/utils/utils.py`: Utility functions. Currently contains a wrapToPi
function.


**Tensorflow Models:**

The `.pb` files in the `tfmodels` folder are "frozen" neural network models, and
contain both the structure and the weights of pretrained networks.
`ssd_mobilenet_v1_coco.pb` is a pretrained MobileNet v1 model, while
`stop_sign_gazebo.pb` is a model fine-tuend to detect stop signs in Gazebo. 

The `coco_labels.txt` file just contains the mapping from the class number
output by the model to human-interpretable labels.
