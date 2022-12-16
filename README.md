# Robot Autonomy (CS237A) Final Project

To run the project simulation in Gazebo and the command center visualization in RVIZ:
```
roslaunch asl_turtlebot project.launch
```

To run the supervisor node (detection network and mission planning):
```
rosrun asl_turtlebot supervisor.py
```


**Added Extensions:**
----------------------
### smach_viewer (State Machine Visualization)

Implemented state machine visualization for navigator and supervisor states:
`scripts/sup_smach.py`
`scripts/nav_smach.py`

<img width="592" alt="Screen Shot 2022-12-15 at 4 32 32 PM" src="https://user-images.githubusercontent.com/90282643/207995085-2945ba56-e6a7-473f-8a01-a8d5644794b3.png">

### Collision Detection
Simple heuristic approach to avoiding dangerous trajectories:
`scripts/navigator.py`

<img width="651" alt="Screen Shot 2022-12-15 at 4 33 39 PM" src="https://user-images.githubusercontent.com/90282643/207995221-52af74f0-dada-4bb6-80d6-1bef674df3d7.png">

### Animal Location Tracking
Visualizing highest confidence animal location estimates in RVIZ:
`scripts/supervisor.py`

<img width="545" alt="Screen Shot 2022-12-15 at 4 35 18 PM" src="https://user-images.githubusercontent.com/90282643/207995374-bb499b77-aa06-43fb-b340-bc7ef58b5ef4.png">

**File Descriptions:**
----------------------
**Gazebo Simulation Files:**

`world/project_city.world`: Defines 3D model of rough, tentative
representation of the final project environment.

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
 A\* implementation in an MPC framework along with cubic spline
interpolation and the differential flatness controller (from HW1), switching to
the pose controller when close to the goal.

`scripts/supervisor.py`: Node that manages high level mission planning and animal rescue.

`scripts/utils/utils.py`: Utility functions. Currently contains a wrapToPi
function.

**Tensorflow Models:**
----------------------

The `.pb` files in the `tfmodels` folder are "frozen" neural network models, and
contain both the structure and the weights of pretrained networks.
`ssd_mobilenet_v1_coco.pb` is a pretrained MobileNet v1 model, while
`stop_sign_gazebo.pb` is a model fine-tuend to detect stop signs in Gazebo. 

The `coco_labels.txt` file just contains the mapping from the class number
output by the model to human-interpretable labels.
