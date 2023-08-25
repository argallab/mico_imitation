# Mico Imitation Learning
Implementation of behavioral cloning with Mico
## Dependencies
Python packages
- [imitation](https://imitation.readthedocs.io/en/latest/)
- numpy
- pandas
- gymnasium

## Python2/3 Incompatibility
The `imitation` Python module requires Python3.8+, but the master branch of `mico_base` is still using ROS Kinetic, which is only comptible with Python 2. The current workaround is to have a version of [imitation_script.py](src/imitation_script.py) hosted on [pythonanywhere.com](https://www.pythonanywhere.com/) along with a Flask app. [web_client.py](src/web_client.py) uses the Python `requests` module to communicate with the Flask app over `http` to query the pre-trained policies. Requests should be made to [http://stephaniiema.pythonanywhere.com/](http://stephaniiema.pythonanywhere.com/). (Note that pythonanywhere.com sets expirations on web app hosting and will stop hosting this web app Friday 17 November 2023.)

## Usage
### Collecting Demonstration Trajectories
To collect demonstration trajectories, launch mico teleoperation
```
roslaunch mico_interaction direct_teleop.launch JOY3:=true
```
and turn on velocity mode
```
rosservice call /mico_interaction/switch_controllers "controllers_to_be_turned_on: 'velocity'"  
```
in another tab, run the `traj_collection` node
```
rosrun mico_imitation traj_collection.py
```
check to make sure that the end effector pose is being published to the `/eef_pose` topic before recording a rosbag by running
```
rosbag record -o <prefix> eef_pose
```
Note: all rosbags of the same motion type (intended to train the same model together) should be prefixed with the same name. When the `create_trajectories` function in `imitation_script.py` is run, it accepts a prefix as the argument and trains on all files which begin with that prefix.

### Policy training
Before training, convert rosbags to csv files (this can be done using [extract_topics_from_bag.py](extract_topics_from_bag.py))  
Train a policy by modifying the main of `imitation_script.py`, changing the prefix to the relevant rosbag prefix and giving it a relevant policy name (policy name can be the same as the rosbag prefix)
```
    n = imitation_class(load=False, prefix=<prefix>, position=True, policy=<policy_name>)
```
Run `imitation_script.py` to train
```
python3 imitation_script.py
```

### Running ML with Mico
To run a trained policy, first launch mico
```
roslaunch mico_interaction mico_moveit.launch
```
in another tab start the web client
```
rosrun mico_imitation web_client.py
```
before running the move_group script, move mico to the desired starting position (can be done using teleop or MoveIt). Then run `ml_move_group.py`
```
rosrun mico_imitation ml_move_group.py
```
You can change the policy being used and the number of steps it takes by modifying the main function in `ml_move_group.py` before running. Pretrained policies can be found in the [policy](policy/) folder.