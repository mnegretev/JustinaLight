
## PID Tuner

PID Tuner is a ROS Package used to tune the PID controllers of Justina.


## Usage

*Requirements*: ROS Core and Gazebo simulation already running and package files already sourced.
*Recommendation*: Use it in a free-obstacle environment to avoid collisions while testing.
***
**Using it without parameters will tune all joints with default run config:**

```console
user@pc:~$ rosrun pid_tuner pid_tuner.py
```
***
**To specify joints to tune we modify the private paremeter *joints_to_tune* using remapping arguments:**

```console
user@pc:~$ rosrun pid_tuner pid_tuner.py _joints_to_tune:=param
```
param format:
```console
_joints_to_tune:="{joint1:{'file':path,'run_params':{'movement':move_val,'msr':msr_val,'accuracy':acc_val}},joint2:...}"
```
_The *file* parameter, *run\_params* dictionary and all of their elements are optional for each joint, when they aren't specified the default value is used_

param components:
- joint: string with the name of the joint, it's the key of the dictionary that will contain all the necessary configuration to run the tuner for that joint.
	- path: string with the relative path to dump the YAML file for the joint regarding the package path.
		- example: 'config\_files/la\_1.yaml' will dump the file into 'PackageAbsolutePath/config\_files/la\_1.yaml'
	- run\_params: modifiable parameters that will change the behavior of the tuner on run for that specific joint.
		- move: the float value that will be used as movement to test the joint.
		- msr: stands for Max Set Repetitions, the tuner works by sets of 10 integer values tested, each 10 values tested for each parameter the maximum accuracy for the movement is checked, if it's good enough the test for integer values is stopped and the test for float values begins, otherwise the sets continue until the Max Set Repetition value.
			-example: for a msr of 3, a maximum of 30 integer values will be tested.
		- accuracy: decimal accuracy for the test.
			-example: with an accuracy of 2 we can obtain values as 2.43, 14.64, 7.65, etc.

*Default param values:*
- file: '../config\_files/ros\_control/joint.yaml'
	example for joint la\_1: 'PackageAbsolutePath/../config\_files/ros\_control/la\_1.yaml'
- move: 2 for left arm, -2 for right arm and 1 for head joints.
- msr: 2
- accuracy: 2

Example tuning joints la\_1, ra\_4 and la\_7:

```console
user@pc:~$ rosrun pid_tuner pid_tuner.p _joints_to_tune:="{'la_1':{'file':'la_config/yaml/la_1.yaml'},'ra_4':{'run_params':{'move':1.5,'accuracy':3}},'la_7':{'file':'la_config/yaml/la_7.yaml','run_params':{'accuracy':2,'move':-1,}}}"
```
***
**A YAML file with the joint configuration can be dumped to the private namespace before executing the node:**
```console
user@pc:~$ rosparam load dump.yaml ~
user@pc:~$ rosrun pid_tuner pid_tuner.py 
```
Example of YAML file:
```yaml
/:
  pid_tuner:
    joints_to_tune: 
      la_1: 
        file: la_config/yaml/la_1.yaml
      ra_5: 
        file: la_config/yaml/ra_5.yaml 
        run_params: {accuracy: 2}
```
***
## Joints supported
- Left arm: la\_1, la\_2, la\_3, la\_4, la\_5, la\_6, la\_7, la\_grip\_right, la\_grip\_left.
- Right arm: ra\_1, ra\_2, ra\_3, ra\_4, ra\_5, ra\_6, ra\_7, ra\_grip\_right, ra\_grip\_left.
- Head: head\_pan, head\_tilt.

