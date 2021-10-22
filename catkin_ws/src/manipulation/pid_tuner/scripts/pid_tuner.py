#!/usr/bin/env python
import rospy
from os import path, makedirs
from yaml import dump
from rospkg import RosPack
from numpy import std as stdev
from datetime import datetime
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
import dynamic_reconfigure.client

PACKAGE_PATH = RosPack().get_path('pid_tuner')
NUM_VALUES_REPORTED = 100

def get_accuracy(movement, values_move, values_ret):
	average_move = float(sum(values_move))/NUM_VALUES_REPORTED
	average_ret = float(sum(values_ret))/NUM_VALUES_REPORTED
	ac_move = 1-2*abs(average_move - movement)
	ac_ret = 1-2*abs(average_ret)
	std_move = 1-5*abs(stdev(values_move))
	std_ret = 1-5*abs(stdev(values_ret))
	return (ac_move*0.35 + ac_ret*0.45 + std_move*0.075 + std_ret*0.125),(std_move*0.4 + std_ret*0.6)

def get_pos(pos_subscriber):
	return rospy.wait_for_message(pos_subscriber, JointControllerState).process_value

def get_best_val(best_val, joint, movement_publisher, iterable, movement, configure_client, pos_subscriber, parameter):	
	bad_val	= 0
	for val in iterable:	
		values_move = []
		values_ret = []
		print(' Testing for: {}'.format(val))
		configure_client.update_configuration({parameter: val})
		movement_publisher.publish(Float64(movement))
		rospy.sleep(0.5)
		for _ in range(NUM_VALUES_REPORTED):
			values_move.append(get_pos(pos_subscriber))
		movement_publisher.publish(Float64(0))
		rospy.sleep(0.5)		
		for _ in range(NUM_VALUES_REPORTED):
			values_ret.append(get_pos(pos_subscriber))
		accuracy, deviation = get_accuracy(movement, values_move, values_ret)
		if accuracy > best_val['accuracy']:
			best_val['val'] = val
			best_val['accuracy'] = accuracy
			bad_val = 0
		elif accuracy < 0.70:
			bad_val += 1
		else:
			bad_val = 0
		print('  Accuracy: {:.4f}'.format(accuracy))
		rospy.sleep(0 if deviation>=0.75 else (0.75-deviation)//0.1)
		if bad_val==5: break

def tune(joints):
	for joint in joints.keys():
		movement_publisher = joints[joint]['publisher']
		configure_client = joints[joint]['client']
		configure_client.update_configuration({'i':0.0,'d':0.0})
		print('-'*50+'\nJOINT: {}'.format(joint))
		for parameter in 'pid':
			print(' Testing {} parameter'.format(parameter))
			movement_publisher.publish(Float64(0))
			rospy.sleep(1)
			rep = 0		
			best_val = {'val': 0, 'accuracy': float('-inf')}
			while rep < joints[joint]['run_params']['msr']:
				iterable = range(rep*10+1,rep*10+11)
				get_best_val(best_val, joint, movement_publisher, iterable, joints[joint]['run_params']['movement'],configure_client, joints[joint]['subscriber'], parameter)
				rep+=1			
				if best_val['accuracy'] >= 0.9: break
			print(' Best int {} val after {} values tested: {} with an accuracy of: {:.4f}\n'.format(parameter,rep*10,best_val['val'],best_val['accuracy']))
			print(' Testing float values')
			for accuracy in range(1,joints[joint]['run_params']['accuracy']+1):
				iterable = [ (best_val['val']-5*(1.0/10**accuracy)+x*(1.0/10**accuracy)) for x in range(0, 11)]
				best_val = {'val': 0, 'accuracy': float('-inf')}
				get_best_val(best_val, joint, movement_publisher, iterable, joints[joint]['run_params']['movement'], configure_client, joints[joint]['subscriber'], parameter)
			joints[joint]['params'][parameter] = best_val['val']		
			print(' Best {} val: {} with an accuracy of: {:.4f}\n'.format(parameter,best_val['val'],best_val['accuracy']))
		configure_client.update_configuration(joints[joint]['params'])	
		YAML = {joint+'_controller':{'type': 'effort_controllers/JointPositionController','joint':joint+'_joint' if joint[3] in range(1,8) else joint, 'pid': joints[joint]['params']}}	
		try:
			print(' Dumping YAML file into '+joints[joint]['file'])
			if not path.exists(joints[joint]['file'].rsplit('/',1)[0]):
				makedirs(joints[joint]['file'].rsplit('/',1)[0])
			with open(joints[joint]['file'],'w+') as file:
				file.write(dump(YAML))
		except (OSError, IOError):
			print('Error dumping file, dumping it into package dir')
			with open(PACKAGE_PATH+'/'+joint+'.yaml','w+') as file:
				file.write(dump(YAML))	
	
if __name__ == '__main__':
	try:
		start = datetime.now()
		rospy.init_node('pid_tuner', anonymous=False)
		joints = {}
		if rospy.has_param('~joints_to_tune'):
			joint_dict = rospy.get_param('~joints_to_tune')
			print(joint_dict)
			for joint, values in joint_dict.items():
				joints[joint] = {}			
				joints[joint]['params'] = {}
				joints[joint]['publisher'] = rospy.Publisher('/{}_controller/command'.format(joint),Float64,queue_size=1)
				joints[joint]['client'] = dynamic_reconfigure.client.Client('/{}_controller/pid'.format(joint)) 
				joints[joint]['subscriber'] = '/'+joint+'_controller/state'
				joints[joint]['file'] = PACKAGE_PATH+'/'+values['file']			
				joints[joint]['run_params'] = {}			
				try:
					joints[joint]['run_params']['movement'] = values['run_params']['movement']
				except KeyError:
					joints[joint]['run_params']['movement'] = 2 if joint[0]=='l' else 1 if joint in joint[0]=='h' else -2				
				try:
					joints[joint]['run_params']['msr'] = values['run_params']['msr']
				except KeyError:
					joints[joint]['run_params']['msr'] = 2				
				try:
					joints[joint]['run_params']['accuracy'] = values['run_params']['accuracy']
				except KeyError:
					joints[joint]['run_params']['accuracy'] = 2			
		else:
			print('No paremeters found, testing all joints with default run config')
			all_joints = ['la_{}'.format(x) for x in range(1,8)]+['ra_{}'.format(x) for x in range(1,8)]+['head_pan','head_tilt','ra_grip_right', 'ra_grip_left','la_grip_right','la_grip_left']
			for joint in all_joints:
				joints[joint] = {}
				joints[joint]['params'] = {}
				joints[joint]['publisher'] = rospy.Publisher('/{}_controller/command'.format(joint),Float64,queue_size=1)
				joints[joint]['client'] = dynamic_reconfigure.client.Client('/{}_controller/pid'.format(joint)) 
				joints[joint]['subscriber'] = '/{}_controller/state'.format(joint)
				joints[joint]['file'] = PACKAGE_PATH+'/../config_files/ros_control/{}.yaml'.format(joint)
				joints[joint]['run_params'] = {'movement':2 if joint[0]=='l' else 1 if joint in joint[0]=='h' else -2,'msr':2,'accuracy':2}

		tune(joints)
		
		end = datetime.now()
		elapsed = end-start
		print(' {} elapsed'.format(str(elapsed)))
	except rospy.exceptions.ROSInterruptException:
		print('\nEXECUTION STOPPED')
