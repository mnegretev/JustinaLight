#!/usr/bin/env python3
"""Drive_Justina controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera, Lidar, RangeFinder
#import rospy
#from std_msgs.msg import Float64
#from sensor_msgs.msg import Image, PointCloud2, PointField, NavSatFix, NavSatStatus, Imu

      
if __name__ == "__main__":
    print('RUNNING JUSTINA CONTROLLER NODE ...')
    """
    rospy.init_node('justina_controller')
    rate = rospy.Rate(int(1000.0/TIME_STEP))
    
    # IMAGE MESSAGE
    msg_image = Image()
    msg_image.height = camera.getHeight()
    msg_image.width = camera.getWidth()
    msg_image.is_bigendian = False
    msg_image.step = camera.getWidth() * 4
    msg_image.encoding = 'bgra8'

  # POINT CLOUD2 MESSAGE 
    msg_point_cloud = PointCloud2()
    msg_point_cloud.header.stamp = rospy.Time.now()
    msg_point_cloud.header.frame_id = 'lidar_link'  
    msg_point_cloud.height = 1
    msg_point_cloud.width = lidar.getNumberOfPoints()
    msg_point_cloud.point_step = 20
    msg_point_cloud.row_step = 20 * lidar.getNumberOfPoints()
    msg_point_cloud.is_dense = False
    msg_point_cloud.fields = [
        PointField(name = 'x', offset = 0, datatype = PointField.FLOAT32, count = 1),
        PointField(name = 'y', offset = 4, datatype = PointField.FLOAT32, count = 1),
        PointField(name = 'z', offset = 8, datatype = PointField.FLOAT32, count = 1),
    ]
    msg_point_cloud.is_bigendian = False
    """ 
    # create the Robot instance.
    robot = Robot()
    
    # get the time step of the current world.
    timeStep = int(robot.getBasicTimeStep())
    max_speed=10
    
    #instancias de los sensores
    kinectRange = robot.getDevice('kinect range')
    kinectRange.enable(timeStep)
    kinectRange = robot.getDevice('kinect color')
    kinectRange.enable(timeStep)
    lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
    lidar.enable(timeStep)
    lidar.enablePointCloud()
    
    #actuadores de los brazos
    leftArm = []
    rightArm= []
    for i in range(0,7):
        leftArm.append(robot.getDevice('LTH' + str(i+1)))
        rightArm.append(robot.getDevice('RTH' + str(i+1)))
    for i in range(0,7):
        leftArm[i].setPosition(float('inf'))
        leftArm[i].setVelocity(0.0)
    
    #motores de ruedas
    left_motor = robot.getDevice('motor_L')
    right_motor = robot.getDevice('motor_R')

    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)
    
    #actuadores de la cabeza
    tiltMotor = robot.getDevice('tiltMotor')
    panMotor = robot.getDevice('panMotor')
    
    """# PUBLISHERS
    pub_camera_data  = rospy.Publisher('/camera/rgb/raw', Image, queue_size=10)
    pub_point_cloud  = rospy.Publisher('/point_cloud'   , PointCloud2, queue_size=10)
    """
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timeStep) != -1:
       
        
        left_motor.setVelocity(0.5*max_speed)
        right_motor.setVelocity(0.5*max_speed)
        for i in range (0,7):
            leftArm[i].setVelocity(0.1*max_speed)
            
        """
        msg_image.data = camera.getImage()                                       # GET IMAGE DATA FROM CAMERA
        msg_point_cloud.data = lidar.getPointCloud(data_type='buffer')           # GET POINT CLOUD FROM LIDAR
        msg_gyro.angular_velocity.x = gyro.getValues()[0]                        # GET X COMPONENT FROM GYRO
        msg_gyro.angular_velocity.y = gyro.getValues()[1]                        # GET Y COMPONENT FROM GYRO
        msg_gyro.angular_velocity.z = gyro.getValues()[2]                        # GET Z COMPONENT FROM GYRO
        msg_gps.latitude = gps.getValues()[0]                                    # GET X COMPONENT FROM GPS  
        msg_gps.longitude = gps.getValues()[1]                                   # GET Y COMPONENT FROM GPS  
        msg_gps.altitude = gps.getValues()[2]                                    # GET Z COMPONENT FROM GPS 
    
        pub_camera_data.publish(msg_image)                                       # PUBLISHING IMAGE MESSAGE
        pub_point_cloud.publish(msg_point_cloud)                                 # PUBLISHING POINTCLOUD2 MESSAGE
        pub_imu_gyro.publish(msg_gyro)                                           # PUBLISHING IMU MESSAGE
        pub_nav_gps.publish(msg_gps)  
        """
        
  