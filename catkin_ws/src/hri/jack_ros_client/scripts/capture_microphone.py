#!/usr/bin/env python3
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import Float32MultiArray
from scipy.io.wavfile import write

def callback_mic(msg):
    global captured_data
    captured_data += msg.data

def main():
    global captured_data
    print("INITIALIZING CAPTURE MICROPHONE NODE...")
    rospy.init_node("capture_microphone")
    rospy.Subscriber("/hri/microphone_raw", Float32MultiArray, callback_mic)
    captured_data = []
    rospy.spin()
    print("Captured a total of " + str(len(captured_data)) + " data")
    f = open("microphone_raw_data.txt", "w")
    for x in captured_data:
        f.write(str(x) + "\n")
    f.close()
    plt.plot(captured_data)
    plt.show()

if __name__ == "__main__":
    main()
