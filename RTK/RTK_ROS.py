from RTK_client import process_serial_data
import serial

import rospy
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, UInt32, Int32
from geometry_msgs.msg import Twist


# Initialize ROS node
rospy.init_node('my_rtk_node')

# Create publishers
imu_pub = rospy.Publisher('imu/data', Imu, queue_size=10)
gps_pub = rospy.Publisher('gps/fix', NavSatFix, queue_size=10)
time_pub = rospy.Publisher('device_time', Float32, queue_size=10)
status_pub = rospy.Publisher('status', UInt32, queue_size=10)
velocity_pub = rospy.Publisher('velocity', Twist, queue_size=10)
scaled_llh_pub = rospy.Publisher('/scaled_llh', NavSatFix, queue_size=10)
#print("working")


def my_callback(parsed_data):
    #print(parsed_data)

    # Check if IMU data is present
    if "Acceleration(XYZ)" in parsed_data and "Gyro(XYZ)" in parsed_data:
        # Create and publish IMU message
        imu_msg = Imu()
        imu_msg.header.frame_id = 'map'
        imu_msg.linear_acceleration.x = parsed_data["Acceleration(XYZ)"][0]
        imu_msg.linear_acceleration.y = parsed_data["Acceleration(XYZ)"][1]
        imu_msg.linear_acceleration.z = parsed_data["Acceleration(XYZ)"][2]
        imu_msg.angular_velocity.x = parsed_data["Gyro(XYZ)"][0]
        imu_msg.angular_velocity.y = parsed_data["Gyro(XYZ)"][1]
        imu_msg.angular_velocity.z = parsed_data["Gyro(XYZ)"][2]
#        print("Acceleration")
        imu_pub.publish(imu_msg)

    # Check if GPS data is present
    if "Latitude-Longitude-Elevation" in parsed_data:
        # Create and publish GPS message
        gps_msg = NavSatFix()
        gps_msg.header.frame_id = 'map'
        gps_msg.latitude = parsed_data["Latitude-Longitude-Elevation"][0]
        gps_msg.longitude = parsed_data["Latitude-Longitude-Elevation"][1]
        gps_msg.altitude = parsed_data["Latitude-Longitude-Elevation"][2]
#        print("GPS")
        gps_pub.publish(gps_msg)

    # Check if Time data is present
 #   if "Time" in parsed_data:
  #      print("Time")
   #     time_msg = Float32()
    #    time_msg.data = parsed_data["Time"]
     #   time_pub.publish(time_msg)

    # Check if Status data is present
    if "Status" in parsed_data:
 #       print("Status")
        status_msg = UInt32()
        status_msg.data = int(parsed_data["Status"].encode('hex'), 16) if isinstance(parsed_data["Status"], str) else parsed_data["Status"][0]


        status_pub.publish(status_msg)

    # Check if Velocity data is present
    if "Velocity" in parsed_data:
        # Create and publish velocity message
        velocity_msg = Twist()
        velocity_msg.linear.x = parsed_data["Velocity"][0]
        velocity_msg.linear.y = parsed_data["Velocity"][1]
        velocity_msg.linear.z = parsed_data["Velocity"][2]
 #       print("Velocity")
        velocity_pub.publish(velocity_msg)

    # Check if Scaled-LLH data is present
    if "Scaled-LLh" in parsed_data:
        # Create and publish Scaled-LLH message
        scaled_llh_msg = NavSatFix()
        scaled_llh_msg.header.frame_id = 'map'
        scaled_llh_msg.latitude = parsed_data["Scaled-LLh"][0]
        scaled_llh_msg.longitude = parsed_data["Scaled-LLh"][1]
        scaled_llh_msg.altitude = parsed_data["Scaled-LLh"][2]

 #       print("Scaled-LLh")
        scaled_llh_pub.publish(scaled_llh_msg)

ser = serial.Serial('/dev/ttyUSB0', baudrate=115200)
process_serial_data(ser, my_callback)
