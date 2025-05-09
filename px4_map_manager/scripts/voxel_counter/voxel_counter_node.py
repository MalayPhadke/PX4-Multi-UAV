#!/usr/bin/env python
import rclpy
from sensor_msgs.msg import PointCloud2



def setRecordFlagTimer(event):
    global recordFlag
    recordFlag = True

def counterCB(pc):
    global recordFlag
    seconds = rclpy.get_time()
    if (recordFlag):   
        print(f"map_size is {pc.width}")
        file = open("voxel_numbers.txt",'a+')
        file.write(f"time: {seconds} ")
        file.write(f"num of voxels: {pc.width}\n")
        file.close()
    recordFlag = False
    

    
def counter():
    rclpy.Subscriber("/dynamic_map/explored_voxel_map",PointCloud2,counterCB)
    pass

if __name__ == "__main__":
    global recordFlag
    global time_interval # in ms
    
    
    recordFlag = False
    time_interval = 2
    rclpy.init_node("vox_counter",anonymous=True)
    rclpy.Timer(rclpy.Duration(time_interval),setRecordFlagTimer,oneshot=False)
    counter()
    # rate = rospy.Rate(time_interval)
    # while not rospy.is_shutdown:
    #     recordFlag = True
    #     print(recordFlag)
    rclpy.spin()