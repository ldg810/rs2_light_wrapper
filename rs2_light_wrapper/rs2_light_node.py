# This code is made from https://github.com/SebastianGrans/ROS2-Point-Cloud-Demo.git
# Author : Dongoo Lee, ldg810@gmail.com

import sys
import os
import time
import pyrealsense2.pyrealsense2 as rs  # In Jetson Nano, you should import pyrealsense2.pyrealsense2 as rs,
                                        # In general desktop(amd64), you should import pyrealsense2 as rs
import numpy as np

import rclpy 
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs

class PCDPublisher(Node):

    def __init__(self):
        super().__init__('pcd_publisher_node')

        self.declare_parameter('frame_name', 'camera_frame')
        self.declare_parameter('depth_width', 320)
        self.declare_parameter('depth_height', 240)
        self.declare_parameter('depth_fps', 30)
        
        param_frame_name = self.get_parameter('frame_name')
        param_depth_width = self.get_parameter('depth_width')
        param_depth_height = self.get_parameter('depth_height')
        param_depth_fps = self.get_parameter('depth_fps')
       
        # I create a publisher that publishes sensor_msgs.PointCloud2 to the 
        # topic 'pcd'. The value '5' refers to the history_depth, which I 
        # believe is related to the ROS1 concept of queue size. 
        # Read more here: 
        # http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers
        self.pcd_publisher = self.create_publisher(sensor_msgs.PointCloud2, 'rs2_pc', 5)

        pipeline = rs.pipeline()
        config = rs.config()
        # You should modify resolution for stream here.
        config.enable_stream(rs.stream.depth, param_depth_width.value, param_depth_height.value, rs.format.z16, param_depth_fps.value)
        profile = pipeline.start(config)
        frame_count = 0
        try:
#            prevTime = 0
            while True:
                frames = pipeline.wait_for_frames()
                depth_frame = frames.get_depth_frame()
                pc = rs.pointcloud()
                points = pc.calculate(depth_frame)
                v = points.get_vertices()
                verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz
                
                # Filter only valid points (from BaseRealSenseNode::publishPointCloud in librealsense2)
                # TODO : Speed up filtering valid points with CUDA
#                verts_valid = []
#                depth_frame_data = np.asarray(depth_frame.get_data())
#                index_point = 0
#                for x in range(depth_frame_data.shape[0]):
#                    for y in range(depth_frame_data.shape[1]):
#                        if(depth_frame_data[x][y] > 0):
#                            verts_valid.append(verts[index_point])
#                        index_point = index_point + 1
#                verts_valid = np.asanyarray(verts_valid)

                self.pcd = point_cloud(verts, param_frame_name.value)
                self.pcd_publisher.publish(self.pcd)
#                curTime = time.time()
#                sec = curTime - prevTime
#                fps = 1/(sec)
                frame_count += 1
                if (frame_count % 10 == 0):
                    self.get_logger().info('frame_count : %d' % frame_count)
#                print(frame_count)
#                prevTime = curTime
        finally:
            pipeline.stop()

def point_cloud(points, parent_frame):
    """ Creates a point cloud message.
    Args:
        points: Nx3 array of xyz positions.
        parent_frame: frame in which the point cloud is defined
    Returns:
        sensor_msgs/PointCloud2 message

    Code source:
        https://gist.github.com/pgorczak/5c717baa44479fa064eb8d33ea4587e0

    References:
        http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html
        http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointField.html
        http://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html

    """
    # In a PointCloud2 message, the point cloud is stored as an byte 
    # array. In order to unpack it, we also include some parameters 
    # which desribes the size of each individual point.
    
    ros_dtype = sensor_msgs.PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize # A 32-bit float takes 4 bytes.

    data = points.astype(dtype).tobytes() 

    # The fields specify what the bytes represents. The first 4 bytes 
    # represents the x-coordinate, the next 4 the y-coordinate, etc.
    fields = [sensor_msgs.PointField(
        name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate('xyz')]

    # The PointCloud2 message also has a header which specifies which 
    # coordinate frame it is represented in. 
    header = std_msgs.Header(frame_id=parent_frame)

    return sensor_msgs.PointCloud2(
        header=header,
        height=1, 
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 3), # Every point consists of three float32s.
        row_step=(itemsize * 3 * points.shape[0]),
        data=data
    )

def main(args=None):
    # Boilerplate code.
    rclpy.init(args=args)
    pcd_publisher = PCDPublisher()
    rclpy.spin(pcd_publisher)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pcd_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
