#!/usr/bin/env python
import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

target = []
region = []
position = []
regionCoordination = [[(-4,4),(-14,14)],[(-3,4),(-10,14)],[(-2,4),(-6,14)],[(-1,4),(-2,14)],[(1,4),(2,14)],[(2,4),(6,14)],[(3,4),(10,14)],[(4,4),(14,14)],
[(-4,3),(-14,10)],[(-3,3),(-10,10)],[(-2,3),(-6,10)],[(-1,3),(-2,10)],[(1,3),(2,10)],[(2,3),(6,10)],[(3,3),(10,10)],[(4,3),(14,10)],
[(-4,2),(-14,6)],[(-3,2),(-10,6)],[(-2,2),(-6,6)],[(-1,2),(-2,6)],[(1,2),(2,6)],[(2,2),(6,6)],[(3,2),(10,6)],[(4,2),(14,10)],
[(-4,1),(-14,2)],[(-3,1),(-10,2)],[(-2,1),(-6,2)],[(-1,1),(-2,2)],[(1,1),(2,2)],[(2,1),(6,2)],[(3,1),(10,2)],[(4,1),(14,2)],
[(-4,-1),(-14,-2)],[(-3,-1),(-10,-2)],[(-2,-1),(-6,-2)],[(-1,-1),(-2,-2)],[(1,-1),(2,-2)],[(2,-1),(6,-2)],[(3,-1),(10,-2)],[(4,-1),(14,-2)],
[(-4,-2),(-14,-6)],[(-3,-2),(-10,-6)],[(-2,-2),(-6,-6)],[(-1,-2),(-2,-6)],[(1,-2),(2,-6)],[(2,-2),(6,-6)],[(3,-2),(10,-6)],[(4,-2),(14,-6)],
[(-4,-3),(-14,-10)],[(-3,-3),(-10,-10)],[(-2,-3),(-6,-10)],[(-1,-3),(-2,-10)],[(1,-3),(2,-10)],[(2,-3),(6,-10)],[(3,-3),(10,-10)],[(4,-3),(14,-10)],
[(-4,-4),(-14,-14)],[(-3,-4),(-10,-14)],[(-2,-4),(-6,-14)],[(-1,-4),(-2,-14)],[(1,-4),(2,14)],[(2,-4),(6,14)],[(3,-4),(10,-14)],[(4,-4),(14,-14)]]

def subscribe():
    while not rospy.is_shutdown():
        rospy.Subscriber("odom", Odometry, localize)
        rospy.Subscriber("base_scan", LaserScan, move)
        rospy.spin()

def approxRegion():
    target.append(float(input("Enter X position of the target: ")))
    target.append(float(input("Enter Y position of the target: ")))
    xy = []
    for coordinate in target:
        if coordinate / 4 < 0:
            xy.append((int(coordinate / 4) - 1))
        else:
            xy.append((int(coordinate / 4) + 1))
    for values in regionCoordination:
        if values[0] == tuple(xy):
            region = values[1]
            return region

def localize(data):
    position.clear()
    position.append(data.pose.pose.position.x)
    position.append(data.pose.pose.position.y)
    position.append(data.pose.pose.orientation.z)

def move(data):
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
    left = data.ranges[401:501]
    leftforward = data.ranges[301:401]
    forward = data.ranges[201:301]
    rightforward = data.ranges[101:201]
    right = data.ranges[0:101]
    leftAvg = sum(left) / len(left)
    leftforwardAvg = sum(leftforward) / len(leftforward)
    forwardAvg = sum(forward) / len(forward)
    rightforwardAvg = sum(rightforward) / len(rightforward)
    rightAvg = sum(right) / len(right)
    views = [leftAvg,leftforwardAvg,forwardAvg,rightforwardAvg,rightAvg]

    base_data = Twist()
    #rounded pointing toward the center point
    if min([forwardAvg, leftAvg, leftforwardAvg, rightforwardAvg, rightAvg]) < 0.5:
        if min([forwardAvg, leftAvg, leftforwardAvg, rightforwardAvg, rightAvg]) == leftAvg:
            base_data.angular.z = -0.628
            base_data.linear.x = 0.15
        elif min([forwardAvg, leftAvg, leftforwardAvg, rightforwardAvg, rightAvg]) == leftforwardAvg:
            base_data.angular.z = -0.628 / 2
            base_data.linear.x = 0.15
        elif min([forwardAvg, leftAvg, leftforwardAvg, rightforwardAvg, rightAvg]) == rightAvg:
            base_data.angular.z = 0.628
            base_data.linear.x = 0.15
        elif min([forwardAvg, leftAvg, leftforwardAvg, rightforwardAvg, rightAvg]) == rightforwardAvg:
            base_data.angular.z = 0.628 / 2
            base_data.linear.x = 0.15
        else:
            if leftAvg + leftforwardAvg > rightAvg + rightforwardAvg:
                base_data.angular.z = 0.628
            else:
                base_data.angular.z = -0.628
            base_data.linear.x = 0.15
        for i in range(40):
            pub.publish(base_data)
    elif round(position[2]*3.14,1) != round((math.atan2(region[1] - position[1], region[0] - position[0]) % 3.14),1) and round(position[2]*3.14,1) != round((-(3.14 - math.atan2(region[1] - position[1], region[0] - position[0]) % 3.14)),1):
        if region[1]>position[1]:
            if round(position[2]*3.14,1) != round((math.atan2(region[1] - position[1], region[0] - position[0]) % 3.14),1):
                base_data.linear.x = 0.15
                base_data.angular.z = 0.1
                pub.publish(base_data)
        elif region[1]<position[1]:
            if round(position[2]*3.14,1) != round((-(3.14 - math.atan2(region[1] - position[1], region[0] - position[0]) % 3.14)),1):
                base_data.linear.x = 0.1
                base_data.angular.z = -0.1
                pub.publish(base_data)
    else:
        base_data.linear.x = 0.15
        pub.publish(base_data)

if __name__ == '__main__':
    rospy.init_node('searcher', anonymous=True)
    region = approxRegion()
    subscribe()