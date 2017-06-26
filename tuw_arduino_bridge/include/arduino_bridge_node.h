#ifndef ARDUINO_BRIDGE_NODE_H
#define ARDUINO_BRIDGE_NODE_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
/**
 * class to cover the ros communication
 **/
class ArduinoBridgeNode {
public:
    ArduinoBridgeNode ( ros::NodeHandle & n ); /// Constructor
private:
    ros::NodeHandle n_;         /// node handler to the root node
    ros::NodeHandle n_param_;   /// node handler to the current node
    ros::Publisher pub_cmd_;    /// publisher for the motion commands
};

#endif // ARDUINO_BRIDGE_NODE_H
