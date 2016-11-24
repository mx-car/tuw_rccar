#ifndef __TUW_RCCAR_NODE_H
#define __TUW_RCCAR_NODE_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <tuw_nav_msgs/JointsIWS.h>
#include <dynamic_reconfigure/server.h>
#include <tuw_rccar/tuw_rccar.h>
#include <tuw_rccar/RCCarConfig.h>
/**
 * class to cover the ros communication
 **/
class RCCarNode : public tuw::RCCar {
public:
    RCCarNode ( ros::NodeHandle & n ); // Constructor
    void publish ();                   // publishes the motion commands

private:
    ros::NodeHandle n_;                // node handler to the root node
    ros::NodeHandle n_param_;          // node handler to the current node
    ros::Subscriber subscriber_;       // Subscriber to ...
    ros::Publisher publisher_;         // Publisher for ...
    void callbackWrite ( const tuw_nav_msgs::JointsIWS &_inp );                            // callback function to execute on incoming sensor data
    void callbackConfigRCCar ( tuw_rccar::RCCarConfig &config, uint32_t level );        // callback function on incoming parameter changes
    dynamic_reconfigure::Server<tuw_rccar::RCCarConfig> reconfigureServer_;             // parameter server stuff
    dynamic_reconfigure::Server<tuw_rccar::RCCarConfig>::CallbackType reconfigureFnc_;  // parameter server stuff

};

#endif // __TUW_RCCAR_NODE_H
