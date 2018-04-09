#ifndef __TUW_RCCAR_NODE_H
#define __TUW_RCCAR_NODE_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <std_srvs/Empty.h>
#include <tuw_nav_msgs/JointsIWS.h>
#include <dynamic_reconfigure/server.h>
#include <tuw_rccar/tuw_rccar.h>
#include <tuw_rccar/RCCarConfig.h>
#include <tuw_rccar/AckermannConfig.h>
#include <tuw_rccar/PIDController.h>

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
    ros::Publisher publisher_twist_;         // Publisher for ...
    ros::Publisher publisher_imu_;         // Publisher for ...
    ros::ServiceServer servprov_;      // Service Provider for ...
    ros::ServiceServer servprov_pid_;      // Service Provider for PID Controller
    void callbackWrite ( const tuw_nav_msgs::JointsIWS &msg );                            // callback function to execute on incoming sensor data
    void callbackConfigRCCar ( tuw_rccar::RCCarConfig &config, uint32_t level );        // callback function on incoming parameter changes
//    bool callbackServiceConfig ( std_srvs::Empty::Request& request, std_srvs::Empty::Response& response );
    bool callbackServiceConfig ( tuw_rccar::AckermannConfig::Request& request, tuw_rccar::AckermannConfig::Response& response );
    bool callbackServicePIDController ( tuw_rccar::PIDController::Request& request, tuw_rccar::PIDController::Response& response );

    dynamic_reconfigure::Server<tuw_rccar::RCCarConfig> reconfigureServer_;             // parameter server stuff
    dynamic_reconfigure::Server<tuw_rccar::RCCarConfig>::CallbackType reconfigureFnc_;  // parameter server stuff

};

#endif // __TUW_RCCAR_NODE_H
