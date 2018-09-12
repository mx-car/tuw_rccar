#include "tuw_rccar_node.h"
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <tf/transform_datatypes.h>

using namespace tuw;
int main ( int argc, char **argv ) {

    ros::init ( argc, argv, "rccar" );  // initializes the ros node with default name
    ros::NodeHandle n;
    RCCarNode rccar ( n );
    rccar.init();
    ros::Rate rate ( 30 );  // ros loop frequence synchronized with the wall time (simulated time)

    while ( ros::ok() ) {

        // sets and publishes velocity commands
        rccar.publish();

        // calls all callbacks waiting in the queue
        ros::spinOnce();

        // sleeps for the time remaining to let us hit our publish rate
        rate.sleep();
    }
    return 0;
}

/**
 * Constructor
 **/
RCCarNode::RCCarNode ( ros::NodeHandle & n )
    : RCCar(ros::NodeHandle("~").getNamespace()),
    n_ ( n ),
    n_param_ ( "~" ){

    subscriber_ = n.subscribe("joint_cmds", 2, &RCCarNode::callbackWrite, this);
    publisher_twist_ = n.advertise<geometry_msgs::TwistStamped> ( "rccar_read", 1 );
    publisher_imu_ = n.advertise<sensor_msgs::Imu> ( "rccar_imu", 1 );

    servprov_ = n.advertiseService("config_read", &RCCarNode::callbackServiceConfig, this);
    servprov_pid_ = n.advertiseService("pid_controller", &RCCarNode::callbackServicePIDController, this);

    reconfigureFnc_ = boost::bind ( &RCCarNode::callbackConfigRCCar, this,  _1, _2 );
    reconfigureServer_.setCallback ( reconfigureFnc_ );
}

//bool RCCarNode::callbackServiceConfig ( std_srvs::Empty::Request& request, std_srvs::Empty::Response& response ) {
bool RCCarNode::callbackServiceConfig ( tuw_rccar::AckermannConfig::Request& request, tuw_rccar::AckermannConfig::Response& response ) {
    ROS_INFO("callbackServiceConfig called");
    service_ackermann_config = true;

    while (service_ackermann_config) {
        // Busy Wait for response
        printf("waiting for response...\n");
        usleep(10000);
    }

    response.axle_distance = amcfg_.d;
    response.wheelbase = amcfg_.l;
    response.weight = amcfg_.weight;
    response.steering_angle = amcfg_.max_steer_angle;

    return true;
}

bool RCCarNode::callbackServicePIDController ( tuw_rccar::PIDController::Request& request, tuw_rccar::PIDController::Response& response ) {
    ROS_INFO("callbackServicePIDController called");
    service_pid_controller = true;

    while (service_pid_controller) {
        // Busy Wait for response
        printf("waiting for response...\n");
        usleep(10000);
    }

    printf("finished waiting\n");
/*    response.axle_distance = amcfg_.d;
    response.wheelbase = amcfg_.l;
    response.weight = amcfg_.weight;
    response.steering_angle = amcfg_.max_steer_angle;
*/
    return true;
}

void RCCarNode::callbackConfigRCCar ( tuw_rccar::RCCarConfig &config, uint32_t level ) {
    config_ = config;
    init();
}

void RCCarNode::callbackWrite ( const tuw_nav_msgs::JointsIWS &msg ) {
    if( msg.type_steering.compare("cmd_position") != 0 ) { 
      ROS_ERROR("Joint cmd type_steering: \"%s\" is not supported", msg.type_steering.c_str() );
    }
    if( msg.type_revolute.compare("cmd_velocity") != 0  ) { 
      ROS_ERROR("Joint cmd type_steering: \"%s\" is not supported", msg.type_revolute.c_str() );
    }
    if(msg.revolute.size() != 2){
      ROS_ERROR("Joint cmd incorrect number of joints: %zu is not supported expected are two!", msg.revolute.size());
    }
    if(msg.steering.size() != 2){
      ROS_ERROR("Joint cmd incorrect number of joints: %zu is not supported expected are two!", msg.steering.size());
    }
    if((msg.revolute[0] == 0.0) || std::isnan(msg.revolute[0]) ){
      actuators_.rad = msg.steering[0] * 0.26f;
    } else {
      ROS_ERROR("Joint cmd revolute[0] must be zero or NAN");
    }
    if((msg.steering[1] == 0.0) || std::isnan(msg.steering[1]) ){
      actuators_.rps = msg.revolute[1] * -50; // Ackermann commands
    } else {
      ROS_ERROR("Joint cmd steering[1] must be zero or NAN");
    }
    actuators_last_received = ros::Time::now();
}


void RCCarNode::publish () {

    static ros::Time time_last = ros::Time::now();

    ros::Time time_now = ros::Time::now();

    double dt = (time_now - time_last).toSec();
    time_last = time_now;

    double vel_tmp = (actuators_.rps/50.0f) * -0.1277f;
    double angle_tmp = actuators_.rad;

    float achsabstand = 0.26;

    geometry_msgs::TwistStamped cmd;
    cmd.header.stamp = time_now;
    // creates motion command
    cmd.twist.linear.x = vel_tmp;
    cmd.twist.linear.y = 0.;
    cmd.twist.angular.z = 1/achsabstand * vel_tmp * sin(angle_tmp);
    // publishes motion command
    publisher_twist_.publish ( cmd );


    if (imu_last_received != imu_last_sent) {
        imu_last_sent = imu_last_received;
        publisher_imu_.publish(imu_);
    }


}
