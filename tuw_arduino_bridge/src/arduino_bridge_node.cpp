#include "arduino_bridge_node.h"

int main ( int argc, char **argv ) {

    ros::init ( argc, argv, "arduino_bridge" );  /// initializes the ros node with default name
    ros::NodeHandle n;
    ArduinoBridgeNode bridge ( n );
    ros::Rate rate ( 10 );  /// ros loop frequence synchronized with the wall time (simulated time)

    while ( ros::ok() ) {

        /// calls all callbacks waiting in the queue
        ros::spinOnce();

        /// sleeps for the time remaining to let us hit our publish rate
        rate.sleep();
    }
    return 0;
}

/**
 * Constructor
 **/
ArduinoBridgeNode::ArduinoBridgeNode ( ros::NodeHandle & n ) :
    n_ ( n ),
    n_param_ ( "~" ) {

    std::string port ( "/dev/ttyACM0" );
    int baudrate = 115200;
    n_param_.getParam ( "port", port );
    n_param_.getParam ( "baudrate", baudrate );
    ROS_INFO ( "port: %s @ %i Bit/s", port.c_str(), baudrate );

    /// defines a publisher for velocity commands
    pub_cmd_ = n.advertise<geometry_msgs::Twist> ( "cmd_vel", 1 );

}