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

    subscriber_ = n.subscribe("rccar_write", 2, &RCCarNode::callbackWrite, this);
    publisher_ = n.advertise<geometry_msgs::Twist> ( "rccar_read", 1 );

    reconfigureFnc_ = boost::bind ( &RCCarNode::callbackConfigRCCar, this,  _1, _2 );
    reconfigureServer_.setCallback ( reconfigureFnc_ );
}

void RCCarNode::callbackConfigRCCar ( tuw_rccar::RCCarConfig &config, uint32_t level ) {
    config_ = config;
    init();
}

void RCCarNode::callbackWrite ( const geometry_msgs::Twist &_inp ) {
    twist_velocity = _inp.linear.x * -50;
    twist_steering_angle = _inp.angular.z;
}


void RCCarNode::publish () {
/*    geometry_msgs::Twist cmd;
    // creates motion command
    cmd.linear.x = cmd_.v();
    cmd.linear.y = 0.;
    cmd.angular.z = cmd_.w();
    // publishes motion command
    pub_cmd_.publish ( cmd );*/
}
