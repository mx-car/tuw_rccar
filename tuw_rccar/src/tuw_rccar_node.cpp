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

void RCCarNode::callbackWrite ( const tuw_nav_msgs::JointsIWS &_inp ) {
    twist_velocity = _inp.revolute[0] * 50; // Ackermann commands
    twist_steering_angle = _inp.steering[0] * 0.26f;
}


void RCCarNode::publish () {

    static ros::Time time_last = ros::Time::now();

    ros::Time time_now = ros::Time::now();

    double dt = (time_now - time_last).toSec();
    time_last = time_now;

    double vel_tmp = (twist_velocity/50.0f) * 0.1277f;
    double angle_tmp = twist_steering_angle;

    float achsabstand = 0.26;

    geometry_msgs::Twist cmd;
    // creates motion command
    cmd.linear.x = vel_tmp;
    cmd.linear.y = 0.;
    cmd.angular.z = 1/achsabstand * vel_tmp * sin(angle_tmp);
    // publishes motion command
    publisher_.publish ( cmd );
}
