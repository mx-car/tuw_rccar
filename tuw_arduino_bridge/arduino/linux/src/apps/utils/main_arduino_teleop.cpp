/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2012 by Markus Bader <markus.bader@tuwien.ac.at>        *
 *                                                                         *
 *   Redistribution and use in source and binary forms, with or without    *
 *   modification, are permitted provided that the following conditions    *
 *   are met:                                                              *
 *                                                                         *
 *   1. Redistributions of source code must retain the above copyright     *
 *      notice, this list of conditions and the following disclaimer.      *
 *   2. Redistributions in binary form must reproduce the above copyright  *
 *      notice, this list of conditions and the following disclaimer in    *
 *      the documentation and/or other materials provided with the         *
 *      distribution.                                                      *
 *   3. Neither the name of the copyright holder nor the names of its      *
 *      contributors may be used to endorse or promote products derived    *
 *      from this software without specific prior written permission.      *
 *                                                                         *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   *
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     *
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS     *
 *   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE        *
 *   COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,  *
 *   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,  *
 *   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;      *
 *   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER      *
 *   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT    *
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY *
 *   WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           *
 *   POSSIBILITY OF SUCH DAMAGE.                                           *
 ***************************************************************************/
#include <iostream>
#include <stdlib.h>

#include <signal.h>
#include <limits>
#include <fcntl.h>
#include <linux/joystick.h>
#include <boost/program_options.hpp>
#include <tuw_shm/shm_arduino_objects.h>

#define JOY_TYPE_UNKNOWN                0
#define JOY_TYPE_LT_GAMEPAD_WIRELESS    1
#define JOY_TYPE_LT_GAMEPAD_WIRED       2
#define JOY_TYPE_LT_RACINGWHEEL         3

size_t counter;
size_t activeParameter;

using namespace std;
bool loop_main = true;
void ctrl_handler ( int s ) {
    loop_main = false;
}

struct Parameters {
    tuw::shm::Parameters shm;
    string joystick_port;
    string joystick_type;
    float limit;
    bool close;
};

bool serial_cout=true;
bool joystick_cout=false;

Parameters readArgs ( int argc, char **argv ) {
    namespace po = boost::program_options;

    Parameters params;
    po::options_description desc ( "Allowed Parameters" );
    desc.add_options()
    ( "help", "get this help message" )
    ( "joydev,j", po::value<string> ( &params.joystick_port )->default_value ( "/dev/input/js0" ), "joystick port" )
    ( "joytype,t", po::value<string> ( &params.joystick_type )->default_value ( "LT_Gamepad_wireless" ), "joystick type (LT_Gamepad_wireless, LT_Gamepad_wired, LT_RacingWheel)" )
    ( "limit,l", po::value<float> ( &params.limit )->default_value ( 0.2 ), "Factor for max. speed" )
    ( "shm_memory_name,m", po::value<std::string> ( &params.shm.shm_memory_name )->default_value ( ShmFw::DEFAULT_SEGMENT_NAME() ), "shared memory segment name" )
    ( "shm_memory_size,s", po::value<unsigned int> ( &params.shm.shm_memory_size )->default_value ( ShmFw::DEFAULT_SEGMENT_SIZE() ), "shared memory segment size" );


    po::variables_map vm;
    try {
        po::store ( po::parse_command_line ( argc, argv, desc ), vm );
    } catch ( const std::exception &ex ) {
        std::cout << desc << std::endl;;
        exit ( 1 );
    }
    po::notify ( vm );

    if ( vm.count ( "help" ) )  {
        std::cout << desc << std::endl;
        exit ( 1 );
    }
    params.shm.clear = false;
    return params;
}



int main ( int argc, char *argv[] ) {

    Parameters params = readArgs ( argc, argv );
    tuw::shm::Objects shm(params.shm);

    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = ctrl_handler;
    sigemptyset ( &sigIntHandler.sa_mask );
    sigIntHandler.sa_flags = 0;
    sigaction ( SIGINT, &sigIntHandler, NULL );


    int joy_fd = -1;
    int joy_type = JOY_TYPE_UNKNOWN;

    if ( ( joy_fd = open ( params.joystick_port.c_str() , O_RDONLY ) ) == -1 ) {
        cerr << "Couldn't open joystick!" << endl;
        return -1;
    }
    int numAxis = 0, numButtons = 0;
    ioctl ( joy_fd, JSIOCGAXES, &numAxis );
    ioctl ( joy_fd, JSIOCGBUTTONS, &numButtons );

    cout << "Joystick has " << numAxis << " axes and " << numButtons << " buttons!" << endl;

    if (strcmp(params.joystick_type.c_str(), "LT_Gamepad_wireless") == 0)
        joy_type = JOY_TYPE_LT_GAMEPAD_WIRELESS;
    else if (strcmp(params.joystick_type.c_str(), "LT_Gamepad_wired") == 0)
        joy_type = JOY_TYPE_LT_GAMEPAD_WIRED;
    else if (strcmp(params.joystick_type.c_str(), "LT_RacingWheel") == 0)
        joy_type = JOY_TYPE_LT_RACINGWHEEL;

    if (joy_type == JOY_TYPE_UNKNOWN) {
        cerr << "Unknown Joystick Type!" << endl;
        exit ( 1 );
    }

    if (params.limit < -1) params.limit = -1;
    if (params.limit >  1) params.limit =  1;

    struct js_event js; //struct for joystick reads
    bool running = true;
    tuw::arduino::Actuators actuators;
    tuw::arduino::Vector2 axis0 ( 0,0 ), axis1 ( 0,0 ), axis2 ( 0,0 );
    bool dead_man_button = false;
    while ( loop_main && running ) {
        if ( read ( joy_fd, &js, sizeof ( struct js_event ) ) == -1 && errno != EAGAIN ) {
            cerr << "Error while reading joystick event!" << endl;
            running = false;
        }
        if ( ( js.type & ~JS_EVENT_INIT ) == JS_EVENT_AXIS ) {
            if ( js.number == 0 ) {
                if ( joystick_cout ) cout << "js.number: " << ( int ) js.number << " = " << js.value << endl;
                axis0.y = ( ( float ) js.value ) / ( float ) SHRT_MAX;
            } else if ( js.number == 1 ) {
                if ( joystick_cout ) cout << "js.number: " << ( int ) js.number << " = " << js.value << endl;
                axis0.x = ( ( float ) js.value ) / ( float ) SHRT_MAX;
            } else if ( js.number == 2 ) {
                if ( joystick_cout ) cout << "js.number: " << ( int ) js.number << " = " << js.value << endl;
                axis1.y = ( ( float ) js.value ) / ( float ) SHRT_MAX;
            } else if ( js.number == 3 ) {
                if ( joystick_cout ) cout << "js.number: " << ( int ) js.number << " = " << js.value << endl;
                axis1.x  = ( ( float ) js.value ) / ( float ) SHRT_MAX;
            } else if ( js.number == 4 ) {
                if ( joystick_cout ) cout << "js.number: " << ( int ) js.number << " = " << js.value << endl;
                axis2.x  = ( ( float ) js.value ) / ( float ) SHRT_MAX;
            } else if ( js.number == 5 ) {
                if ( joystick_cout ) cout << "js.number: " << ( int ) js.number << " = " << js.value << endl;
                axis2.x  = ( ( float ) js.value ) / ( float ) SHRT_MAX;
            }
        } else if ( ( js.type & ~JS_EVENT_INIT ) == JS_EVENT_BUTTON ) {
            if ( js.number == 0 && js.value == 1 ) {
                if ( joystick_cout ) cout << "js.number:" << ( int ) js.number << endl;
            } else if ( js.number == 1 && js.value == 1 ) {
                if ( joystick_cout ) cout << "js.number:" << ( int ) js.number << endl;
            } else if ( js.number == 2 && js.value == 1 ) {
                if ( joystick_cout ) cout << "js.number:" << ( int ) js.number << endl;
            } else if ( js.number == 3 && js.value == 1 ) {
                if ( joystick_cout ) cout << "js.number:" << ( int ) js.number << endl;
            } else if ( js.number == 4 ) {
                if ( joystick_cout ) cout << "js.number:" << ( int ) js.number << endl;
                dead_man_button = js.value;
            } else if ( js.number == 5 && js.value == 1 ) {
                if ( joystick_cout ) cout << "js.number:" << ( int ) js.number << endl;
            } else if ( js.number == 6 && js.value == 1 ) {
                if ( joystick_cout ) cout << "js.number:" << ( int ) js.number << endl;
            } else if ( js.number == 7 && js.value == 1 ) {
                if ( joystick_cout ) cout << "js.number:" << ( int ) js.number << endl;
            } else if ( js.number == 8 && js.value == 1 ) {
                if ( joystick_cout ) cout << "js.number:" << ( int ) js.number << endl;
            } else if ( js.number == 9 && js.value == 1 ) {
                if ( joystick_cout ) cout << "js.number:" << ( int ) js.number << endl;
            }
        }
        if ( joystick_cout ) cout << flush;
        usleep ( 20 );

        //cout << (dead_man_button?"on ":"off") << axis0 << ", " << y0 << endl;
        actuators.zero();
        if ( dead_man_button ) {
            switch (joy_type) {
                case JOY_TYPE_LT_GAMEPAD_WIRELESS:
                    actuators.rps = axis0.x*500.;
                    actuators.rad = -axis1.y*M_PI/8.;
                break;

                case JOY_TYPE_LT_GAMEPAD_WIRED:
                    actuators.rps = axis0.x*500.;
                    actuators.rad = -axis1.x*M_PI/8.;
                break;

                case JOY_TYPE_LT_RACINGWHEEL:
                    actuators.rps = axis0.x*500.;
                    actuators.rad = -axis0.y*M_PI/8.;
                break;
            }
        }
        actuators.rps *= params.limit; // Limit Maximum Speed
        shm.command_actuators.set(actuators);

    }
    cout<<"good-bye!" <<endl;

}



