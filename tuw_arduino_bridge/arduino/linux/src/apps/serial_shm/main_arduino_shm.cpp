
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <termios.h>
#include <boost/program_options.hpp>
#include <tuw_serial_arduino/serial_arduino.h>
#include "serial_arduino_shm.h"
#include <shmfw/variable.h>

using namespace std;

bool loop_main = true;
void ctrl_handler ( int s ) {
    loop_main = false;
}

int main ( int argc, char* argv[] ) {
    namespace po = boost::program_options;

    Parameters params;
    po::options_description desc ( "Allowed Parameters" );
    desc.add_options()
    ( "help", "get this help message" )
    ( "port,p", po::value<std::string> ( &params.serial.port )->default_value ( "/dev/ttyACM0" ), "serial port" )
    ( "baudrate,b", po::value<int> ( &params.serial.baudrate )->default_value ( 115200 ), "baudrate" )
    ( "clear,c", "clears the shared memory" )
    ( "print_rx,r", "print received objects" )
    ( "shm_memory_name,m", po::value<std::string> ( &params.shm.shm_memory_name )->default_value ( ShmFw::DEFAULT_SEGMENT_NAME() ), "shared memory segment name" )
    ( "shm_memory_size,s", po::value<unsigned int> ( &params.shm.shm_memory_size )->default_value ( ShmFw::DEFAULT_SEGMENT_SIZE() ), "shared memory segment size" );

    po::variables_map vm;
    try {
        po::store ( po::parse_command_line ( argc, argv, desc ), vm );
    } catch ( const std::exception &ex ) {
        std::cout << desc << std::endl;
        exit ( 1 );
    }
    po::notify ( vm );
    params.shm.clear = ( vm.count ( "clear" ) > 0 );
    params.print_rx = ( vm.count ( "print_rx" ) > 0 );

    if ( vm.count ( "help" ) )  {
        std::cout << desc << std::endl;
        exit ( 1 );
    }

    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = ctrl_handler;
    sigemptyset ( &sigIntHandler.sa_mask );
    sigIntHandler.sa_flags = 0;
    sigaction ( SIGINT, &sigIntHandler, NULL );

    SerialArduinoShm serial_arduino_shm(params);
    while ( loop_main ) {
        sleep ( 8 );
    }
    serial_arduino_shm.close();
    cout << "good-bye!" <<endl;
}
