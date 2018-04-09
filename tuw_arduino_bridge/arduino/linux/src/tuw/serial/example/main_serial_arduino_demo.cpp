
#include <signal.h>
#include <boost/program_options.hpp>
#include <tuw_serial_arduino/serial_arduino.h>

using namespace std;
bool loop_main = true;
void ctrl_handler ( int s ) {
    loop_main = false;
}

struct Parameters {
    static int loop;
    tuw::serial::Parameters serial;
};

tuw::serial::SerialArduino serial_arduino;

void callback ( tuw::serial::Message &header,  tuw::serial::Objects & objects ) {

    tuw::arduino::Pose pose;
    tuw::arduino::Text text;
    std::cout << header << std::endl;
    for ( tuw::serial::Objects::iterator it=objects.begin(); it!=objects.end(); ++it ) {
        tuw::serial::Object &object = it->second;
        switch ( it->first ) {
        case tuw::serial::TYPE_SYNC_REQUEST:
            std::cout << "Sync request" << std::endl;
            break;
        case tuw::serial::TYPE_POSE:
            object.get ( pose );
            std::cout << "Pose: " << pose << std::endl;
            pose.y +=1.5;
            serial_arduino.addObject ( tuw::serial::Object ( pose, tuw::serial::TYPE_POSE ) );
            break;
        case tuw::serial::TYPE_TEXT:
            object.get ( text );
            std::cout << "Text: " << text.txt << std::endl;
            break;
        default:
            std::cout << "Type id: " << object.type << ", of size: " << object.size << std::endl;
        }
    }
}


int main ( int argc, char* argv[] ) {
    namespace po = boost::program_options;

    Parameters params;
    po::options_description desc ( "Allowed Parameters" );
    desc.add_options()
    ( "help", "get this help message" )
    ( "port,m", po::value<std::string> ( &params.serial.port )->default_value ( "/dev/ttyACM0" ), "serial port" )
    ( "baudrate,b", po::value<int> ( &params.serial.baudrate )->default_value ( 115200 ), "baudrate" );

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

    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = ctrl_handler;
    sigemptyset ( &sigIntHandler.sa_mask );
    sigIntHandler.sa_flags = 0;
    sigaction ( SIGINT, &sigIntHandler, NULL );

    auto  callback_fnc ( std::bind ( &callback, std::placeholders::_1,  std::placeholders::_2 ) );
    serial_arduino.init ( params.serial, callback_fnc );
    while ( loop_main ) {
        sleep ( 8 );
    }
    cout<<"good-bye!" <<endl;
}
