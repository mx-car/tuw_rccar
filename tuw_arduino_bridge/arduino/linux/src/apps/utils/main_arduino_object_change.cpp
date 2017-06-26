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
#include <iomanip>

#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */


#include "shmfw/variable.h"
#include "shmfw/handler.h"
#include "tuw_shm/shm_arduino_object_handler.h"

#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>

size_t counter;
size_t activeParameter;

using namespace std;

struct Prarmeters {
    string shm_memory_name;
    unsigned int shm_memory_size;
    string variable_name;
    string value_string;
    bool lock;
    bool unlock;
    bool quiet;
    bool close;
};

Prarmeters readArgs ( int argc, char **argv ) {
    namespace po = boost::program_options;

    Prarmeters params;
    po::options_description desc ( "Allowed Parameters" );
    desc.add_options()
    ( "help,h", "get this help message" )
    ( "quiet,q", "no output" )
    ( "lock,l", "lock" )
    ( "unlock,u", "unlock" )
    ( "value,v", po::value<string> ( &params.value_string ), "shared variable value string" )
    ( "variable_name,n", po::value<string> ( &params.variable_name ), "shared variable name" )
    ( "shm_memory_name", po::value<string> ( &params.shm_memory_name )->default_value ( ShmFw::DEFAULT_SEGMENT_NAME() ), "shared memory segment name" )
    ( "shm_memory_size", po::value<unsigned int> ( &params.shm_memory_size )->default_value ( ShmFw::DEFAULT_SEGMENT_SIZE() ), "shared memory segment size" );

    po::variables_map vm;
    try {
        po::store ( po::parse_command_line ( argc, argv, desc ), vm );
    } catch ( const exception &ex ) {
        cout << desc << endl;;
        exit ( 1 );
    }
    po::notify ( vm );

    if ( vm.count ( "help" ) )  {
        cout << desc << endl;
        exit ( 1 );
    }
    if ( vm.count ( "variable_name" ) == 0 )  {
        cout << desc << endl;
        exit ( 1 );
    }
    params.quiet = ( vm.count ( "quiet" ) > 0 );
    params.unlock = ( vm.count ( "unlock" ) > 0 );
    params.lock = ( vm.count ( "lock" ) > 0 );

    return params;
}

int main ( int argc, char *argv[] ) {
    Prarmeters params = readArgs ( argc, argv );
    params.close = false;

    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( params.shm_memory_name, params.shm_memory_size );

    if ( shmHdl->findName ( params.variable_name ) == NULL ) {
        cerr << RED << "No variable: " << params.variable_name ;
        cerr << " in share segment " << params.shm_memory_name;
        cerr << "!" <<  RESET << endl;
        return 0;
    }
    tuw::shm::HandlerObjectPtr obj = tuw::shm::HandlerObject::open ( params.variable_name, shmHdl );
    if ( !params.quiet ) cout << obj->name() << endl;
    if ( !params.quiet ) cout << obj->timestamp() << ( obj->locked() ? ", locked  " : ", unlocked" ) << endl;
    if ( !params.quiet ) cout << GREEN << " = " << obj->value() << RESET << endl;
    if ( params.unlock ) obj->unlock();
    if ( params.lock ) obj->lock();
    if ( !params.value_string.empty() ) {
        if ( !params.quiet )  cout << CYAN << " = " << params.value_string << RESET << endl;
        if ( !obj->locked() ) {
            obj->lock();
            obj->value ( params.value_string );
            obj->it_has_changed();
            obj->unlock();
        } else {
            cerr << RED << "variable is locked!"<< RESET << endl;
        }
        if ( !params.quiet ) cout << YELLOW << " = " << obj->value() << RESET << endl;
        if ( !params.quiet ) cout << obj->timestamp() << ( obj->locked() ? ", locked  " : ", unlocked" ) << endl;
    }
}



