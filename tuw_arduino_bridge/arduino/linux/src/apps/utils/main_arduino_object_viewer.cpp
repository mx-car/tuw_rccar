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

#include <boost/program_options.hpp>
#include <tuw_shm/shm_arduino_objects.h>

#include <ncurses.h>


size_t counter;


using namespace std;
bool loop_main = true;

struct Parameters {
    tuw::shm::Parameters shm;
    bool close;
};


Parameters readArgs ( int argc, char **argv ) {
    namespace po = boost::program_options;

    Parameters params;
    po::options_description desc ( "Allowed Parameters" );
    desc.add_options()
    ( "help", "get this help message" )
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
    params.close = false;
    return params;
}

void print_help ( int rows, int row ) {
    move ( row, 0 );
    for ( int r = row; r < rows; r++ ) {
        move ( r, 0 );
        clrtoeol();
    }
    int r = row;
    attron ( COLOR_PAIR ( 1 ) );
    mvprintw ( r++, 5, "'Esc'   for exit" );
    mvprintw ( r++, 5, "'i'     for shm information (NOT WORKING)" );
    mvprintw ( r++, 5, "'d'     row view of containers instead of ; split (NOT WORKING)" );
    mvprintw ( r++, 5, "'h'     for help" );
    mvprintw ( r++, 5, "'a'     toggle auto update (NOT WORKING)" );
    mvprintw ( r++, 5, "'u'     for update (NOT WORKING)" );
    mvprintw ( r++, 5, "any key to continue" );
    getch();
    for ( int r = row; r < rows; r++ ) {
        move ( r, 0 );
        clrtoeol();
    }
}

template <class T> void printObj ( T &obj, int row,  int activeRow ) {
    if ( row == activeRow ) attron ( COLOR_PAIR ( 2 ) );
    else attron ( COLOR_PAIR ( 1 ) );
    mvprintw ( row, 0, "%25s: %s", obj.name().c_str(), obj->getToString().c_str() );
}

int main ( int argc, char *argv[] ) {
    Parameters params = readArgs ( argc, argv );
    tuw::shm::Objects shm ( params.shm );
    initscr();                        // Start curses mode
    raw();                            // Line buffering disabled
    start_color();
    init_pair ( 1, COLOR_WHITE, COLOR_BLACK );
    init_pair ( 2, COLOR_RED, COLOR_BLACK );
    init_pair ( 3, COLOR_GREEN, COLOR_BLACK );
    init_pair ( 4, COLOR_BLUE, COLOR_BLACK );
    init_pair ( 5, COLOR_RED, COLOR_YELLOW );
    init_pair ( 6, COLOR_WHITE, COLOR_YELLOW );
    keypad ( stdscr, TRUE );          // We get F1, F2 etc..

    noecho();
    int key = ' ';
    int rows,cols;
    getmaxyx ( stdscr,rows,cols );
    attron ( COLOR_PAIR ( 3 ) );
    mvprintw ( 0, cols/2-5, "Viewer" );
    attron ( COLOR_PAIR ( 1 ) );
    mvprintw ( 0, cols-20, "press 'h' for help" );
    attron ( COLOR_PAIR ( 1 ) );
    int firstParameterRow = 2;
    char str[80];
    std::string step_size_str;
    int firstRow = 3;
    int activeRow = firstRow;
    int lastRow = rows;
    while ( params.close == false ) {
        std::stringstream message;
        std::stringstream message_error;
        getmaxyx ( stdscr,rows,cols );
        int row = firstRow;
	mvprintw ( activeRow, 0, "-" );
        printObj ( shm.message, row++, activeRow );
        printObj ( shm.pose_estimated, row++, activeRow );
        printObj ( shm.pose_covariance_estimated, row++, activeRow );
        printObj ( shm.ackermann_config, row++, activeRow );
        printObj ( shm.motor_state, row++, activeRow );
        printObj ( shm.motor_pid, row++, activeRow );
        printObj ( shm.servo_state, row++, activeRow );
        printObj ( shm.servo_pid, row++, activeRow );
        printObj ( shm.imu_accelerometer, row++, activeRow );
        printObj ( shm.imu_gyroscope, row++, activeRow );
        printObj ( shm.imu_magnetometer, row++, activeRow );
        printObj ( shm.imu_environment, row++, activeRow );
        printObj ( shm.command_actuators, row++, activeRow );
        printObj ( shm.command_ackermann, row++, activeRow );
	lastRow = row-1;
        timeout ( 100 );
        key = getch();
        if ( key == -1 ) continue;
        timeout ( -1 );
        move ( rows-1, 0 );
        clrtoeol();
        move ( rows-2, 0 );
        clrtoeol();
        switch ( key ) {
        case 'h':
            print_help ( rows, firstParameterRow );
            break;
        case 3: //Ctrl-C
        case 27: //Esc
            params.close = true;
            break;
        case KEY_UP:
            activeRow--;
	    if(activeRow < firstRow) activeRow = firstRow;
            break;
        case KEY_DOWN:
            activeRow++;
	    if(activeRow > lastRow) activeRow = lastRow;
            break;
        }
    }
    endwin();
    exit ( 0 );
    cout<<"good-bye!" <<endl;

}



