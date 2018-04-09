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

#include <tuw_shm/shm_arduino_object_handler.h>

using namespace tuw::shm;

HandlerObjectPtr  HandlerObject::open ( const std::string &name, ShmFw::HandlerPtr &shmHdl ) {

    if ( shmHdl->findName ( name ) == NULL ) {
        std::cerr << "no shared variable with name: " << name << std::endl;
        throw 0;
    }
    ShmFw::Header shmHeader ( shmHdl, name );

#define RETURN_IF_TYPE_VAR( TYPE )  if(shmHeader.isType<ShmFw::Var< TYPE > >()) return HandlerObjectPtr( new HandlerVar< TYPE >(name, shmHdl));

    RETURN_IF_TYPE_VAR ( tuw::arduino::Text );
    RETURN_IF_TYPE_VAR ( tuw::arduino::Pose );
    RETURN_IF_TYPE_VAR ( tuw::arduino::Mat3x3 );
    RETURN_IF_TYPE_VAR ( tuw::arduino::AckermannConfig );
    RETURN_IF_TYPE_VAR ( tuw::arduino::Motor );
    RETURN_IF_TYPE_VAR ( tuw::arduino::PID );
    RETURN_IF_TYPE_VAR ( tuw::arduino::Actuators );
    return HandlerObjectPtr();
}
HandlerObjectPtr HandlerObject::create ( const std::string &name, ShmFw::HandlerPtr &shmHdl, const std::string &type ) {

#define CREATE_TYPE_VAR( TYPE ) if(boost::iequals(type, "TYPE")) return HandlerObjectPtr( new HandlerVar< TYPE >(name, shmHdl));
    if ( shmHdl->findName ( name ) != NULL ) {
        std::cerr << "variable exists allready!" << name << std::endl;
        return open ( name, shmHdl );
    }

    CREATE_TYPE_VAR ( tuw::arduino::Text );
    CREATE_TYPE_VAR ( tuw::arduino::Pose );
    CREATE_TYPE_VAR ( tuw::arduino::Mat3x3 );
    CREATE_TYPE_VAR ( tuw::arduino::AckermannConfig );
    CREATE_TYPE_VAR ( tuw::arduino::Motor );
    CREATE_TYPE_VAR ( tuw::arduino::PID );
    CREATE_TYPE_VAR ( tuw::arduino::Actuators );
    return HandlerObjectPtr();
}




