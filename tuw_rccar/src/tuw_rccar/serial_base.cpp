#include "tuw_rccar/serial_base.h"

using namespace tuw::serial;



TObjectHeader::TObjectHeader()
    : type ( 0 )
    , size ( 0 ) {};
TObjectHeader::TObjectHeader ( const TObjectHeader &o )
    : type ( o.type ), size ( o.size ) {};
TObjectHeader::TObjectHeader ( ObjectType type, int16_t size )
    : type ( type )
    , size ( size ) {};



TObject::TObject()
    : TObjectHeader()
    , buffer ( NULL ) {};
TObject::TObject ( ObjectType type )
    : TObjectHeader ( type, 0 )
    , buffer ( NULL ) {};
TObject::TObject ( const TObject &o )
    : TObjectHeader ( o )
    , buffer ( NULL )  {
    if ( ( o.buffer != NULL ) && ( this->size > 0 ) ) {
        alloc ( this->size );
        copy_buffer_from ( o.buffer );
    }
};
TObject::~TObject () {
    dealloc();
};

void TObject::alloc ( size_t size ) {
    dealloc();
    if ( size > 0 ) buffer = ( char* ) malloc ( size );
}
void TObject::dealloc() {
    if ( buffer != NULL ) free ( buffer );
    buffer = NULL;
}
void TObject::copy_buffer_from ( const char *src ) {
    if ( buffer != NULL ) {
        for ( std::size_t i = 0; i < this->size; i++ ) {
            buffer[i] = src[i];
        }
    }
}

TObject& TObject::empty() {
    this->type = TYPE_EMPTY;
    this->size = 0;
    dealloc();
    return *this;
}
TObject& TObject::error() {
    this->type = TYPE_ERROR;
    this->size = 0;
    dealloc();
    return *this;
}
bool TObject::isValid() {
    return ! ( ( this->type == TYPE_ERROR ) || ( this->size < 0 ) );
}
unsigned int TObject::deserialize ( const char *data, unsigned int len ) {
    if ( len <  sizeof ( TObjectHeader ) ) {
        this->error();
        return len;
    }
    * ( ( TObjectHeader * ) this )  = * ( ( TObjectHeader * ) data );
    if ( len-sizeof ( TObjectHeader ) < this->size ) {
        this->error();
        return len;
    }
    alloc ( this->size );
    copy_buffer_from ( data + sizeof ( TObjectHeader ) );
    return  this->size + sizeof ( TObjectHeader );
}
TObject& TObject::operator= ( const TObject &o ) {
    this->size = o.size;
    this->type = o.type;
    alloc ( this->size );
    copy_buffer_from ( o.buffer );
}

TMessageHeader::TMessageHeader()
    : size ( 0 )
    , seq ( 0 )
    , stamp() {};
TMessageHeader::TMessageHeader ( uint16_t size, uint32_t seq, const tuw::arduino::TTime &stamp )
    : size ( size )
    , seq ( seq )
    , stamp ( stamp ) {};

void TMessageHeader::zeros() {
    size = 0, seq = 0, stamp.sec = 0, stamp.nsec = 0;
}

TMessage::TMessage()
    : TMessageHeader()
    , stack_pointer ( 0 ) {};

bool TMessage::isValid() {
    return ( this->size < MAX_BUFFER_SIZE );
}
void TMessage::reset() {
    this->zeros();
    stack_pointer = 0;
    memset ( buffer, '\0', MAX_BUFFER_SIZE );
}

unsigned int TMessage::deserialize ( const char *data, unsigned int len ) {
    stack_pointer = 0;
    if ( len <  sizeof ( TMessageHeader ) ) return -1;
    * ( ( TMessageHeader * ) this )  = * ( ( TMessageHeader * ) data );
    for ( std::size_t i = 0; i < this->size; i++ ) {
        buffer[i] = data[sizeof ( TMessageHeader ) +i];
    }
    return this->size + sizeof ( TMessageHeader );
}

bool TMessage::push_sync () {
    if ( this->size + sizeof ( TObjectHeader ) <= MAX_BUFFER_SIZE ) {
        TObjectHeader *object_header = ( TObjectHeader* )  buffer + this->size;
        object_header->type = TYPE_SYNC;
        object_header->size = 0;
        this->size += sizeof ( TObjectHeader );
        return true;
    } else {
        return false;
    }
}
TObject &TMessage::pop_object ( TObject &object ) {
    if ( stack_pointer < this->size ) {
        stack_pointer+= object.deserialize ( &buffer[stack_pointer], this->size - stack_pointer );
    } else {
        object.error();
    }
    return object;
}

TMessage &TMessage::push_object ( const TObject &object ) {
    if ( this->size + sizeof ( TObjectHeader ) +  object.size <= MAX_BUFFER_SIZE ) {
        memcpy ( buffer + this->size, &object, sizeof ( TObjectHeader ) );
        this->size += sizeof ( TObjectHeader );
        memcpy ( buffer + this->size, object.buffer, object.size );
        this->size += object.size;
    }
    return *this;
}
