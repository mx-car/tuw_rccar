/**
 * @author Markus Bader <markus.bader@tuwien.ac.at>
 * @license Simplified BSD License
 */

#include "serial_message.h"

using namespace tuw::serial;

Time Time::OFFSET ( 0,0 );
bool Time::CLOCK_SYNC ( false );
uint32_t Message::tx_count = 0;
uint32_t Message::rx_count = 0;
uint16_t Message::sync_count = 0;


Time::Time() {
    set ( 0,0 );
}
Time::Time ( int32_t sec, int32_t nsec ) {
    set ( sec,nsec );
}
void Time::setClock ( const TTime &now) {
  setClock ( now, millis() );
}
void Time::setClock ( const TTime &now, uint32_t millisecond ) {
    int32_t s = ( millisecond / 1000 );
    int32_t n = ( millisecond % 1000UL ) * 1000UL * 1000UL;
    OFFSET.sec = now.sec - s;
    OFFSET.nsec = now.nsec - n;
    if ( OFFSET.nsec < 0 ) {
        OFFSET.nsec = ( 1000UL * 1000UL * 1000UL ) + OFFSET.nsec;
        OFFSET.sec--;
    }
    CLOCK_SYNC = true;
}
void Time::set ( int32_t sec, int32_t nsec ) {
    this->sec = sec, this->nsec = nsec;
}
void Time::now ( ) {
    return now ( millis() );
}
void Time::now ( uint32_t millisecond ) {
    int32_t s = millisecond / 1000;
    int32_t m = millisecond % 1000;
    int32_t n = OFFSET.nsec + m * ( 1000UL * 1000UL );
    sec = OFFSET.sec + s + n / ( 1000UL * 1000UL * 1000UL );
    nsec = n % ( 1000UL * 1000UL * 1000UL );
}
Time Time::offest() {
    return OFFSET;
}
bool Time::isSet() {
    return CLOCK_SYNC;
}

Time &Message::time() {
    return ( Time & ) this->stamp;
}
const Time &Message::time() const {
    return ( Time & ) this->stamp;
}

void Message::writeByte(uint8_t bt) {
    if (bt == 0x00) {
        serialbuffer[serialbuffer_used++] = 0x02;
        serialbuffer[serialbuffer_used++] = 0x30;
    } else if (bt == 0x01) {
        serialbuffer[serialbuffer_used++] = 0x02;
        serialbuffer[serialbuffer_used++] = 0x31;
    } else if (bt == 0x02) {
        serialbuffer[serialbuffer_used++] = 0x02;
        serialbuffer[serialbuffer_used++] = 0x32;
    } else if (bt == 0x04) {
        serialbuffer[serialbuffer_used++] = 0x02;
        serialbuffer[serialbuffer_used++] = 0x34;
    } else {
        serialbuffer[serialbuffer_used++] = bt;
    }
}

int8_t Message::readByte(uint8_t* bt) {
    uint8_t temp;
    temp = serialbuffer[serialbuffer_used++];

    if (temp == 0x00) {
        // assert(false); // Start Delimiter
    } else if (temp == 0x01) {
        // assert(false); // End Delimiter
    } else if (temp == 0x02) {
        temp = serialbuffer[serialbuffer_used++];
        if (temp == 0x30) *bt = 0x00;
        if (temp == 0x31) *bt = 0x01;
        if (temp == 0x32) *bt = 0x02;
        if (temp == 0x34) *bt = 0x04;
        return 0;
	// assert(false);
    } else {
        *bt = temp;
        return 0;
    }
}


int Message::send () {
    int total = 0;
    if ( Serial ) {
	this->seq = tx_count;
        serialbuffer_used = 0;
        char *c = ( char * ) this;
        serialbuffer[serialbuffer_used++] = 0x00;  // Start Delimiter
        for ( int i = 0; i < sizeof ( TMessageHeader ); i++ ) {
            writeByte( (uint8_t) *(c++) );
        }
        for ( int i = 0; i < this->size; i++ ) {
            writeByte( (uint8_t) buffer[i] );
        }
        serialbuffer[serialbuffer_used++] = 0x01;  // End Delimiter



//        Serial.write((const char*) serialbuffer, (size_t) serialbuffer_used);
        for (int i = 0; i < serialbuffer_used; i++) {
            Serial.write((char) serialbuffer[i]);
        }


        Serial.flush();
        tx_count++;
    }
    return sizeof ( TMessageHeader ) + this->size;
}

int Message::receive() {
    while ( 1 ) {
        uint16_t temp;
        if ( Serial.available() ) {
            temp = Serial.read();
            if (temp == 0x00) { // Start Delimiter
                serialbuffer_used = 0;
            } else if (temp == 0x01) {
                break; // End Delimiter -> continue with parsing
            } else {
                // assert(serialbuffer_used < MAX_BUFFER_SIZE);
                serialbuffer[serialbuffer_used++] = temp;
            }
        } else {
            return 0; // Currently no more bytes to read
        }
    }

    reset();
    int8_t rc;
    uint8_t rdByte;
    int rx_count = 0;

    serialbuffer_used = 0; // Used for reading

    char *c = ( char* ) this;
    for ( uint16_t i = 0; i < sizeof ( TMessageHeader ); i++ ) {
        readByte(&rdByte);
        *c++ = rdByte;
        rx_count++;
    }

    if ( rx_count == sizeof ( TMessageHeader ) ) {
        for ( uint16_t i = 0; i < this->size; i++ ) {
            readByte((uint8_t*) &this->buffer[i]);
            rx_count++;
        }
    }
    rx_count++;

    stack_pointer = 0;
    return rx_count;
}

void Message::try_sync() {
    if ( Time::isSet() ) return;
    Object object;
    for ( int i = 0; Time::isSet() == false; i++ ) {
        if ( i%10 == 0 ) {
            this->reset();
            this->push_object ( Object ( TYPE_SYNC_REQUEST ) );
            this->send();
        };
        if ( receive() ) {      /// check for messages
            while ( this->pop_object ( object ).isValid() ) {
                if ( object.type == TYPE_SYNC ) {
                    Time::setClock ( stamp); /// set clock
                }
            }
        }
        delay ( 100 );
    }
}
