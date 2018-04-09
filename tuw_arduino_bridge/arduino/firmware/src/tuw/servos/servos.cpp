/**
 * @author Markus Bader <markus.bader@tuwien.ac.at>
 * @brief Simple servo driver for an arduino zero to controll
 * three servos with timer 1
 * @file servos.cpp
 * @license Simplified BSD License
 */


#include "servos.h"

using namespace tuw;


#ifdef F_CPU
/// 1us on f_clk = 48Mhz and prescaler = 16 : ( 0.000001 / (1.0/48000000*16) )
const int32_t Servos::counter_ticks_per_micro_seconds = F_CPU / ( 16*1000000 );
#endif

Servos::Servos() {
    TC = NULL;
    servo_frq_cycle_micro_seconds = 20000;
    for ( unsigned int i = 0; i < 4; i++ ) {
        pin_output[i] = -1;
        servo_neutral_signal_micro_seconds[i] = 1500;
        servo_counter_ticks_per_rad[i] = ( 48000000/1000 ) / ( 16.0 * M_PI );
        servo_compare_value[i] = servo_neutral_signal_micro_seconds[i] * counter_ticks_per_micro_seconds;
    }
}

void Servos::init ( Tcc* timer ) {

    TC = timer;

    // Enable GCLK
    if ( TC == ( Tcc* ) TCC0 ) {
        GCLK->CLKCTRL.reg = ( uint16_t ) ( GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID ( GCM_TCC0_TCC1 ) ) ;
    } else if ( TC == ( Tcc* ) TCC1 ) {
        GCLK->CLKCTRL.reg = ( uint16_t ) ( GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID ( GCM_TCC0_TCC1 ) ) ;
    } else if ( TC == ( Tcc* ) TCC1 ) {
        GCLK->CLKCTRL.reg = ( uint16_t ) ( GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID ( GCM_TCC2_TC3 ) ) ;
    } else {
        TC = NULL;
        return;
    }
    while ( GCLK->STATUS.bit.SYNCBUSY == 1 ) ;


    TC->CTRLA.reg &= ~TCC_CTRLA_ENABLE;   // Disable TC
    while ( TC->SYNCBUSY.bit.ENABLE == 1 ); // wait for sync

    TC->CTRLA.reg |= TCC_CTRLA_PRESCALER_DIV16;   // Set perscaler

    TC->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM;   // Set wave form configuration
    while ( TC->SYNCBUSY.bit.WAVE == 1 ); // wait for sync

    TC->PER.reg = servo_frq_cycle_micro_seconds*counter_ticks_per_micro_seconds;    // Set counter Top using the PER register
    while ( TC->SYNCBUSY.bit.PER == 1 ); // wait for sync

    TC->CTRLA.reg |= TCC_CTRLA_ENABLE ; // Enable TC
    while ( TC->SYNCBUSY.bit.ENABLE == 1 ); // wait for sync

    return;
}

void Servos::setCycleTime ( int32_t micro_seconds ) {
    servo_frq_cycle_micro_seconds = micro_seconds;
    if ( TC != NULL ) {
        init ( TC );
    }
}

int32_t Servos::getCycleTime ( ) {
    return servo_frq_cycle_micro_seconds;
}

int32_t Servos::getTicksPerMicroSecond() {
    return counter_ticks_per_micro_seconds;
}

void Servos::enable ( uint32_t pin ) {
    //set pins as outputs
    pinPeripheral ( pin, g_APinDescription[pin].ulPinType ) ;
    uint8_t channel = GetTCChannelNumber ( g_APinDescription[pin].ulPWMChannel ) ;
    update ( channel );
}

void Servos::disable ( uint32_t pin ) {
    //set pins as outputs and to low
    pinMode ( pin, OUTPUT ) ;
    digitalWrite ( pin, LOW ) ;
}

void Servos::setNeutral ( int32_t micro_seconds, uint32_t pin ) {
    uint8_t channel = GetTCChannelNumber ( g_APinDescription[pin].ulPWMChannel ) ;
    int32_t offset_ticks = servo_compare_value[channel] - servo_neutral_signal_micro_seconds[channel]*counter_ticks_per_micro_seconds;
    servo_neutral_signal_micro_seconds[channel] = micro_seconds;
    servo_compare_value[channel] = servo_neutral_signal_micro_seconds[channel] * counter_ticks_per_micro_seconds + offset_ticks;
    update ( channel );
}

int32_t Servos::getNeutral ( int32_t micro_seconds, uint32_t pin ) {
    uint8_t channel = GetTCChannelNumber ( g_APinDescription[pin].ulPWMChannel ) ;
    return servo_neutral_signal_micro_seconds[channel];
}

void Servos::setHighTime ( int32_t micro_seconds, uint32_t pin ) {
    uint8_t channel = GetTCChannelNumber ( g_APinDescription[pin].ulPWMChannel ) ;
    servo_compare_value[channel] = micro_seconds * counter_ticks_per_micro_seconds;
    update ( channel );
}

void Servos::setRad ( float rad, uint32_t pin ) {
    uint8_t channel = GetTCChannelNumber ( g_APinDescription[pin].ulPWMChannel ) ;
    int32_t offset = servo_counter_ticks_per_rad[channel] * rad;
    int32_t base = servo_neutral_signal_micro_seconds[channel] * counter_ticks_per_micro_seconds;
    servo_compare_value[channel] = offset + base;
    update ( channel );
}

void Servos::setDegrees ( float deg, uint32_t pin ) {
    uint8_t channel = GetTCChannelNumber ( g_APinDescription[pin].ulPWMChannel ) ;
    setRad ( ( deg * M_PI ) / 180.0, pin );
}

int32_t Servos::getHighTime ( uint32_t pin ) {
    uint8_t channel = GetTCChannelNumber ( g_APinDescription[pin].ulPWMChannel ) ;
    return servo_compare_value[channel] / counter_ticks_per_micro_seconds;
}

float Servos::getRad ( uint32_t pin ) {
    uint8_t channel = GetTCChannelNumber ( g_APinDescription[pin].ulPWMChannel ) ;
    int32_t base = servo_neutral_signal_micro_seconds[channel] * counter_ticks_per_micro_seconds;
    float offset = servo_compare_value[channel] - base;
    return offset / servo_counter_ticks_per_rad[channel];
}

float Servos::getDegrees ( uint32_t pin ) {
    uint8_t channel = GetTCChannelNumber ( g_APinDescription[pin].ulPWMChannel ) ;
    return getRad ( pin ) * ( 180.0 / M_PI );
}

void Servos::setMicroSecondsPerRad ( int32_t micro_seconds,  uint32_t pin ) {
    uint8_t channel = GetTCChannelNumber ( g_APinDescription[pin].ulPWMChannel ) ;
    servo_counter_ticks_per_rad[channel] = micro_seconds * counter_ticks_per_micro_seconds;
}

int32_t Servos::getMicroSecondsPerRad ( uint32_t pin ) {
    uint8_t channel = GetTCChannelNumber ( g_APinDescription[pin].ulPWMChannel ) ;
    return servo_counter_ticks_per_rad[channel] / counter_ticks_per_micro_seconds;
}

void Servos::update ( uint32_t channel ) {
    TC->CC[channel].reg = servo_compare_value[channel];
    switch ( channel ) {
    case 0:
        while ( TC->SYNCBUSY.bit.CC0 == 1 );
        break;
    case 1:
        while ( TC->SYNCBUSY.bit.CC1 == 1 );
        break;
    case 2:
        while ( TC->SYNCBUSY.bit.CC2 == 1 );
        break;
    case 3:
        while ( TC->SYNCBUSY.bit.CC3 == 1 );
        break;
    };
}
