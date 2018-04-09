/**
 * @author Markus Bader <markus.bader@tuwien.ac.at>
 * @file bldc.cpp
 * @license Simplified BSD License
 */


#include "bldc.h"
#include "wiring_private.h"
using namespace tuw;

int pin_pwm_signal = 2;


/** state - sensor - value (a+2*b+4*c)
  *   0     a b -  = 3
  *   1     - b -  = 2
  *   2     - b c  = 6
  *   3     - - c  = 4
  *   4     a - c  = 5
  *   5     - - -  = 1
  **/
const int8_t BLDC::sensor_sum2state_[8] = {/*0*/-1, /*1*/5, /*2*/1, /*3*/0, /*4*/3, /*5*/4, /*6*/2, /*7*/-1};
const int8_t BLDC::signal_wave_[3][6] = {/* U - BLUE   */ { 1, 0,-1,-1, 0, 1},
                                        /* V - YELLOW */  { 0, 1, 1, 0,-1,-1},
                                        /* W - RED    */  {-1,-1, 0, 1, 1, 0}
                                        };

BLDC::BLDC() {
    timer_pwm = NULL;
    direction_ = FORWARD;
    rotor_position_ = 0;
    pin_led0_debug_ = 13;
    pin_pwm_debug_ = 11;
    pwm_frq_ = 20000.;
    tick_2_rad_ = 0;
    rps_target_ = 20.;
    rps_ = 0.;
    pwm_ = 0.;
    pwm_ticks_ = 0.;
    motor_ticks_ = 0.;
    timer_pwm_top_value_ = 0;
    timer_pwm_cc_value_ = 0;
    motor_control_trigger_ = 0;
    rps_error_ = 0;
    rps_integral_ = 0;
    rps_derivative_ = 0;
    pwm_output_ = 0;
    pwm_output_last_ = 0;
    motor_controller_.init ( 0.001, 0.0005,  0.0002, -0.9, 0.9 );
}

void hall_sensor_pin_change() {
    BLDC::getInstance().handler_irq_hall_sensor();
}

void TCC0_Handler() {
    BLDC::getInstance().handler_irq_timer_pwm();
}

void BLDC::init ( int hall_a, int hall_b, int hall_c, int pwm_a, int pwm_b, int pwm_c, int dir_a, int dir_b, int dir_c ) {

    pin_hall_[0] = hall_a, pin_hall_[1] = hall_b, pin_hall_[2] = hall_c;
    pin_pwm_[0]  = pwm_a,  pin_pwm_[1]  = pwm_b,  pin_pwm_[2]  = pwm_c;
    pin_dir_[0]  = dir_a,  pin_dir_[1]  = dir_b,  pin_dir_[2]  = dir_c;

    prepare_pins ();
    init_timer_pwm();
    handler_irq_hall_sensor();
}

void BLDC::init_timer_pwm ( ) {

    timer_pwm_top_value_ = 48000000 / 1 / ( uint32_t ) pwm_frq_;
    timer_pwm = ( Tcc* ) TCC0;
    Tcc* TC = timer_pwm;

    GCLK->CLKCTRL.reg = ( uint16_t ) ( GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID ( GCM_TCC0_TCC1 ) ) ;
    while ( GCLK->STATUS.bit.SYNCBUSY == 1 ) ;


    TC->CTRLA.reg &= ~TCC_CTRLA_ENABLE;   // Disable TC
    while ( TC->SYNCBUSY.bit.ENABLE == 1 ); // wait for sync

    TC->CTRLA.reg |= TCC_CTRLA_PRESCALER_DIV1;   // Set perscaler

    TC->WAVE.reg |= TCC_WAVE_WAVEGEN_NFRQ;   // Set wave form configuration
    while ( TC->SYNCBUSY.bit.WAVE == 1 ); // wait for sync

    TC->PER.reg = timer_pwm_top_value_; // = 48000000/1/25000;    // Set counter Top using the PER register
    while ( TC->SYNCBUSY.bit.PER == 1 ); // wait for sync

    // Interrupts
    TC->INTENSET.reg = 0;              // disable all interrupts
    TC->INTENSET.bit.OVF = 1;          // enable overflow
    TC->INTENSET.bit.MC0 = 1;          // enable compare match to CC0
    //TC->INTENSET.bit.MC1 = 1;          // enable compare match to CC0
    //TC->INTENSET.bit.MC2 = 1;          // enable compare match to CC0

    TC->CC[0].reg = timer_pwm_cc_value_;
    while ( TC->SYNCBUSY.bit.CC0 == 1 );
    //TC->CC[1].reg = timer_pwm_top_value_/2;
    //while ( TC->SYNCBUSY.bit.CC1 == 1 );
    //TC->CC[1].reg = timer_pwm_cc_value_;
    //while ( TC->SYNCBUSY.bit.CC1 == 1 );
    //TC->CC[2].reg = timer_pwm_cc_value_;
    //while ( TC->SYNCBUSY.bit.CC2 == 1 );

    // Enable InterruptVector
    NVIC_EnableIRQ ( TCC0_IRQn );

    TC->CTRLA.reg |= TCC_CTRLA_ENABLE ; // Enable TC
    while ( TC->SYNCBUSY.bit.ENABLE == 1 ); // wait for sync

}

void BLDC::handler_irq_timer_pwm() {
    if ( timer_pwm == NULL ) return;
    Tcc *TC = timer_pwm;
    uint32_t clear_flags = 0;
    if ( TC->INTFLAG.bit.OVF == 1 ) { // An overflow caused the interrupt
        clear_flags |= TCC_INTENCLR_OVF;
        PORT->Group[g_APinDescription[pin_pwm_signal].ulPort].OUTSET.reg = ( 1ul << g_APinDescription[pin_pwm_signal].ulPin ) ;
        ticks_since_last_position_++;
        if ( ticks_since_last_position_ > 2000 ) {
            rps_ = 0;
            ticks_since_last_position_ = 0;
            rotor_position_estimate_reset();
        }
        //digitalWrite ( pin_pwm_signal, HIGH );
    }
    if ( TC->INTFLAG.bit.MC0 == 1 ) {
        clear_flags |= TCC_INTENCLR_MC0;
        digitalWrite ( pin_pwm_signal, LOW );
    }
    TC->INTFLAG.reg = clear_flags;
    //digitalWrite ( pin_pwm_debug_, HIGH );
    BLDC::motor_control_trigger_++;
    if ( BLDC::motor_control_trigger_ > 2000 ) {
        BLDC::motor_control_trigger_ = 0;
        BLDC::motor_measure_force();
        BLDC::motor_control_pid();
        
    }
}
void BLDC::motor_measure_force(){
  force_ = 30.4;
  
}

void BLDC::motor_control_pid() {
    if ( fabs ( rps_target_ ) < 5 ) control_state_ = HOLD;
    switch ( control_state_ ) {
    case RESET:
    case INITIALIZE: {
        float pwm_min = ( rps_target_ < 0 ?-0.1:0.1 );
        motor_controller_.reset ( pwm_min, pwm_min );
    }
    break;
    case RUNNING:
        motor_controller_.compute ( rps_target_, rps_ );
        break;
    case HOLD:
        motor_controller_.reset ( 0, 0 );
        break;
    }
    set_pwm ( motor_controller_.output() );
}

void BLDC::prepare_pins () {
    for ( size_t i = 0; i < 3; i++ ) {
        pinMode ( pin_pwm_[i], OUTPUT );
        digitalWrite ( pin_pwm_[i], LOW );
        pinMode ( pin_dir_[i], OUTPUT );
        digitalWrite ( pin_dir_[i], LOW );
        pinMode ( pin_hall_[i], INPUT );
        attachInterrupt ( pin_hall_[i], hall_sensor_pin_change, CHANGE );
    }
    pinMode ( pin_led0_debug_, OUTPUT );
    digitalWrite ( pin_led0_debug_, LOW );
    pinMode ( pin_pwm_debug_, OUTPUT );
    digitalWrite ( pin_pwm_debug_, LOW );
}

void BLDC::rotor_position_estimate_reset () {
    init_state_counter_ = 0;
    control_state_ = RESET;
    digitalWrite ( pin_led0_debug_, LOW );
}

void BLDC::rotor_position_estimate () {
    if ( rotor_position_ == rotor_estimated_ ) {
        if ( init_state_counter_ > ( 2*6 ) ) {
            control_state_ = RUNNING;
            digitalWrite ( pin_led0_debug_, HIGH );
        } else {
            control_state_ = INITIALIZE;
            init_state_counter_++;
            digitalWrite ( pin_led0_debug_, ( ( init_state_counter_/6 ) %2 ) );
        }
    }  else {
        rotor_position_estimate_reset ();
    }
    char txt[0xFF];
    int rotor_next = ( rotor_position_ + direction_ ) % 6;
    if ( rotor_next < 0 ) rotor_next = 5;
#define PRINT_FLOAT2(x) (x<0?"-":" "), ((int) abs(x)), ((int) (abs((x-(int) x)*100.)))
    //sprintf(txt,"r %i, r_e0 %i, r_e1 %i, direction_ %i, rps %s%i.%02d, rps_target %s%i.%02d, pwm %s%i.%02d",
    //        rotor_position_, rotor_estimated_, rotor_next, direction_, PRINT_FLOAT2 ( rps_ ), PRINT_FLOAT2 ( rps_target_ ), PRINT_FLOAT2 ( pwm_ ));
    //Serial.println(txt);
    rotor_estimated_ = rotor_next ;
}

void BLDC::handler_irq_hall_sensor() {
    int a = digitalRead ( pin_hall_[0] ); // read the input pin
    int b = digitalRead ( pin_hall_[1] ); // read the input pin
    int c = digitalRead ( pin_hall_[2] ); // read the input pin
    int sum = ( a + b * 2 + c * 4 ) ;
    rotor_position_ = sensor_sum2state_[sum];

    rotor_position_estimate ();
    ticks_per_position_ = ticks_since_last_position_;
    ticks_since_last_position_ = 0;
    rps_ = ( pwm_frq_*direction_ ) / ( float ) ( ticks_per_position_ * 6 );

    for ( int channel = 0; channel < 3; channel++ ) {
        int signal = signal_wave_[channel][rotor_position_] * direction_;
        if ( signal > 0 ) {
            // pwm
            pin_pwm_signal = pin_pwm_[channel];
            digitalWrite ( pin_dir_[channel], HIGH );
        } else if ( signal < 0 ) {
            // gnd
            digitalWrite ( pin_pwm_[channel], LOW );
            digitalWrite ( pin_dir_[channel], HIGH );
        } else {
            // floating
            digitalWrite ( pin_pwm_[channel], LOW );
            digitalWrite ( pin_dir_[channel], LOW );
        }
    }
}

void BLDC::set_pwm ( float pwm ) {
    if ( pwm > +0.9 ) pwm = +0.9;
    if ( pwm < -0.9 ) pwm = -0.9;
    direction_ = ( pwm < 0. ) ? BACKWARD:FORWARD;
    pwm_ = pwm;
    timer_pwm_cc_value_ = float ( timer_pwm_top_value_ ) * fabs ( pwm_ );
    Tcc *TC = timer_pwm;
    TC->CC[0].reg = timer_pwm_cc_value_;
    while ( TC->SYNCBUSY.bit.CC0 == 1 );
}

void BLDC::set_rps ( float rps ) {
    rps_target_ = rps;
}

float BLDC::get_rps() {
    return rps_;
}

float BLDC::get_pwm() {
    return pwm_;
}

float BLDC::get_rps_target() {
    return rps_target_;
}

float BLDC::get_force() {
  return force_;
}

int BLDC::get_state() {
    return control_state_;
}

int BLDC::get_rotor_position() {
    return rotor_position_;
}

int BLDC::get_rotor_estimated() {
    return rotor_estimated_;
}

int BLDC::debug_msg ( char *txt ) {
#define PRINT_FLOAT4(x) (x<0?"-":" "), ((int) abs(x)), ((int) (abs((x-(int) x)*10000.)))
#define PRINT_FLOAT2(x) (x<0?"-":" "), ((int) abs(x)), ((int) (abs((x-(int) x)*100.)))
    sprintf ( txt, "$s%s%i.%02d, %s%i.%02d, %s%i.%02d\n", PRINT_FLOAT2 ( pwm_ ), PRINT_FLOAT2 ( rps_ ), PRINT_FLOAT2 ( rps_target_ ) );
    return strlen ( txt );
}
