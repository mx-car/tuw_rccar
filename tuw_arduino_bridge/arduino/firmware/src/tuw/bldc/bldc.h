/**
 * @author Markus Bader <markus.bader@tuwien.ac.at>
 * @license Simplified BSD License
 */


#ifndef BLDC_H
#define BLDC_H

#include <Arduino.h>
#include "pid.h"

#define PRINT_FLOAT4(x) (x<0?"-":" "), ((int) abs(x)), ((int) (abs((x-(int) x)*10000.)))
#define PRINT_FLOAT2(x) (x<0?"-":" "), ((int) abs(x)), ((int) (abs((x-(int) x)*100.)))

namespace tuw{
class BLDC {
public:
    typedef enum {RESET = 0, INITIALIZE = 1, RUNNING = 2, HOLD = 3} CONTROL_STATE;
private:
    static const int FORWARD  = +1;
    static const int BACKWARD = -1;
    static const int8_t signal_wave_[3][6];
    static const int8_t sensor_sum2state_[8];
    int pin_hall_[3];      /// hall sensor pins
    int pin_pwm_[3];       /// pwm output pins
    int pin_dir_[3];       /// direction control output pins
    int direction_;         /// current rotation direction

    int pin_pwm_debug_;    /// debug pwm pin
    int pin_led0_debug_;   /// debug led pin
    Tcc* timer_pwm;        /// Tcc timer to use
    float pwm_frq_;        /// pwm frequency in Hz
    uint32_t pwm_ticks_;   /// pwm frequency in timer ticks
    uint32_t ticks_per_position_; /// pwm ticks per rotor position
    uint32_t ticks_since_last_position_; /// pwm ticks since lase rotor position
    float rps_target_;      /// motor rotation in Hz
    uint32_t motor_ticks_; /// motor rotation in timer ticks
    float tick_2_rad_;     /// scale to convert timer ticks to rad for phase control
    uint8_t rotor_position_;  /// rotor position_ base on hall sensors
    uint32_t timer_pwm_top_value_; /// timer top value (it defines with the prescaler the pwm frq)
    uint32_t timer_pwm_cc_value_;  /// maximum pwm value
    uint32_t motor_control_trigger_;
    float rps_;
    float pwm_;
    float force_;
    float rps_error_;
    float rps_integral_;
    float rps_derivative_;
    float pwm_output_;
    float pwm_output_last_;
    CONTROL_STATE control_state_;
    int init_state_counter_;
    int rotor_estimated_;
    BLDC();
    void rotor_position_estimate ();
    void rotor_position_estimate_reset ();
    void prepare_pins ();           /// prepares the pins used
    void init_timer_pwm();          /// initializes the timer
    void motor_control_pid();
    void motor_measure_force();
    PID  motor_controller_;
public:

    static BLDC &getInstance( ){
        static BLDC instance;
        return instance;
    }
    /**
     * sets the maximum pwm signal
     * @param v value between 0 and 1
     **/
    void set_pwm(float v);
    /**
     * returns the current pwm signal
     * @return value between 0 and 1
     **/
    float get_pwm();
    /**
     * sets the target rotation/second
     * @param rps target rotation
     **/
    void set_rps( float rps );
    /**
     * returns motor state
     * @return motor state
     **/
    int get_state();
    /**
     * returns rotor postion
     * @return rotor postion
     **/
    int get_rotor_position();
    /**
     * returns rotor estimate
     * @return rotor estimate
     **/
    int get_rotor_estimated();
    /**
     * returns the rotation pers second
     * @return moter rotation frquency in Hz
     **/
    float get_rps();
    /**
     * returns the current target
     * @return moter rotation frquency in Hz
     **/
    float get_rps_target();
    /**
     * returns the measured force
     * @return force
     **/
    float get_force();
    /**
     * irq hander for pwm timer
     **/
    void handler_irq_timer_pwm();
    /**
     * irq hander for hall sensors
     **/
    void handler_irq_hall_sensor();
    /**
     * init system
     * @param hall_a pin
     * @param hall_b
     * @param hall_c
     * @param pwm_a
     * @param pwm_b
     * @param pwm_c
     * @param dir_a
     * @param dir_b
     * @param dir_c
     **/
    void init (int hall_a, int hall_b, int hall_c, int pwm_a, int pwm_b, int pwm_c, int dir_a, int dir_b, int dir_c);
    /**
     * irq hander for hall sensors
     **/
    int debug_msg(char *txt);

    PID &pid(){
        return motor_controller_;
    }
};

}
#endif

