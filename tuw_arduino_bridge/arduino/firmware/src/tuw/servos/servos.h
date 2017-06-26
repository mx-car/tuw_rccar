/**
 * @author Markus Bader <markus.bader@tuwien.ac.at>
 * @brief Simple servo driver for an arduino zero
 * @file servos.h
 * @license Simplified BSD License
 */


#ifndef _SERVOS_H_
#define _SERVOS_H_

#include <Arduino.h>

namespace tuw {


/**
 * Class to simplify servo commands
 * works on Arduino Zero
 * @author Markus Bader <markus.bader@tuwien.ac.at>
 **/
class Servos {
    static const int32_t counter_ticks_per_micro_seconds;      /// ticks per micro (0.000001) second

    Tcc* TC;                                                   /// Tcc timer to use
    int32_t servo_frq_cycle_micro_seconds;                     /// servo frq cyle in micros (0.000001) second
    uint32_t pin_output[4];                                    /// output pin for servo 0, values < 0 means inactive
    int32_t servo_compare_value[4];                            /// servo neutral pose
    int32_t servo_neutral_signal_micro_seconds[4];             /// servo neutral pose
    float servo_counter_ticks_per_rad[4];


    /**
     * update the compare register
     * @param channel compare channel to update
     **/
    void update ( uint32_t channel );

    Servos();
public:

    static Servos &getInstance( ){
      static Servos instance;
      return instance;
    }
    /**
     * @brief inits a timer with the frq defined by
     * @param timer to use
     */
    void init ( Tcc* timer );
    /**
     * @brief enables and attaches a pin to the timer
     * @param pin servo pin to update
     */
    void enable ( uint32_t pin );
    /**
     * @brief set pins as outputs and to low
     * @param pin servo pin to disable
     */
    void disable ( uint32_t pin );

    /**
     * @return number of counter ticks per micro second
     **/
    int32_t getTicksPerMicroSecond();
    /**
     * To change the servo cycle time
     * @param micro_seconds defines the servos cycle time
     **/
    void setCycleTime ( int32_t micro_seconds = 20000);
    /**
     * returns the current cycle time
     * @return cycle time in micro_seconds
     **/
    int32_t getCycleTime();
    /**
     * sets the scale to set a servo angle
     * @param micro_seconds defines us per rad  
     **/
    void setMicroSecondsPerRad ( int32_t micro_seconds, uint32_t pin);
    /**
     * returns the current scale to set a servo angle
     * @return us per rad
     **/
    int32_t getMicroSecondsPerRad(uint32_t pin);
    /**
     * sets the neutral pose
     * @param micro_seconds high time in micro_seconds (us)
     * @param pin servo pin to update
     **/
    void setNeutral ( int32_t micro_seconds, uint32_t pin );
    /**
     * sets the neutral pose
     * @param micro_seconds high time in micro_seconds (us)
     * @param pin servo pin to read
     **/
    int32_t getNeutral ( int32_t micro_seconds, uint32_t pin );
    /**
     * sets an angle on the selected servo in degrees
     * @param deg angle 0 means neutral pose
     * @param pin servo pin to update
     **/
    void setDegrees ( float deg, uint32_t pin );
    /**
     * sets an angle on the selected servo in rad
     * @param deg angle 0 means neutral pose
     * @param pin servo pin to update
     **/
    void setRad ( float rad, uint32_t pin );
    /**
     * set the high time for a servo in us
     * @param micro_seconds high time in micro_seconds (us)
     * @param pin servo pin to update
     **/
    void setHighTime ( int32_t micro_seconds, uint32_t pin );
    /**
     * returns the high time in us
     * @param pin servo pin to read
     * @return high time in us
     **/
    int32_t getHighTime ( uint32_t pin );
    /**
     * returns an angle value to the selected servo in degrees
     * @param pin servo pin to read
     * @return angle in degrees
     **/
    float getDegrees ( uint32_t pin );
    /**
     * returns an angle value to the selected servo in rad
     * @param pin servo pin to read
     * @return angle in rad
     **/
    float getRad ( uint32_t pin );

    friend void TCC1_Handler ( void ); /// friend to allow access
};

}
#endif
