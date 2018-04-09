#ifndef _PID_H_
#define _PID_H_
namespace tuw {
class PID {
public:
    PID (  );
    void init(float kp = 0, float ki = 0, float kd = 0, float output_min = 0, float output_max = 0);
    float compute ( float target, float measurment );
    float &Kp() {
        return kp_;
    };
    float &Ki() {
        return ki_;
    };
    float &Kd() {
        return kd_;
    };
    const float &Kp() const {
        return kp_;
    };
    const float &Ki() const {
        return ki_;
    };
    const float &Kd() const {
        return kd_;
    };
    void reset (float integral, float output) {
        output_now_ = output;
        output_last_ = output;
        integral_ = integral;
    };
    const float output () const {
        return output_now_;
    };
private:
    float kp_;
    float ki_;
    float kd_;
    float output_min_, output_max_;
    float error_;
    float output_now_, output_last_;
    float measurment_now_, measurment_last_;
    float target_;
    float proportional_;
    float integral_;
    float derivative_;
};
};
#endif

