#ifndef PID_H
#define PID_H

    typedef struct {
        /* Tunable parameters */
        float kp;
        float ki;
        float kd;

        float dt;            // control period (s)

        /* State */
        float error;
        float error_last;
        float integral;

        /* Output */
        float out;

        /* Limits */
        float integral_limit;
        float output_limit;
    } PID_t;

    /* Initialize PID controller */
    void PID_Init(PID_t *pid,
                  float kp, float ki, float kd,
                  float dt,
                  float integral_limit,
                  float output_limit);

    /* Reset internal state (important for safety & mode switch) */
    void PID_Reset(PID_t *pid);

    /* Compute PID output */
    float PID_Update(PID_t *pid, float target, float measurement);

#endif /* PID_H */
