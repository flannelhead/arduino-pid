#ifndef PID_H
#define PID_H

#include <stdint.h>

#define CLAMP(x, m, M) (((x) < (M)) ? (((x) > (m)) ? (x) : (m)) : (M))

#define PID_DIVIDER 128

typedef struct {
    int16_t Kp, Ki, Kd;
    int16_t outMin, outMax;
    int32_t maxIntegral;

    int16_t lastProcessValue;
    int32_t integral;

    int16_t *setPoint;
    int16_t *output;
    int16_t *processValue;
} pidparams;

void pidInit(pidparams * const params, int16_t * const setPoint,
    int16_t * const processValue, int16_t * const output,
    int16_t Kp, int16_t Ki, int16_t Kd,
    int16_t outMin = INT16_MIN, int16_t outMax = INT16_MAX) {
    params->Kp = Kp;
    params->Ki = Ki;
    params->Kd = Kd;

    params->setPoint = setPoint;
    params->output = output;
    params->processValue = processValue;

    params->maxIntegral = INT32_MAX / (params->Ki + 1);

    params->lastProcessValue = 0;
    params->integral = 0;

    params->outMin = outMin;
    params->outMax = outMax;
}

void pidCompute(pidparams * const params) {
    int32_t pTerm, iTerm, dTerm, error;
    int16_t processValue = *params->processValue;

    error = *params->setPoint - processValue;

    params->integral = CLAMP(params->integral + error,
        -params->maxIntegral, params->maxIntegral);

    pTerm = params->Kp * error;
    iTerm = params->Ki * params->integral;
    dTerm = (int32_t)params->Kd * (processValue - params->lastProcessValue);

    params->lastProcessValue = processValue;

    *params->output = CLAMP((pTerm + iTerm + dTerm) / PID_DIVIDER,
        params->outMin, params->outMax);
}

void pidReset(pidparams * const params) {
    params->integral = CLAMP(*params->output / params->Ki,
        params->outMin, params->outMax);
    params->lastProcessValue = *params->processValue;
}

#endif

