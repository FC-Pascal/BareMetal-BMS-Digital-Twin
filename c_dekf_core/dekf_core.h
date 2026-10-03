#ifndef DEKF_CORE_H
#define DEKF_CORE_H

#include <stdint.h>

// Battery Equivalent Circuit Parameters
#define Q_NOM_AS 9360.0f  // 2.6Ah * 3600
#define R0_NOM   0.015f
#define R1_NOM   0.010f
#define C1_NOM   1500.0f
#define R2_NOM   0.012f
#define C2_NOM   4500.0f
#define DT       1.0f     // 1-second sample time

// State Vector Definition
typedef struct {
    float SoC;       // State of Charge [0.0 - 1.0]
    float V1;        // RC1 Polarization Voltage
    float V2;        // RC2 Polarization Voltage
    float R0_est;    // Online Estimated Internal Resistance (Health)
} BatteryState_t;

// Covariance Matrices Definition
typedef struct {
    float P[3][3];   // State Error Covariance (3x3 for SoC, V1, V2)
    float P_R0;      // Parameter Error Covariance (1x1 for R0)
    float Q[3][3];   // Process Noise
    float R;         // Measurement Noise
} FilterCovariance_t;

// Function Prototypes
void EKF_Init(BatteryState_t* state, FilterCovariance_t* cov);
void EKF_Predict(BatteryState_t* state, FilterCovariance_t* cov, float current_mA);
void EKF_Update(BatteryState_t* state, FilterCovariance_t* cov, float voltage_mV, float current_mA);

#endif // DEKF_CORE_H