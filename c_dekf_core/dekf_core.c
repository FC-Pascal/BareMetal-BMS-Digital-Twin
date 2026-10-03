#include "dekf_core.h"

// Bare-metal fast exponential approximation (3rd-Order Taylor Series)
// Completely bypasses standard C-library memory allocation bugs
static float fast_expf(float x) {
    // e^x ˜ 1 + x + x^2/2 + x^3/6
    return 1.0f + x + (x * x) / 2.0f + (x * x * x) / 6.0f;
}

void EKF_Init(BatteryState_t* state, FilterCovariance_t* cov) {
    // 1. Initial State Guess (Deliberately wrong to prove convergence)
    state->SoC = 0.80f; 
    state->V1 = 0.0f;
    state->V2 = 0.0f;
    
    // We start assuming the nominal health/resistance
    state->R0_est = R0_NOM;

    // 2. Initial Error Covariance (P)
    // We don't trust our initial SoC guess, so we set P high
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
            cov->P[i][j] = (i == j) ? 0.1f : 0.0f; // Diagonal matrix
        }
    }
    cov->P_R0 = 0.01f; // High uncertainty for initial R0

    // 3. Process Noise Covariance (Q)
    // We trust our physics equations, so Q is very small
    cov->Q[0][0] = 1e-6f; // SoC process noise
    cov->Q[1][1] = 1e-5f; // V1 process noise
    cov->Q[2][2] = 1e-5f; // V2 process noise
    cov->Q[0][1] = cov->Q[0][2] = cov->Q[1][0] = 0.0f;
    cov->Q[1][2] = cov->Q[2][0] = cov->Q[2][1] = 0.0f;

    // 4. Measurement Noise Covariance (R)
    // Represents the physical noise of our ADCs
    cov->R = 1e-2f; 
}


void EKF_Predict(BatteryState_t* state, FilterCovariance_t* cov, float current_A) {
    // 1. Calculate Exponential Decay Factors using our custom bare-metal math
    float exp1 = fast_expf(-DT / (R1_NOM * C1_NOM));
    float exp2 = fast_expf(-DT / (R2_NOM * C2_NOM));
    
    // 2. State Prediction (Time Update)
    // Coulomb counting for SoC, and exponential decay for RC voltages
    state->SoC = state->SoC - (current_A * DT) / Q_NOM_AS;
    state->V1 = state->V1 * exp1 + R1_NOM * (1.0f - exp1) * current_A;
    state->V2 = state->V2 * exp2 + R2_NOM * (1.0f - exp2) * current_A;
    
    // 3. Covariance Prediction (P = A * P * A^T + Q)
    // ARCHITECTURE WIN: Because our A (Jacobian) matrix is diagonal, 
    // we do not need to run a heavy O(N^3) matrix multiplication loop. 
    // We can unroll it directly to save massive amounts of CPU cycles!
    
    // Row 1 (SoC)
    cov->P[0][0] = cov->P[0][0] + cov->Q[0][0];
    cov->P[0][1] = cov->P[0][1] * exp1;
    cov->P[0][2] = cov->P[0][2] * exp2;
    
    // Row 2 (V1)
    cov->P[1][0] = cov->P[1][0] * exp1;
    cov->P[1][1] = (exp1 * cov->P[1][1] * exp1) + cov->Q[1][1];
    cov->P[1][2] = (exp1 * cov->P[1][2] * exp2);
    
    // Row 3 (V2)
    cov->P[2][0] = cov->P[2][0] * exp2;
    cov->P[2][1] = (exp2 * cov->P[2][1] * exp1);
    cov->P[2][2] = (exp2 * cov->P[2][2] * exp2) + cov->Q[2][2];
}

// Define OCV curve constants at the top of the file (under your includes)
const float SOC_CURVE[11] = {0.0f, 0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f, 0.9f, 1.0f};
const float OCV_CURVE[11] = {3.0f, 3.4f, 3.5f, 3.6f, 3.65f, 3.7f, 3.8f, 3.9f, 4.0f, 4.1f, 4.2f};
const float dOCV_dSOC_CURVE[11] = {4.0f, 1.0f, 1.0f, 0.5f, 0.5f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};

// Lightweight 1D Linear Interpolation (Replaces MATLAB's interp1)
static float interp1(const float* x_data, const float* y_data, int size, float x_val) {
    if (x_val <= x_data[0]) return y_data[0];
    if (x_val >= x_data[size - 1]) return y_data[size - 1];
    
    for (int i = 0; i < size - 1; i++) {
        if (x_val >= x_data[i] && x_val <= x_data[i + 1]) {
            float slope = (y_data[i + 1] - y_data[i]) / (x_data[i + 1] - x_data[i]);
            return y_data[i] + slope * (x_val - x_data[i]);
        }
    }
    return y_data[0];
}

void EKF_Update(BatteryState_t* state, FilterCovariance_t* cov, float voltage_V, float current_A) {
    // 1. Predict Terminal Voltage
    float ocv_pred = interp1(SOC_CURVE, OCV_CURVE, 11, state->SoC);
    float vt_pred = ocv_pred - state->V1 - state->V2 - (state->R0_est * current_A);
    
    // 2. Innovation (Residual error between measured and predicted voltage)
    float innovation = voltage_V - vt_pred;
    
    // 3. Measurement Jacobian (H Matrix) -> [dOCV/dSoC, -1, -1]
    float H[3];
    H[0] = interp1(SOC_CURVE, dOCV_dSOC_CURVE, 11, state->SoC);
    H[1] = -1.0f;
    H[2] = -1.0f;
    
    // 4. Calculate Kalman Gain (K = P * H^T * S^-1)
    float S = 0.0f;
    float PH[3] = {0}; 
    
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            PH[i] += cov->P[i][j] * H[j]; // P * H^T
        }
        S += H[i] * PH[i];
    }
    S += cov->R; // Innovation covariance
    
    float K[3];
    for(int i = 0; i < 3; i++) {
        K[i] = PH[i] / S;
    }
    
    // 5. Update State Estimate (x = x + K * innovation)
    state->SoC += K[0] * innovation;
    state->V1  += K[1] * innovation;
    state->V2  += K[2] * innovation;
    
    // Clamp SoC to valid physical limits
    if(state->SoC > 1.0f) state->SoC = 1.0f;
    if(state->SoC < 0.0f) state->SoC = 0.0f;
    
    // 6. Update Covariance (P = P - K * H * P)
    float KHP[3][3];
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            // Since P is symmetric, H*P simplifies to PH
            KHP[i][j] = K[i] * PH[j]; 
            cov->P[i][j] -= KHP[i][j];
        }
    }
}