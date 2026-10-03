#include "dekf_core.h"
#include "synthetic_bms_data.h"

// 1. Move these OUTSIDE of main() to make them Global Variables.
// The Keil debugger can now track their memory addresses perfectly.
BatteryState_t bms_state;
FilterCovariance_t bms_cov;

int main() {
    // 2. Initialize the EKF
    EKF_Init(&bms_state, &bms_cov);

    // 3. Loop through the exported MATLAB data
    for (int i = 0; i < NUM_SAMPLES; i++) {
        EKF_Predict(&bms_state, &bms_cov, I_MEASURED[i]);
        EKF_Update(&bms_state, &bms_cov, V_MEASURED[i], I_MEASURED[i]);
    }

    // 4. Safety trap to keep the processor running at the end
    while(1); 
    
    return 0;
}