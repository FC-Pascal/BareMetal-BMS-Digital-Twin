%% Battery Cell Parameters (Nominal 18650 Li-ion)
Q_nom = 2.6 * 3600;  % Nominal Capacity in Ampere-seconds (Ah -> As)
R0 = 0.015;          % Internal Ohmic Resistance (Ohms)
R1 = 0.010;          % Short-term polarization resistance
C1 = 1500;           % Short-term polarization capacitance
R2 = 0.012;          % Long-term polarization resistance
C2 = 4500;           % Long-term polarization capacitance

%% Simulation Setup
dt = 1;              % 1-second sampling time
time = 0:dt:1000;    % 1000 seconds of simulation

% Create a current profile: 0A for 100s, 5A discharge for 400s, 0A rest
current = zeros(size(time));
current(100:500) = 5.0;

%% Non-Linear OCV-SoC Relationship
% Lookup table representing a typical Li-ion discharge curve
soc_vector = 0:0.1:1;
ocv_vector = [3.0, 3.4, 3.5, 3.6, 3.65, 3.7, 3.8, 3.9, 4.0, 4.1, 4.2];

%% State Initialization
N = length(time);
SoC = zeros(1, N);
V1 = zeros(1, N);
V2 = zeros(1, N);
Vt = zeros(1, N);

% Initial Conditions (Starting 100% charged)
SoC(1) = 1.0; 
V1(1) = 0;
V2(1) = 0;
Vt(1) = interp1(soc_vector, ocv_vector, SoC(1));

%% Discrete-Time State-Space Execution
for k = 2:N
    % 1. Update State of Charge (Coulomb Counting)
    SoC(k) = SoC(k-1) - (current(k-1) * dt) / Q_nom;
    
    % 2. Update RC Polarization Voltages 
    V1(k) = V1(k-1) * exp(-dt / (R1 * C1)) + R1 * (1 - exp(-dt / (R1 * C1))) * current(k-1);
    V2(k) = V2(k-1) * exp(-dt / (R2 * C2)) + R2 * (1 - exp(-dt / (R2 * C2))) * current(k-1);
    
    % 3. Calculate Terminal Voltage
    OCV = interp1(soc_vector, ocv_vector, SoC(k), 'linear', 'extrap');
    Vt(k) = OCV - V1(k) - V2(k) - R0 * current(k);
end

%% Validation Plotting
figure;
subplot(3,1,1);
plot(time, current, 'r', 'LineWidth', 1.5);
ylabel('Current (A)'); title('Load Profile (Pulse Discharge)');

subplot(3,1,2);
plot(time, SoC, 'g', 'LineWidth', 1.5);
ylabel('SoC'); title('State of Charge Drift');

subplot(3,1,3);
plot(time, Vt, 'b', 'LineWidth', 1.5);
ylabel('Terminal Voltage (V)'); title('Battery Voltage Drop (V_t)');
xlabel('Time (s)');

%% 4. Sensor Noise Injection (Simulating Real-World ADCs)
voltage_noise_variance = 0.005^2; % 5mV noise on the voltage sensor
current_noise_variance = 0.05^2;  % 50mA noise on the current sensor

Vt_measured = Vt + sqrt(voltage_noise_variance) * randn(1, N);
I_measured = current + sqrt(current_noise_variance) * randn(1, N);

%% 5. Extended Kalman Filter (EKF) Initialization
% State vector: x = [SoC; V1; V2]
x_hat = [0.8; 0; 0]; % DELIBERATELY WRONG GUESS: Filter thinks battery is at 80%
P = diag([0.1, 0.01, 0.01]);  % Initial Error Covariance
Q = diag([1e-6, 1e-5, 1e-5]); % Process Noise (We trust our physics model)
R = 1e-2;                     % Measurement Noise (We distrust the noisy sensors)

SoC_estimated = zeros(1, N);
SoC_estimated(1) = x_hat(1);

% Pre-calculate the derivative of the OCV curve for the Jacobian Matrix (H)
dOCV_dSoC = diff(ocv_vector) ./ diff(soc_vector);
dOCV_dSoC = [dOCV_dSoC, dOCV_dSoC(end)]; % Pad array to match length

%% 6. The EKF Real-Time Loop
for k = 2:N
    % --- A. PREDICTION STEP (Time Update) ---
    % Predict next state using noisy current measurement
    SoC_pred = x_hat(1) - (I_measured(k-1) * dt) / Q_nom;
    V1_pred = x_hat(2) * exp(-dt / (R1 * C1)) + R1 * (1 - exp(-dt / (R1 * C1))) * I_measured(k-1);
    V2_pred = x_hat(3) * exp(-dt / (R2 * C2)) + R2 * (1 - exp(-dt / (R2 * C2))) * I_measured(k-1);
    x_pred = [SoC_pred; V1_pred; V2_pred];
    
    % State Transition Jacobian (A Matrix)
    A = [1, 0, 0;
         0, exp(-dt / (R1 * C1)), 0;
         0, 0, exp(-dt / (R2 * C2))];
         
    % Predict Covariance
    P_pred = A * P * A' + Q;
    
    % --- B. UPDATE STEP (Measurement Update) ---
    % Predict what the terminal voltage SHOULD be
    OCV_pred = interp1(soc_vector, ocv_vector, SoC_pred, 'linear', 'extrap');
    Vt_pred = OCV_pred - V1_pred - V2_pred - R0 * I_measured(k);
    
    % Measurement Jacobian (H Matrix): Partial derivatives of Vt with respect to states
    dOCV = interp1(soc_vector, dOCV_dSoC, SoC_pred, 'linear', 'extrap');
    H = [dOCV, -1, -1]; 
    
    % Calculate Innovation (Residual): Difference between measured and predicted voltage
    innovation = Vt_measured(k) - Vt_pred;
    
    % Calculate Kalman Gain
    S = H * P_pred * H' + R;
    K = (P_pred * H') / S;
    
    % Update State Estimate
    x_hat = x_pred + K * innovation;
    
    % Update Covariance Matrix
    P = (eye(3) - K * H) * P_pred;
    
    % Log the estimated SoC for plotting
    SoC_estimated(k) = x_hat(1);
end

%% 7. EKF Validation Plotting
figure;
plot(time, SoC, 'g', 'LineWidth', 2); hold on;
plot(time, SoC_estimated, 'r--', 'LineWidth', 1.5);
title('EKF State of Charge Estimation');
xlabel('Time (s)'); ylabel('State of Charge (SoC)');
legend('True SoC', 'EKF Estimated SoC');
grid on;

%% 8. Export Data to C Header for Bare-Metal Testing
fileID = fopen('synthetic_bms_data.h', 'w');
fprintf(fileID, '#ifndef SYNTHETIC_BMS_DATA_H\n#define SYNTHETIC_BMS_DATA_H\n\n');
fprintf(fileID, '#define NUM_SAMPLES %d\n\n', N);

% Write Current Array
fprintf(fileID, 'const float I_MEASURED[%d] = {\n    ', N);
for i = 1:N-1
    fprintf(fileID, '%.5ff, ', I_measured(i));
    if mod(i, 10) == 0, fprintf(fileID, '\n    '); end
end
fprintf(fileID, '%.5ff\n};\n\n', I_measured(N));

% Write Voltage Array
fprintf(fileID, 'const float V_MEASURED[%d] = {\n    ', N);
for i = 1:N-1
    fprintf(fileID, '%.5ff, ', Vt_measured(i));
    if mod(i, 10) == 0, fprintf(fileID, '\n    '); end
end
fprintf(fileID, '%.5ff\n};\n\n', Vt_measured(N));

fprintf(fileID, '#endif // SYNTHETIC_BMS_DATA_H\n');
fclose(fileID);
disp('Successfully exported synthetic_bms_data.h to c_dekf_core directory.');