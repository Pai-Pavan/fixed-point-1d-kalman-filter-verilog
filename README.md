# Fixed Point 1D Kalman Filter in Verilog

This project implements a 1D Kalman Filter in Verilog using fixed point arithmetic for real time estimation of a noisy signal.

The design focuses on translating the mathematical Kalman filter equations into a hardware efficient RTL architecture, with careful handling of precision, scaling, and sequential data flow.

---

## Overview

The implemented system estimates an angular position signal from noisy measurements.

Instead of directly using floating point arithmetic, the design uses a Q12.12 fixed point representation to make the implementation suitable for FPGA based systems.

A dataset containing noisy measurements is stored in a `.mem` file and fed into the design during simulation. The filter output is then compared against:
- the true signal
- the noisy measurements
- the filtered output

using MATLAB visualization.

---

## Kalman Filter Model

A 1D constant velocity model is used.

The following equations are implemented:

### Prediction Step
- x_pred = x_prev + (ω × Δt)
- P_pred = P_prev + Q

### Update Step
- K = P_pred / (P_pred + R)
- x = x_pred + K × (z - x_pred)
- P = (1 - K) × P_pred

Where:
- x → estimated state (angle)
- z → measured value
- ω → angular velocity (derived from measurements)
- Q → process noise
- R → measurement noise

---

## Implementation Details

### Fixed Point Representation
- Format: Q12.12 (24 bit signed)
- Scaling factor: 2¹² = 4096

---

## Architecture

The design is modular and consists of:

- `predict_mean` → state prediction
- `pred_covariance` → covariance prediction
- `gain_calc` → Kalman gain computation
- `update_mean` → state correction
- `update_covariance` → covariance update
- `top module` → sequential state update across clock cycles

The system operates synchronously with state variables updated every clock cycle.

---

## Testbench and Data Flow

- Noisy sensor values are stored in a `.mem` file
- Data is sequentially fed into the filter
- Angular velocity is computed as:
  
  ω = (z_current - z_previous) / Δt

- Output is printed during simulation

The testbench validates:
- convergence behavior
- noise reduction capability
- stability of estimation

---

## MATLAB Visualization

The filter output is plotted against:
- true signal
- noisy measurement
- Kalman filtered output

This helps visually verify:
- smoothing effect
- tracking accuracy
- steady state convergence

---

## Key Observations

- The filter effectively reduces measurement noise
- Output converges smoothly to the true signal
- Fixed-point implementation introduces minor quantization effects
- Proper scaling is critical for stability

---

## Limitations

- Uses pre generated dataset (no real sensor input yet)
- Model assumes constant velocity
- No adaptive tuning of Q and R
  
---

## Possible Improvements

- Replace division with approximation methods (e.g., reciprocal LUT)
- Extend to 2D Kalman filter (position + velocity)
- Integrate real sensor input (IMU)
- Optimize for FPGA synthesis and resource usage

---

## Author

Pavan Pai
