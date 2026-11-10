# 6-DOF Dual-Mode Robotic Additive Manufacturing Digital Twin (Formlabs SLA & SLS)

An end-to-end, physics-grounded 3D Digital Twin and closed-loop process control platform for automated Robotic Additive Manufacturing (RAM). The system models industrial **Formlabs Form 4L/3L (Low Force Stereolithography - SLA)** and **Formlabs Fuse 1+ 30W (Selective Laser Sintering - SLS)** stations integrated with an industrial 6-DOF articulated manipulator maintaining a calibrated $150\text{ mm}$ coaxial inspection standoff.

![System Architecture](https://raw.githubusercontent.com/username/repo/main/docs/assets/system_architecture.png)

## Core Capabilities

- **Dual-Mode Process Physics:**
  - **SLA Mode:** Beer-Lambert absorption and Jacobs Working Curves ($C_d = D_p \ln(E_{\max}/E_c)$) with parabolic line width ($L_w$) modeling for Formlabs Clear, Tough 2000, Rigid 10K, Silicone 40A, and benchmark PEGDA-700.
  - **SLS Mode:** 3D quasi-steady-state Rosenthal moving point-source thermal solver modified with volumetric melting enthalpy $H_v = \rho[c_p(T_m - T_0) + L_f]$ to capture polymer latent heat of fusion for Nylon 12, Nylon 12 GF, Nylon 11, and TPU 90A.
- **6-DOF Kinematic Engine:**
  - Standard Denavit-Hartenberg (DH) forward kinematics (${}^0T_6$).
  - Pieper closed-form spherical wrist decoupling and Damped Least Squares (DLS) Jacobian inverse kinematics ($\Delta\boldsymbol{\theta} = J^T(JJ^T + \lambda_d^2 I)^{-1}\mathbf{e}$).
  - Calibrated end-effector standoff holding distance ($d_{\text{hold}} = 150\text{ mm}$) for non-contact inline sensing.
- **Closed-Loop Dual-Actuator PID Controller:**
  - Layer-by-layer dynamic error compensation mitigating exothermic vat heating and core thermal accumulation.
  - Actuator effort splitting between laser radiant power ($P_L$) and galvanometer velocity ($v_s$).
  - Clamping and back-calculation anti-windup logic.
- **50-Layer Cylindrical Test Build ($D = 40\text{ mm}$):**
  - Continuous toolpath simulation with contour perimeters and alternating $0^\circ/90^\circ$ bidirectional raster infill.
  - Layer-by-layer 3D mesh deposition color-coded by dimensional error tolerance.
- **6-Channel DSP-Style Telemetry Suite:**
  - Live charts for Jacobs working curves, Rosenthal moving melt fields, joint trajectories ($\theta_1 \dots \theta_6$), RSM response contours, PID error tracking, and frequency-domain disturbance spectra.

---

## Mathematical Formulation

### 1. 6-DOF Manipulator Denavit-Hartenberg Parameters

| Joint $i$ | Type / Description | $a_i$ (mm) | $\alpha_i$ (deg) | $d_i$ (mm) | $\theta_{i,\text{home}}$ (deg) | Joint Limits (deg) |
| :---: | :--- | :---: | :---: | :---: | :---: | :---: |
| **1** | Base Yaw ($Z_0$) | $25.0$ | $-90.0^\circ$ | $400.0$ | $0.0^\circ$ | $[-170^\circ, +170^\circ]$ |
| **2** | Shoulder Pitch ($Z_1$) | $455.0$ | $0.0^\circ$ | $0.0$ | $-90.0^\circ$ | $[-190^\circ, +45^\circ]$ |
| **3** | Elbow Pitch ($Z_2$) | $35.0$ | $-90.0^\circ$ | $0.0$ | $0.0^\circ$ | $[-120^\circ, +156^\circ]$ |
| **4** | Forearm Roll ($Z_3$) | $0.0$ | $+90.0^\circ$ | $420.0$ | $0.0^\circ$ | $[-185^\circ, +185^\circ]$ |
| **5** | Wrist Pitch ($Z_4$) | $0.0$ | $-90.0^\circ$ | $0.0$ | $0.0^\circ$ | $[-120^\circ, +120^\circ]$ |
| **6** | Flange Roll ($Z_5$) | $0.0$ | $0.0^\circ$ | $80.0$ | $0.0^\circ$ | $[-350^\circ, +350^\circ]$ |

$$\mathbf{p}_w = \mathbf{p}_{\text{TCP}} - d_6 \mathbf{a}, \quad \theta_1 = \text{atan2}(y_w, x_w)$$

### 2. SLA Mode: Jacobs Working Curve
$$I(r, z) = \frac{2 P_{\text{eff}}}{\pi W_0^2} \exp\left(-\frac{2r^2}{W_0^2}\right)\exp\left(-\frac{z}{D_p}\right)$$

$$E_{\max} = \sqrt{\frac{2}{\pi}} \frac{P_{\text{eff}}}{W_0 v_s}, \quad C_d = D_p \ln\left(\frac{E_{\max}}{E_c}\right), \quad L_w = 2 W_0 \sqrt{\frac{C_d}{2 D_p}}$$

### 3. SLS Mode: Enthalpy-Corrected Rosenthal 3D Thermal Model
$$T(\xi, y, z) = T_0 + \frac{\alpha_a P_L}{2\pi k_{\text{eff}} R}\exp\left(-\frac{v_s(\xi + R)}{2\alpha_{\text{th}}}\right)$$

Under high Peclet numbers ($\text{Pe} \gg 1$) with latent heat of fusion $L_f$:
$$d_m = \sqrt{\frac{2 \alpha_a P_L}{\pi e \, \rho_{\text{eff}}\left[c_p(T_m - T_0) + L_f\right] v_s}}, \quad w_m = \sqrt{(2 d_m)^2 + d_b^2}$$

---

## Experimental Validation vs. Literature

| Physical Metric | Digital Twin | Benchmark / Literature | Absolute Error | Reference |
| :--- | :---: | :---: | :---: | :--- |
| **Clear V5 $D_p / E_c$** | $155\ \mu\text{m} \ /\ 10.2\text{ mJ/cm}^2$ | $150\text{–}160\ \mu\text{m} \ /\ 9.8\text{–}10.5$ | $< 1.3\%$ | Jacobs (1992), Bennett (2017) |
| **Rigid 10K $C_d$** | $146.8\ \mu\text{m}$ | $148.0\ \mu\text{m}$ | $0.81\%$ | Formlabs Technical Data Sheet |
| **PA12 Melt Depth $d_m$** | $135.9\ \mu\text{m}$ | $138.0\ \mu\text{m}$ | $1.52\%$ | Chatham et al. (2019) |
| **PA12 Relative Density** | $99.09\%$ | $98.8\text{–}99.2\%$ | $0.10\%$ | Lupone et al. (2022) |

---
