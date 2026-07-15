# Error-state Kalman filter mathematical specification

This document is the normative convention for the estimator core. Code,
Jacobians, tests, dataset adapters, and ROS frames must agree with it. Symbols
without a hat denote the physical state; hats denote the nominal estimate.

## Frames, axes, transforms, and units

- `w`: local world frame. Dataset adapters provide ENU axes: x east, y north,
  z up. `map` is the default ROS name.
- `b`: vehicle/body frame at `base_link`, using ROS FLU axes: x forward, y left,
  z up.
- `i`: IMU measurement frame. The ROS component transforms IMU vectors and
  orientation into `b` before calling the core. The core therefore assumes
  `i == b` and contains no IMU extrinsic state.
- `g`: GNSS antenna point. Its fixed lever arm is
  \(\ell_{bg}^{b}\), from the body origin to the antenna, expressed in `b`.
- \(R_{wb}=R(q_{wb})\) maps a vector from `b` to `w`.
- \(q_{wb}\) is an active Hamilton quaternion stored internally as Eigen XYZW
  coefficients but constructed as `(w,x,y,z)`.
- Position is metres, velocity m/s, specific force m/s², angular rate rad/s,
  bias units match their sensors, time is seconds, and all covariance entries
  use squared SI units.
- \([a]_\times b=a\times b\).

The gravity vector is \(g^w=[0,0,g]^T\), positive upward as stored in the
code, and is subtracted from transformed accelerometer specific force. A
stationary, level standard IMU therefore measures approximately `[0,0,+g]` in
FLU. Dataset converters must restore gravity when a source publishes
gravity-compensated acceleration.

## Nominal and error states

The 16-parameter nominal state and 15-dimensional local error are

\[
x=(p^w,v^w,q_{wb},b_g^b,b_a^b),\qquad
\delta x=(\delta p^w,\delta v^w,\delta\theta^b,\delta b_g^b,\delta b_a^b).
\]

The code index mapping is fixed:

| Block | Nominal indices | Error indices | Dimension |
|---|---:|---:|---:|
| position | 0--2 | 0--2 | 3 |
| world velocity | 3--5 | 3--5 | 3 |
| quaternion XYZW | 6--9 | — | 4 |
| gyro bias | 10--12 | 9--11 | 3 |
| accelerometer bias | 13--15 | 12--14 | 3 |
| local attitude error | — | 6--8 | 3 |

Attitude uses a right-multiplicative local error:

\[
R_{wb}^{true}=R_{wb}^{nom}\operatorname{Exp}(\delta\theta),\qquad
q^{true}=q^{nom}\otimes\operatorname{Exp}_q(\delta\theta).
\]

This makes \(\delta\theta\) a body-frame tangent vector. It is not an RPY
difference.

## Continuous dynamics

With IMU measurements \(\omega_m,a_m\), corrected inputs are
\(\hat\omega=\omega_m-\hat b_g\) and
\(\hat a=a_m-\hat b_a\). The nominal dynamics are

\[
\dot p=v,\quad
\dot v=R_{wb}\hat a-g^w,\quad
\dot R_{wb}=R_{wb}[\hat\omega]_\times,
\]
\[
\dot b_g=-\tau_g^{-1}b_g+w_{bg},\qquad
\dot b_a=-\tau_a^{-1}b_a+w_{ba}.
\]

For infinite or disabled correlation time the corresponding decay term is
zero. The first-order right-error dynamics are

\[
\dot{\delta x}=F_c\delta x+G_c n,
\]

\[
F_c=\begin{bmatrix}
0&I&0&0&0\\
0&0&-R_{wb}[\hat a]_\times&0&-R_{wb}\\
0&0&-[\hat\omega]_\times&-I&0\\
0&0&0&-\tau_g^{-1}I&0\\
0&0&0&0&-\tau_a^{-1}I
\end{bmatrix}.
\]

For \(n=(n_a,n_g,w_{bg},w_{ba})\), a sign-consistent noise matrix is

\[
G_c=\operatorname{blkcol}((0,-R,0,0,0)^T,
(0,0,-I,0,0)^T,(0,0,0,I,0)^T,(0,0,0,0,I)^T).
\]

The signs of independent zero-mean white-noise columns do not change
\(G_cQ_cG_c^T\), but the implementation and tests use the signs above.
The continuous spectral-density matrix is

\[
Q_c=\operatorname{diag}(\sigma_a^2I,\sigma_g^2I,
\sigma_{bg}^2I,\sigma_{ba}^2I).
\]

Configuration fields named `var_*` are variances/spectral densities, not
standard deviations.

## Discretization

The nominal state uses a zero-order hold over \(\Delta t\):

\[
p_{k+1}=p_k+v_k\Delta t+\tfrac12(R_k\hat a_k-g)\Delta t^2,
\]
\[
v_{k+1}=v_k+(R_k\hat a_k-g)\Delta t,
\quad q_{k+1}=q_k\otimes\operatorname{Exp}_q(\hat\omega_k\Delta t),
\]
\[
b_{g,k+1}=e^{-\Delta t/\tau_g}b_{g,k},\qquad
b_{a,k+1}=e^{-\Delta t/\tau_a}b_{a,k}.
\]

The `fast` path uses
\(F_d=I+F_c\Delta t+\tfrac12(F_c\Delta t)^2\). Its process covariance is
the symmetric third-order expansion of

\[
Q_d=\int_0^{\Delta t}e^{F_c s}G_cQ_cG_c^T e^{F_c^Ts}\,ds.
\]

The `exact` path evaluates the 30x30 Van Loan exponential

\[
\exp\left(\begin{bmatrix}F_c&G_cQ_cG_c^T\\0&-F_c^T\end{bmatrix}\Delta t\right)
=\begin{bmatrix}\Phi&A\\0&\Phi^{-T}\end{bmatrix},\qquad Q_d=A\Phi^T.
\]

The covariance prediction is \(P^-=\Phi P^+\Phi^T+Q_d\), followed by explicit
symmetrization. `legacy` preserves the historical discrete-noise behaviour only
for regression. `fast` and `exact` use midpoint IMU mean integration; `exact`
is the offline oracle and `fast` is the runtime approximation.

## Measurement models

Every residual is \(r=z-h(\hat x)\). Jacobians below are with respect to the
right error state at zero. Measurement covariance matrices must be finite and
positive definite.

### Body-origin position

\[
h_p=p^w,\qquad H_p=[I\;0\;0\;0\;0].
\]

### GNSS antenna position and lever arm

\[
h_g=p^w+R_{wb}\ell_{bg}^b,
\]

\[
H_g=[I\;0\;-R_{wb}[\ell_{bg}^b]_\times\;0\;0].
\]

The lever arm is never rotated twice and is never treated as a world-frame
offset.

### World-frame velocity

\[
h_{v,w}=v^w,\qquad H_{v,w}=[0\;I\;0\;0\;0].
\]

GNSS Doppler velocity and position-derived velocity use this model after the
ROS adapter expresses them in `w`.

### Body-frame velocity, wheel speed, and NHC

Let \(u^b=R_{wb}^Tv^w\). Then

\[
h_{v,b}=u^b,\qquad
H_{v,b}=[0\;R_{wb}^T\;[u^b]_\times\;0\;0].
\]

- Forward wheel speed selects row x of this model.
- The non-holonomic constraint selects rows y,z and normally observes zero.
- A simultaneous wheel+NHC update uses all three rows with independent or full
  measurement covariance as configured.
- `wheel_vertical_nhc_gnss_available_variance_scale` multiplies only the
  vertical NHC variance while GNSS is available. During a detected GNSS outage
  the nominal variance is restored. This prevents the flat-ground assumption
  from dominating observed altitude while retaining vertical drift control in
  dead reckoning; the selected variance is captured with each replay event.

Omitting the attitude columns or restricting the Kalman gain to a velocity
sub-block is a named `decoupled` ablation, not the normative ESKF update.

### Gyro bias during stationarity

\[
h_{bg}=b_g,\qquad H_{bg}=[0\;0\;0\;I\;0].
\]

### Absolute orientation and course yaw

For quaternion measurement \(q_z\), the innovation is the shortest-arc log map

\[
r_q=\operatorname{Log}_q(\hat q^{-1}\otimes q_z).
\]

At zero error, \(H_q=[0\;0\;I\;0\;0]\). Full orientation, flat-ground
roll/pitch, GNSS course yaw, and Doppler course yaw all use selected components
of this tangent-space model. Course yaw is gated below its speed/displacement
observability threshold; it is not an absolute heading observation at rest.

## Kalman update, injection, and reset

Innovation solves must use an LLT/LDLT factorization, never an explicit matrix
inverse:

\[
S=HP^-H^T+R,\qquad K=(S^{-1}HP^-)^T.
\]

After \(\delta\hat x=Kr\), the nominal injection is

\[
p\leftarrow p+\delta p,\quad v\leftarrow v+\delta v,\quad
q\leftarrow q\otimes\operatorname{Exp}_q(\delta\theta),
\]
\[
b_g\leftarrow b_g+\delta b_g,\qquad b_a\leftarrow b_a+\delta b_a.
\]

The pre-reset Joseph covariance is

\[
P_J=(I-KH)P^-(I-KH)^T+KRK^T.
\]

Because the attitude error origin changed during injection, covariance must be
reset with

\[
G_r=\operatorname{diag}(I,I,J_r(\delta\theta),I,I),\qquad
P^+=G_rP_JG_r^T,
\]

where \(J_r\) is the SO(3) right Jacobian. For tiny corrections,
\(J_r(\delta\theta)\simeq I-\tfrac12[\delta\theta]_\times\). The exact stable
closed form is required away from zero. Quaternion normalization follows every
propagation and injection.

## Numerical invariants

After every accepted prediction or update:

1. every nominal state and covariance entry is finite;
2. quaternion norm differs from one by no more than numerical roundoff;
3. covariance is explicitly symmetric;
4. an LDLT/LLT check finds no materially negative covariance eigenvalue;
5. innovation factorization succeeds before state mutation.

Debug/test builds expose these checks. A numerical failure rejects the update
without partially mutating state. Phase 6 reports NIS using the same innovation
factorization and NEES in this 15-dimensional local error convention.
