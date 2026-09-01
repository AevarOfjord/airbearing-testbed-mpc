# Math (planar 3-DOF)

Not flight software. Symbols match the code.

## Plant

State (inertial) \(x = [p_x, p_y, \theta, v_x, v_y, \omega]^\top\).

Body wrench \(u_b = [F_x, F_y, M_z]^\top\). With rotation \(R(\theta)\),

\[
\dot v = \frac{1}{m} R(\theta)\, F_b - \frac{c_\ell}{m} v, \qquad
\dot\omega = \frac{1}{I_z} M_z - \frac{c_r}{I_z}\omega.
\]

Thruster \(i\) at body position \(r_i\) with unit force direction \(d_i\) and command \(u_i\in[u_i^{\min}, u_i^{\max}]\) contributes \(F_i = u_i F_{i,\max}\, d_i\) and torque \((r_i - r_{\mathrm{com}}) \times F_i\).

PWM fans: \(\dot F_i = (u_i F_{i,\max} - F_i)/\tau_i\).

Reaction wheel (sim): spacecraft \(M_z \mathrel{+}= \tau_w\), wheel momentum saturates at \(h_{\max}\).

## Allocation

\[
B_{\cdot i} = F_{i,\max}
\begin{bmatrix} d_i \\ (r_i - r_{\mathrm{com}})_x d_{i,y} - (r_i - r_{\mathrm{com}})_y d_{i,x} \end{bmatrix},
\quad
\min_u \|Bu - w\|_2^2 + \varepsilon \|u\|_2^2
\]

subject to box constraints. Binary solenoids: relax \(u_i\in[0,1]\), apply as duty or round (`--round-binary`).

## Linear MPC

Horizon \(N\), sample \(\Delta t\). Freeze \(\theta\) over the horizon so \(A,B\) are constant:

\[
x_{k+1} = A(\theta) x_k + B(\theta) u_k, \quad
u_k = [F_x, F_y, M_z]_b.
\]

Cost \(\sum \|x_k - x_{\mathrm{ref}}\|_Q^2 + \|u_k\|_R^2\). Heading error is wrapped. Solve with OSQP / Clarabel. Log solve time vs \(\Delta t\) (deadline miss if wall time \(> 1.05\Delta t\)).

## PD / LQR

PD: inertial position/yaw error → body force via \(R^\top\). LQR: discrete DARE on the same \((A,B)\), cached in coarse \(\theta\) bins.
