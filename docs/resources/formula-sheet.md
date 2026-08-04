# Formula sheet

## Longitudinal control

$$
e_v = v_{\mathrm{ref}} - v
$$

$$
u_P = K_P e_v
$$

$$
u_{PI} = K_P e_v + K_I \int e_v\,dt
$$

## Kinematic bicycle model

$$
\dot{x}=v\cos\psi,\qquad
\dot{y}=v\sin\psi,\qquad
\dot{\psi}=\frac{v}{L}\tan\delta
$$

## Curve-aware speed

$$
a_y=v^2\kappa
$$

$$
v_{\mathrm{safe}}=\sqrt{\frac{a_{y,\max}}{|\kappa|}}
$$

## Adaptive cruise control

$$
d_{\mathrm{desired}}=d_0+T_hv
$$

