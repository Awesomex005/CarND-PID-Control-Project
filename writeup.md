# PID - Control

## The effect of the P, I, D component

### P
Steer in **proportion** to CTE. The larger the CTE is the harder `P*CTE` conponent will contribute to steering angle.

### I
This component is used to eliminate systematic bias. It will be defined as `I*(integral_CTE)`. It will work togather with the integral CTE over time.

But it seems that in the simulator there is not systematic bias.

### D
D is a nice component to reduce oscillations. It will contribute to lower the **change rate** of CTE which helps to smooth the trajectory of the vehicle. Helps to drive the safely.

## How the final hyperparameters were chosen
A twiddle program was implemented, the final hyperparameters are the output of the twiddle procedure.

The twiddle log can be found in file `/pid.log`.
