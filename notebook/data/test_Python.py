import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

# System parameters
tau = 1.0  # time constant
K = 10     # gain

# Define the differential equation dx/dt = (K*e(t) - x(t)) / tau
def model(x, t):
    global K, tau
    e_t = 1  # constant input e(t) = 1
    dxdt = (K * e_t - x) / tau
    return dxdt

# Initial conditions
x0 = [0]  # initial condition x(0) = 0
t_span = (0, 5)  # time interval from 0 to 5 seconds
t = np.linspace(0, 5, 500)  # evaluation points

# Solve the differential equation
x = odeint(model, x0, t)

x = np.reshape(x,np.shape(t))

# Calculate the 5% settling time
response_time_possible = t[x >= 9.5]
response_time = response_time_possible.min()

# Display the settling time
print(f"5% Settling time: {response_time:.2f} seconds")

# Plot the system response
plt.plot(t, x, label='System response x(t)')
plt.axhline(y=K, color='r', linestyle='--', label='Steady state')
plt.axhline(y=0.95*K, color='g', linestyle='--', label='5% margin')
plt.axhline(y=1.05*K, color='g', linestyle='--')
plt.axvline(x=response_time, color='b', linestyle='--', label=f'5% Settling time: {response_time:.2f}s')
plt.xlabel('Time (s)')
plt.ylabel('x(t)')
plt.legend()
plt.title('Response of a first-order system')
plt.show()