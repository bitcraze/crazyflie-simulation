import matplotlib.pyplot as plt
import numpy as np

def rpm_to_force_np_model(rpm):
    # polyfit using data and scripts from https://github.com/IMRCLab/crazyflie-system-id
    p = [2.55077341e-08, -4.92422570e-05, -1.51910248e-01]
    force_in_grams = np.polyval(p, rpm)
    force_in_newton = force_in_grams * 9.81 / 1000.0
    return np.maximum(force_in_newton, 0)

def rpm_to_force_webots_model(rpm):
    # polyfit using data and scripts from https://github.com/IMRCLab/crazyflie-system-id
    rps = rpm / 60.0
    p = [1e-4, 0, 0]
    force_in_grams = np.polyval(p, rps)
    force_in_newton = force_in_grams * 9.81 / 1000.0
    return np.maximum(force_in_newton, 0)

def pwm_to_rpm(pwm):
    # polyfit using data and scripts from https://github.com/IMRCLab/crazyflie-system-id
    if pwm < 10000:
        return 0
    p = [3.26535711e-01, 3.37495115e+03]
    return np.polyval(p, pwm)

# maximum value of uint16_t
MAX_UINT16 = 65535

pwms = range(0, MAX_UINT16)

rpms = [pwm_to_rpm(p) for p in pwms]

force_np = [rpm_to_force_np_model(rpm) for rpm in rpms]
force_webots = [rpm_to_force_webots_model(rpm) for rpm in rpms]

plt.plot(rpms, force_np)
plt.plot(rpms, force_webots)

# legend
plt.legend(['numpy model', 'webots model'])
plt.title('Force vs RPM')

plt.xlabel('PWM')
plt.ylabel('Force (N)')
plt.show()

print(1e-4*0.006)

