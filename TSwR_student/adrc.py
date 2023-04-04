import matplotlib.pyplot as plt
import numpy as np
from numpy import pi
from scipy.integrate import odeint

from controllers.adrc_controller import ADRController

from trajectory_generators.constant_torque import ConstantTorque
from trajectory_generators.sinusonidal import Sinusoidal
from trajectory_generators.poly3 import Poly3
from utils.simulation import simulate

Tp = 0.001
end = 5

# traj_gen = ConstantTorque(np.array([0., 1.0])[:, np.newaxis])
traj_gen = Sinusoidal(np.array([0., 1.]), np.array([2., 2.]), np.array([0., 0.]))
# traj_gen = Poly3(np.array([0., 0.]), np.array([pi/4, pi/6]), end)

omega_o=60
omegac=0.3*omega_o
ksi=1

b_est_1 =2
b_est_2 =20
kp_est_1 =omegac**2
kp_est_2 = omegac**2
kd_est_1 = 2*ksi*omegac
kd_est_2 = 2*ksi*omegac
p1 = -1
p2 = -1

q0, qdot0, _ = traj_gen.generate(0.)
q1_0 = np.array([q0[0], qdot0[0]])
q2_0 = np.array([q0[1], qdot0[1]])
controller = ADRController(Tp=Tp,params=[[b_est_1, kp_est_1, kd_est_1, omega_o, q1_0],
                                       [b_est_2, kp_est_2, kd_est_2, omega_o, q2_0]])

Q, Q_d, u, T = simulate("PYBULLET", traj_gen, controller, Tp, end)

sta=controller.joint_controllers[0].eso.states

eso1 = controller.joint_controllers[0].eso.states[0:-1]
eso2 = controller.joint_controllers[1].eso.states[0:-1]

plt.subplot(221)
plt.plot(T, eso1[:, 0])
plt.plot(T, Q[:, 0], 'r')
plt.subplot(222)
plt.plot(T, eso1[:, 1])
plt.plot(T, Q[:, 2], 'r')
plt.subplot(223)
plt.plot(T, eso2[:, 0])
plt.plot(T, Q[:, 1], 'r')
plt.subplot(224)
plt.plot(T, eso2[:, 1])
plt.plot(T, Q[:, 3], 'r')
plt.show()

plt.subplot(221)
plt.plot(T, Q[:, 0], 'r')
plt.plot(T, Q_d[:, 0], 'b')
plt.subplot(222)
plt.plot(T, Q[:, 1], 'r')
plt.plot(T, Q_d[:, 1], 'b')
plt.subplot(223)
plt.plot(T, u[:, 0], 'r')
plt.plot(T, u[:, 1], 'b')
plt.show()
