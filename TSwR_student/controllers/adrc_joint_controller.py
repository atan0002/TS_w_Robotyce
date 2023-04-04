import numpy as np
from observers.eso import ESO
from .controller import Controller
from models.manipulator_model import ManiuplatorModel


class ADRCJointController(Controller):
    def __init__(self, b, kp, kd, omeg, q0, Tp):
        self.b = b
        self.kp = kp
        self.kd = kd
        self.omega_0=omeg

        A = np.array([[0,1,0],[0,0,1],[0,0,0]])
        B = np.array([0,self.b,0])
        L = np.array([3*self.omega_0,3*self.omega_0**2,self.omega_0**3])
        W = np.array([0,1,0])
        self.eso = ESO(A, B, W, L, q0, Tp)
        self.u_n1=0
       



    def set_b(self, b):
        self.b=b
    

    def calculate_control(self, x, q_d, q_d_dot, q_d_ddot):


        q = x[0]
        q_dot = x[1]

      

        states=self.eso.update(q,self.u_n1)
        
        u_star=self.kd*(q_d_dot-states[1])+self.kp*(q_d-states[0])
        
        

        u=(u_star[-1]-states[2])/self.b

        self.u_n1=u[0]

        return u[0]
