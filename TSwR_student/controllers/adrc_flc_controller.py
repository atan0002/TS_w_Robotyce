import numpy as np

# from models.free_model import FreeModel
from observers.eso import ESO
from .adrc_joint_controller import ADRCJointController
from .controller import Controller
# from models.ideal_model import IdealModel
from models.manipulator_model import ManiuplatorModel


class ADRFLController(Controller):
    def __init__(self, Tp, q0, Kp, Kd, p):
        self.manipulator=ManiuplatorModel(Tp)
        self.model = None
        self.Kp = Kp
        self.Kd = Kd
        self.I=np.array([[self.manipulator.I_1,self.manipulator.I_3],[0,self.manipulator.I_2]])
        self.L = None
        W = np.array([self.I,np.zeros(2,4)])
        A = None#np.array([[np.zeros(2,2),self.I,np.zeros(2,2)],[np.zeros(2,2),-self.manipulator.M]])
        B = None
        self.eso = ESO(A, B, W, self.L, q0, Tp)
        self.update_params(q0[:2], q0[2:])

    def update_params(self, q, q_dot):
        ### TODO Implement procedure to set eso.A and eso.B
        x=np.array([q,q_dot])
        M_inv=np.linalg.inv(self.manipulator.M(x))
        self.eso.A = np.array([[np.zeros(2,2),self.I,np.zeros(2,2)],[np.zeros(2,2),-M_inv@self.manipulator.C(x),self.I],
                               [np.zeros(2,2),np.zeros(2,2),np.zeros(2,2)]])
        self.eso.B = self.array([np.zeros(2,2),M_inv,np.zeros(2,2)])

    def calculate_control(self, x, q_d, q_d_dot, q_d_ddot):
        ### TODO implement centralized ADRFLC
        return NotImplementedError
