import numpy as np
from models.manipulator_model import ManiuplatorModel
from .controller import Controller


class FeedbackLinearizationController(Controller):
    def __init__(self, Tp , model=None):

        if model is None:
            self.model = ManiuplatorModel(Tp)
        else:
            self.model=model

    def calculate_control(self, x, q_r, q_r_dot, q_r_ddot):
        """
        Please implement the feedback linearization using self.model (which you have to implement also),
        robot state x and desired control v.
        """
        ksi=1.0
        omega_c=5.0
        kd=2*ksi*omega_c
        kp=omega_c**2

        q1, q2, q1_dot, q2_dot = x

        q=np.array([[q1],[q2]])

        q_dot=np.array([[q1_dot],[q2_dot]])

        M=self.model.M(x)
        C=self.model.C(x)
        v=q_r_ddot.reshape(2,1)+kd*(q_r_dot.reshape(2,1)-q_dot)+kp*(q_r.reshape(2,1)-q)
        Tau=M@v.reshape(2,1)+C@q_dot
        



        return Tau
