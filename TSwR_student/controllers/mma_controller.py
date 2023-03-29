import numpy as np
from .controller import Controller

from models.manipulator_model import ManiuplatorModel
# from .controller import feedback_linearization_controller 



class MMAController(Controller):
    def __init__(self, Tp):
        # TODO: Fill the list self.models with 3 models of 2DOF manipulators with different m3 and r3
        # I:   m3=0.1,  r3=0.05
        # II:  m3=0.01, r3=0.01
        # III: m3=1.0,  r3=0.3

        self.Tp=Tp
        #model 1
        self.model_1=ManiuplatorModel(Tp)
        self.model_1.m3=0.1
        self.model_1.r3=0.05
        #model 2
        self.model_2=ManiuplatorModel(Tp)
        self.model_2.m3=0.01
        self.model_2.r3=0.01
        #model 3
        self.model_3=ManiuplatorModel(Tp)
        self.model_3.m3=1.0
        self.model_3.r3=0.3
    
        self.models = [self.model_1, self.model_2, self.model_3]
        self.i = 0

        self.ksi=1.0
        self.omega_c=15.0
        self.kd=2*self.ksi*self.omega_c
        self.kp=self.omega_c**2

    def choose_model(self, x,q_r_ddot):
        # TODO: Implement procedure of choosing the best fitting model from self.models (by setting self.i)

       
        q_dot = x[2:]
        x1=self.model_1.esitmate(x,q_r_ddot)
        x2=self.model_2.esitmate(x,q_r_ddot)
        x3=self.model_3.esitmate(x,q_r_ddot)

        states_errors=[q_dot-x1,q_dot-x2,q_dot-x3]

        min_value=0    
        min_index=0
        for i in range(0,len(states_errors)-1):
            
            if (np.minimum(states_errors[i],states_errors[i+1])< min_value).all():
                
                min_index=i
                     
        

        self.i=min_index


    def calculate_control(self, x, q_r, q_r_dot, q_r_ddot):
        self.choose_model(x,q_r_ddot)
        q = x[:2]
        q_dot = x[2:]
        v = q_r_ddot.reshape(2,1)+self.kd*(q_r_dot.reshape(2,1)-q_dot)+self.kp*(q_r.reshape(2,1)-q)
        M = self.models[self.i].M(x)
        C = self.models[self.i].C(x)
        u = M @ v[:, np.newaxis] + C @ q_dot[:, np.newaxis]
        return u
