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
        self.model_1.update()
        #model 2
        self.model_2=ManiuplatorModel(Tp)
        self.model_2.m3=0.01
        self.model_2.r3=0.01
        self.model_2.update()
        #model 3
        self.model_3=ManiuplatorModel(Tp)
        self.model_3.m3=1.0
        self.model_3.r3=0.3
        self.model_3.update()
    
        self.models = [self.model_1, self.model_2, self.model_3]
        self.i = 0

        self.ksi=1.0
        self.omega_c=50.0
        self.kd=2*self.ksi*self.omega_c
        self.kp=self.omega_c**2
        self.u=np.array([0,0])

    def choose_model(self, x):
        # TODO: Implement procedure of choosing the best fitting model from self.models (by setting self.i)

        q = x[:2]
        q_dot = x[2:]
        x1=self.model_1.esitmate(x,self.u)
        x2=self.model_2.esitmate(x,self.u)
        x3=self.model_3.esitmate(x,self.u)

        # print(self.model_2.m3)

        states_errors=[np.absolute(q_dot-x1),np.absolute(q_dot-x2),np.absolute(q_dot-x3)]

        min_value=states_errors[0]    
        min_index=0
        for i in range(0,len(states_errors)):
            
            if np.less(states_errors[i],min_value).all():

                test=np.less(states_errors[i],min_value)
                
                min_index=i
                min_value=states_errors[i]


                     
        

        self.i=min_index
        


    def calculate_control(self, x, q_r, q_r_dot, q_r_ddot):
        self.choose_model(x)
        print(f'i {self.i}')
        q = x[:2]
        q_dot = x[2:]
        q_dot=q_dot.reshape(2,1)
        v = q_r_ddot.reshape(2,1)+self.kd*(q_r_dot.reshape(2,1)-q_dot.reshape(2,1))+self.kp*(q_r.reshape(2,1)-q.reshape(2,1))
        # v=v.reshape(2,1)
        M = self.models[self.i].M(x)
        C = self.models[self.i].C(x)
        u = M @ v.reshape(2,1) + C @ q_dot
        self.u=u
        return u
