from copy import copy
import numpy as np


class ESO:
    def __init__(self, A, B, W, L, state, Tp):
        self.A = A
        self.B = B
        self.W = W
        self.L = L
        self.Ad=np.eye(3)+Tp*self.A
        self.Bd=Tp*self.B
        self.Od=Tp*self.L
        self.Bd=self.Bd.reshape(3,1)
        self.Od=self.Od.reshape(3,1)

        self.state = np.pad(np.array(state), (0, A.shape[0] - len(state)))
        self.Tp = Tp
        self.states = np.empty((0,3))
        state=np.array([state[0],state[1],0])
        state=state.reshape(1,3)
        self.states=np.append(self.states,state,axis=0)

        self.state_n1=np.array([0.0,0.0,0.0])
        self.state_n1=self.state_n1.reshape(3,1)
        

    def set_B(self, B):
        self.B = B

    def update(self, q, u):
        # stat=self.states[-1][:]
        # stat=stat.reshape(3,1)
        self.state_n1=self.states[-1][:].reshape(3,1)
        
        
        self.new_state=self.Ad@self.state_n1+self.Bd*u+self.Od*(q[0]-self.state_n1[0]) #



        self.states=np.append(self.states,self.new_state.reshape(1,3),axis=0)

        self.state_n1=self.new_state
        # self.states.append(copy(self.new_state))
        
        return self.new_state 


    def get_state(self):
        return self.states[-1]
