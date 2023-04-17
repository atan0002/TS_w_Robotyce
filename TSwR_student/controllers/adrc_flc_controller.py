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
        self.Kp = np.array([Kp[0,0],Kp[1,1]])
        self.Kd = np.array([Kd[0,0],Kd[1,1]])
        self.omega=p
        self.Tp=Tp
        self.I=np.eye(2)#np.array([[self.manipulator.I_1,self.manipulator.I_3],[0,self.manipulator.I_2]])
        self.L = np.array([
            [3*self.omega[0],0],
            [0,3*self.omega[1]],
            [3*self.omega[0]**2,0],
            [0,3*self.omega[1]**2],
            [self.omega[0]**3,0],
            [0,self.omega[1]**3]
            
            ])

        self.Od=self.Tp*self.L
        zer=np.zeros((4,2))
        self.W = np.concatenate((self.I,zer),axis=0)
        self.W=self.W.reshape(6,2)
        self.A = None
        self.B = None
        
        
        
        self.states = np.empty((0,6))
        state=np.zeros((1,6))
        self.states=np.append(self.states,state,axis=0)
    
        q=np.array([0.0,0.0])
        q_dot=np.array([0.0,0.0])
        self.update_params(q, q_dot)
        self.Tp=Tp
        self.x_est_n1=np.zeros((6,1))
        self.u=np.zeros((2,1))
        self.model_est=ManiuplatorModel(Tp)
      


    def update_params(self, q, q_dot):
       
        x=np.array([q,q_dot])
        x=x.reshape(4,1)
        M=self.manipulator.M(x)
        M_inv=np.linalg.inv(M)
        N=-M_inv@self.manipulator.C(x)
    

        self.A = np.array([[np.zeros((2,2)),self.I,np.zeros((2,2))],[np.zeros((2,2)),-M_inv@self.manipulator.C(x),self.I],
                               [np.zeros((2,2)),np.zeros((2,2)),np.zeros((2,2))]])

        self.A= np.array([[0,0,1,0,0,0],
                          [0,0,0,1,0,0],
                          [0,0,N[0,0],N[0,1],1,0],
                          [0,0,N[1,0],N[1,1],0,1],
                          [0,0,0,0,0,0],
                          [0,0,0,0,0,0]
                          ])

        self.B=np.array([[0,0],
                         [0,0],
                         [M_inv[0,0],M_inv[0,1]],
                         [M_inv[1,0],M_inv[1,1]],
                         [0,0],
                         [0,0]                         
                         ])

    def calculate_control(self, x, q_d, q_d_dot, q_d_ddot):

        q=x[:2]
        q_dot=x[2:]
        
        self.Ad=np.eye(6)+self.Tp*self.A
        self.Bd=self.Tp*self.B
       
      

        x_est=self.Ad@self.x_est_n1+self.Bd@self.u+self.Od@(q-self.W.reshape(2,6)@self.x_est_n1)

        v=self.Kp.reshape(2,1)*(q_d.reshape(2,1)-q.reshape(2,1))+ self.Kd.reshape(2,1)*(q_d_dot.reshape(2,1)-q_dot.reshape(2,1))+q_d_ddot.reshape(2,1)

        x_ev=np.array([x_est[0,0],x_est[1,1],x_est[2,0],x_est[3,1]])
        f_est=np.array([x_est[4,0],x_est[5,1]])
        q_est_dot=x_ev[2:]

        # test=self.model_est.M(x_ev)@(v.reshape(2,1)-f_est.reshape(2,1))
        # test1=self.model_est.C(x_ev)@q_est_dot.reshape(2,1)

        u=self.model_est.M(x_ev)@(v.reshape(2,1)-f_est.reshape(2,1))+self.model_est.C(x_ev)@q_est_dot.reshape(2,1)

        self.update_params(x_ev[:2],x_ev[2:4])

        state=np.concatenate((x_ev.reshape(1,4), f_est.reshape(1,2)),axis=1)

        self.states=np.append(self.states,state,axis=0)
        
        return u
