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
        self.I=np.ones(2,2)#np.array([[self.manipulator.I_1,self.manipulator.I_3],[0,self.manipulator.I_2]])
        self.L = None
        self.W = np.array([self.I,np.zeros(2,4)]).reshape(6,2)
        self.A = None#np.array([[np.zeros(2,2),self.I,np.zeros(2,2)],[np.zeros(2,2),-self.manipulator.M]])
        self.B = None
        self.eso = ESO(self.A, self.B, self.W, self.L, q0, Tp)
        self.omega=p
        self.update_params(q0[:2], q0[2:])
        self.Tp=Tp
        self.x_est_n1=np.zeros(6,1)
        self.u=np.zeros(2,1)
        self.model_est=ManiuplatorModel(Tp)
        self.omegac=30
        self.ksi=1
        self.Kp = self.omegac**2
        self.Kd = 2*self.ksi*self.omegac


    def update_params(self, q, q_dot):
        ### TODO Implement procedure to set eso.A and eso.B
        x=np.array([q,q_dot])
        M_inv=np.linalg.inv(self.manipulator.M(x))
        N=-M_inv@self.manipulator.C(x)
    
        self.l1= N[0,0]/2+N[1,1]/2+3*self.omega
        self.l3=(N[0,0]**3*self.I[0,1] - N[0,0]**2*N[0,1]*self.I[0,0] + 6*N[0,0]**2*self.I,[0,1]*self.omega + 
                 2*N[0,0]*N[0,1]*N[1,0]*self.I[0,1] - N[0,0]*N[0,1]*N[1,1]*self.I[0,0] - 6*N[0,0]*N[0,1]*self.I[0,0]*self.omega + 
                 15*N[0,0]*self.I[0,1]*self.omega**2 - N[0,1]**2*N[1,0]*self.I[0,0] + N[0,1]*N[1,0]*N[1,1]*self.I[0,1] + 6*N[0,1]*N[1,0]*self.I[0,1]*self.omega - 
                 N[0,1]*N[1,1]**2*self.I[0,0] - 6*N[0,1]*N[1,1]*self.I[0,0]*self.omega - 15*N[0,1]*self.I[0,0]*self.omega**2 + 20*self.I[0,1]*self.omega**3)/(2*N[0,0]*self.I[0,0]*self.I[0,1] - 2*N[0,1]*self.I[0,0]**2 + 2*N[1,0]*self.I[0,1]**2 - 2*N[1,1]*self.I[0,0]*self.I[0,1])
        
        self.l4=(N[0,0]**2*N[1,0]*self.I[0,1] - N[0,0]*N[0,1]*N[1,0]*self.I[0,0] + N[0,0]*N[1,0]*N[1,1]*self.I[0,1] + 6*N[0,0]*N[1,0]*self.I01*self.omega + N[0,1]*N[1,0]**2*self.I[0,1] - 
                 2*N[0,1]*N[1,0]*N[1,1]*self.I[0,0] - 6*N[0,1]*N[1,0]*self.I[0,0]*self.omega + N[1,0]*N[1,1]**2*self.I[0,1] + 
                 6*N[1,0]*N[1,1]*self.I[0,1]*self.omega + 15*N[1,0]*self.I[0,1]*self.omega**2 - N[1,1]**3*self.I[0,0] - 6*N[1,1]**2*self.I[0,0]*self.omega -
                   15*N[1,1]*self.I[0,0]*self.omega**2 - 20*self.I[0,0]*self.omega**3)/(2*N[0,0]*self.I[0,0]*self.I[0,1] - 2*N[0,1]*self.I[0,0]**2 + 2*N[1,0]*self.I[0,1]**2 - 2*N[1,1]*self.I[0,0]*self.I[0,1])
        
       
        self.L=np.array([self.l1,self.l1],[0,0],[self.l3,self.l3],[self.l4,self.l4],[0,0],[0,0])

        self.eso.A = np.array([[np.zeros(2,2),self.I,np.zeros(2,2)],[np.zeros(2,2),-M_inv@self.manipulator.C(x),self.I],
                               [np.zeros(2,2),np.zeros(2,2),np.zeros(2,2)]])
        self.eso.B = self.array([np.zeros(2,2),M_inv,np.zeros(2,2)])

    def calculate_control(self, x, q_d, q_d_dot, q_d_ddot):

        q=x[:2]
        q_dot=x[2:]
        
        self.Ad=np.eye(6)+self.Tp*self.A
        self.Bd=self.Tp*self.B
        self.Od=self.Tp*self.L
        


        x_est=self.Ad@self.x_est_n1+self.Bd*self.u+self.Od*(q-self.W@self.x_est_n1)

        v=self.Kp*(q_d.reshape(2,1)-q)+ self.Kd*(q_d_dot.reshape(2,1)-q_dot)+q_d_ddot


        u=self.model_est.M(x_est)@(v-x_est[4:])+self.model_est.C(x_est)@x_est[2:4]


        self.update_params(x_est[:2],x_est[2:4])
        
        return u
