import numpy as np


class ManiuplatorModel:
    def __init__(self, Tp):
        self.Tp = Tp
        self.l1 = 0.5
        self.r1 = 0.04
        self.m1 = 3.
        self.l2 = 0.4
        self.r2 = 0.04
        self.m2 = 2.4
       
        self.m3 = 0.1
        self.r3 = 0.05
      
        self.d1=self.l1/2
        self.d2=self.l2/2
        self.d2_new=self.d2#(self.d2*self.m2+((self.l2+self.r3)*self.m3))/(self.m2+self.m3)
        self.m_new=self.m2
        self.I_1 = 1 / 12 * self.m1 * (3 * self.r1 ** 2 + self.l1 ** 2)
        self.I_2 = 1 / 12 * self.m2 * (3 * self.r2 ** 2 + self.l2 ** 2) #+ self.m2*(self.d2_new-self.d2)**2
        self.I_3 = 2. / 5 * self.m3 * self.r3 ** 2 #+self.m3*(self.l2-self.d2_new)**2
        self.alpha=self.m1*self.d1**2+self.I_1+self.m2*(self.l1**2+self.d2**2) + self.I_2 + self.m3*(self.l1**2+(self.l2**2))+self.I_3
        self.beta=self.m2*self.l1*self.d2 + self.m3*((self.l2)*self.l1)
        self.gamma=self.m2*self.d2**2 +self.I_2+self.m3*(self.l2)**2+self.I_3
        
        
    def update(self):
        self.alpha=self.m1*self.d1**2+self.I_1+self.m2*(self.l1**2+self.d2**2) + self.I_2 + self.m3*(self.l1**2+(self.l2**2))+self.I_3
        self.beta=self.m2*self.l1*self.d2 + self.m3*((self.l2)*self.l1)
        self.gamma=self.m2*self.d2**2 +self.I_2+self.m3*(self.l2)**2+self.I_3


    def M(self, x):
        """
        Please implement the calculation of the mass matrix, according to the model derived in the exercise
        (2DoF planar manipulator with the object at the tip)
        """
        q1, q2, q1_dot, q2_dot = x


        #Macierz  dla manipulatora z m3
        M = np.array([[float(self.alpha+2*self.beta*np.cos(q2)), float(self.gamma+self.beta*np.cos(q2)) ],[float(self.gamma+self.beta*np.cos(q2)), float(self.gamma)]])
        


        return M

    def C(self, x):
        """
        Please implement the calculation of the Coriolis and centrifugal forces matrix, according to the model derived
        in the exercise (2DoF planar manipulator with the object at the tip)
        """
        q1, q2, q1_dot, q2_dot = x

        #macierz C dla manipulatora bez m3

        C=np.array([[float(-self.beta*np.sin(q2)*q2_dot),float(-self.beta*np.sin(q2)*(q1_dot+q2_dot))],[float(self.beta*np.sin(q2)*q1_dot),0.0]])
        
        
        return C


    def esitmate(self,x,u):

        q = x[:2]
        q_dot = x[2:]
        # x=x.reshape(4,1)
        M=self.M(x)
        C=self.C(x)
        


        M_inv=np.linalg.inv(self.M(x))
      
        x_est=M_inv@u.reshape(2,1)-M_inv@self.C(x)@q_dot.reshape(2,1)
        

        return x_est
