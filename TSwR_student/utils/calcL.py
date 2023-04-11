import numpy as np 
from sympy import *


class CalculateL:
    def __init__(self):
        self.lam=symbols('lam')
        self.omega=symbols('self.omega')
        self.bandw=(self.lam+self.omega)
        # sesolution=Pow(band,7)
        self.I00=symbols('self.I00')
        self.I01=symbols('self.I01')
        self.I10=symbols('self.I10')
        self.I11=symbols('self.I11')
        self.N00=symbols('N00')
        self.N01=symbols('N01')
        self.N10=symbols('N10')
        self.N11=symbols('N11')
        self.l1_1=symbols('l1_1')
        self.l1_2=symbols('l1_2')
        self.l1_3=symbols('l1_3')
        self.l1_4=symbols('l1_4')
        self.l1_5=symbols('l1_5')
        self.l1_6=symbols('l1_6')
        # l2_1=symbols('l2_1')
        # l2_2=symbols('l2_2')
        # l2_3=symbols('l2_3')
        # l2_4=symbols('l2_4')
        # l2_5=symbols('l2_5')
        # l2_6=symbols('l2_6')

    


    def solveExpr(self):
       

        

        A=Matrix([[0,0,self.I00,self.I01,0,0],[0,0,self.I10,self.I11,0,0],
                 [0,0,self.N00,self.N01,0,0],[0,0,self.N10,self.N11,0,0],
                 [0,0,0,0,0,0],
                 [0,0,0,0,0,0]])
        
        L=Matrix([[self.l1_1,self.l1_1],[self.l1_2,self.l1_2],[self.l1_3,self.l1_3],[self.l1_4,self.l1_4],[self.l1_5,self.l1_5],[self.l1_6,self.l1_6]])
        C=Matrix([[1,0,0,0,0,0],[1,0,0,0,0,0]])   
        H=A-L*C
        I=eye(6)*self.lam
        Wd=I-H
        print(f'Wd: {Wd}')
        Wd=Wd.det()

        return Wd.as_poly(self.lam).coeffs()#solution.as_poly()
    
    def band(self):
       
        band=Pow((self.lam+self.omega),6)
        return band.as_poly(self.lam).coeffs()


    def main(self):
       
        sol=self.solveExpr()
        # print('sol:',sol)
        sol2=self.band()
        # print(f'band: {sol2}')
        eq=[sol2[0]-sol[0],sol2[1]-sol[1],sol2[2]-sol[2],sol2[3]-sol[3],sol2[4],sol2[5],sol2[6]]
        # print(eq)

        solution=solve(eq,[self.l1_1,self.l1_2,self.l1_3,self.l1_4,self.l1_5,self.l1_6],dict=True)
        print('l1 ',solution)



if __name__ == "__main__":
    calc=CalculateL()
    calc.main()


