import numpy as np 
from sympy import *


class CalculateL:
    def __init__(self):
        
       pass


    def solveExpr(self):
        lam=symbols('lam')
        omega=symbols('omega')
        band=(lam+omega)
        solution=Pow(band,7)
        I00=symbols('I00')
        I01=symbols('I01')
        I10=symbols('I10')
        I11=symbols('I11')
        N00=symbols('N00')
        N01=symbols('N01')
        N10=symbols('N10')
        N11=symbols('N11')
        l1_1=symbols('l1_1')
        l1_2=symbols('l1_2')
        l1_3=symbols('l1_3')
        l1_4=symbols('l1_4')
        l1_5=symbols('l1_5')
        l1_6=symbols('l1_6')
        l2_1=symbols('l2_1')
        l2_2=symbols('l2_2')
        l2_3=symbols('l2_3')
        l2_4=symbols('l2_4')
        l2_5=symbols('l2_5')
        l2_6=symbols('l2_6')


        

        A=Matrix([[0,0,I00,I01,0,0],[0,0,I10,I11,0,0],
                 [0,0,N00,N01,0,0],[0,0,N10,N11,0,0],
                 [0,0,0,0,0,0],
                 [0,0,0,0,0,0]])
        
        L=Matrix([[l1_1,l2_1],[l1_2,l2_2],[l1_3,l2_3],[l1_4,l2_4],[l1_5,l2_5],[l1_6,l2_6]])
        C=Matrix([[1,0,0,0,0,0],[1,0,0,0,0,0]])   
        H=A-L*C
        I=eye(6)*lam
        Wd=I-H
        Wd=Wd.det()

        return Wd.as_poly(lam).coeffs()#solution.as_poly()
    
    


    def main(self):
        sol=self.solveExpr()
        print(sol)
        # print((sol))


if __name__ == "__main__":
    calc=CalculateL()
    calc.main()


