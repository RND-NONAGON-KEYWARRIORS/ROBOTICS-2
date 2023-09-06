# ROBOTICS-2
Ubuntu (/ʊˈbʊntuː/ uu-BUUN-too) is a Linux distribution based on Debian and composed mostly of free and open-source software. Ubuntu is officially released in multiple editions: Desktop, Server, and Core for Internet of things devices and robots. All of the editions can run on a computer alone, or in a virtual machine.

<b>ARTICULATED MODEL</b>


![image](https://github.com/RND-NONAGON-KEYWARRIORS/ROBOTICS-2/assets/143982031/0d058de5-1ad4-45ef-abbd-895762ad86f9)

<b>ARTICULATED CALCULATOR</b>

![image](https://github.com/RND-NONAGON-KEYWARRIORS/ROBOTICS-2/assets/143982031/3290a1b1-a215-46a7-a05a-f5bda8fb64f9)

<b>CODE IN UBUNTO FOR ACTICULATED MANIPULATOR</b>

#MX3201_Company8_Articulated_withVeloCalculator.py

from tkinter import *
from tkinter import  messagebox
from tkinter import PhotoImage
import numpy as np
import math
import roboticstoolbox as rtb
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH
import spatialmath
from spatialmath import SE2
import matplotlib
matplotlib.use('TkAgg')

#Create GUI Window with title
mygui = Tk()
mygui.title('Articulated Calculator')
mygui.resizable(False,False)
mygui.configure(bg='#264653')

def reset():
        a1_E.delete(0,END)
        a2_E.delete(0,END)
        a3_E.delete(0,END)

        T1_E.delete(0,END)
        T2_E.delete(0,END)
        T3_E.delete(0,END)

        Z_E.delete(0,END)
        Y_E.delete(0,END)
        X_E.delete(0,END)

def f_k():
    #link lengths in cm
        a1 = float(a1_E.get())
        a2 = float(a2_E.get())
        a3 = float(a3_E.get())

    #joint variables: if d cm, if Theta degrees
        T1 = float(T1_E.get())
        T2 = float(T2_E.get())
        T3 = float(T3_E.get())

    #degrees to radian
        T1 = (T1/180.0)*np.pi
        T2 = (T2/180.0)*np.pi
        T3 = (T3/180.0)*np.pi

    #Parametric Table (Theta, alpha, r, d)
        PT =   [[T1,(90.0/180.0)*np.pi,0,a1],
                [T2,(0.0/180.0)*np.pi,a2,0],
                [T3,(0.0/180.0)*np.pi,a3,0]]


    #HTM formulae
        i = 0
        H0_1=[[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
      [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
      [0,np.sin(PT[i][1]),np.cos(PT[i][1]),PT[i][3]],
      [0,0,0,1]]

        i = 1
        H1_2=[[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
      [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
      [0,np.sin(PT[i][1]),np.cos(PT[i][1]),PT[i][3]],
      [0,0,0,1]]

        i = 2
        H2_3=[[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
      [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
      [0,np.sin(PT[i][1]),np.cos(PT[i][1]),PT[i][3]],
      [0,0,0,1]]
        
        #Position/Translation Joints
        H0_1 = np.matrix(H0_1)

        H1_2 = np.matrix(H1_2)

        H2_3 = np.matrix(H2_3)

        H0_2 = np.dot(H0_1,H1_2)
        H0_3 = np.dot(H0_2,H2_3)

        X0_3 = H0_3[0,3]
        X_E.delete(0,END)
        X_E.insert(0,np.around(X0_3*100,3))

        Y0_3 = H0_3[1,3]
        Y_E.delete(0,END)
        Y_E.insert(0,np.around(Y0_3*100,3))

        Z0_3 = H0_3[2,3]
        Z_E.delete(0,END)
        Z_E.insert(0,np.around(Z0_3*100,3))

        ##Jacobian Matrix Program

        #Jacobian window

        J_sw = Toplevel()
        J_sw.title('Velocity Calculator')
        J_sw.resizable(False,False)

        #1. Linear/Prismatic Vectors
        Z_1 = [[0],
               [0],
               [1]] # The [0,0,1] vector
        
        #Row 1 to 3, Column 1
        J1a = [[1,0,0],
               [0,1,0],
               [0,0,1]] #R0_0
        J1a = np.dot(J1a,Z_1)

        J1b_1 = H0_3[0:3,3]
        J1b_1 = np.matrix (J1b_1)

        J1b_2 = [[0],[0],[0]]
        J1b_2 = np.matrix (J1b_2)

        J1b = J1b_1 - J1b_2

        J1 = [[(J1a[1,0]*J1b[2,0])-(J1a[2,0]*J1b[1,0])],
              [(J1a[2,0]*J1b[0,0])-(J1a[0,0]*J1b[2,0])],
              [(J1a[0,0]*J1b[1,0])-(J1a[1,0]*J1b[0,0])]]
        
        J1 = np.matrix(J1)

        #Row 1 to 3, Column 2
        J2a = H0_1[0:3,0:3] #R0_1
        J2a = np.dot (J2a,Z_1)

        J2b_1 = H0_3[0:3,3]
        J2b_1 = np.matrix (J2b_1)

        J2b_2 = H0_1[0:3,3]
        J2b_2 = np.matrix (J2b_2)

        J2b = J2b_1 - J2b_2

        J2 = [[(J2a[1,0]*J2b[2,0])-(J2a[2,0]*J2b[1,0])],
              [(J2a[2,0]*J2b[0,0])-(J2a[0,0]*J2b[2,0])],
              [(J2a[0,0]*J2b[1,0])-(J2a[1,0]*J2b[0,0])]]
        
        J2 = np.matrix(J2)

        #Row 1 to 3, Column 3
        J3a = H0_2[0:3,0:3]
        J3a = np.dot (J3a,Z_1)

        J3b_1 = H0_3[0:3,3:]
        J3b_1 = np.matrix (J3b_1)

        J3b_2 = H0_2[0:3,3:]
        J3b_2 = np.matrix (J3b_2)

        J3b = J3b_1 - J3b_2

        J3 = [[(J3a[1,0]*J3b[2,0])-(J3a[2,0]*J3b[1,0])],
              [(J3a[2,0]*J3b[0,0])-(J3a[0,0]*J3b[2,0])],
              [(J3a[0,0]*J3b[1,0])-(J3a[1,0]*J3b[0,0])]]
        
        J3 = np.matrix(J3)

        #2. Rotation/Orientation Vectors

        #Row 4 to 6, Column 1
        J4 = J1a
        J4 = np.matrix(J4)

        #Row 4 to 6, Column 2
        J5 = J2a
        J5 = np.matrix(J5)

        #Row 4 to 6, Column 3
        J6 = J3a
        J6 = np.matrix(J6)

        #3. Concatenated Jacobian Matrix
        JM1 = np.concatenate((J1,J2,J3),1)
        JM2 = np.concatenate((J4,J5,J6),1)

        J = np.concatenate((JM1,JM2),0)
        J = np.matrix(J)

        #Velocity Slide Update
        def update_velo():
                T1p = T1_slider.get()
                T2p = T2_slider.get()
                T3p = T3_slider.get()

                q = np.array([[T1p],[T2p],[T3p]])
                E = np.dot(J,q)

                xp_e = E[0,0]
                x_entry.delete(0,END)
                x_entry.insert(0,str(xp_e))
                
                yp_e = E[1,0]
                y_entry.delete(0,END)
                y_entry.insert(0,str(yp_e))

                zp_e = E[2,0]
                z_entry.delete(0,END)
                z_entry.insert(0,str(zp_e))

                ωx_e = E[3,0]
                ωx_entry.delete(0,END)
                ωx_entry.insert(0,str(ωx_e))

                ωy_e = E[4,0]
                ωy_entry.delete(0,END)
                ωy_entry.insert(0,str(ωy_e))

                ωz_e = E[5,0]
                ωz_entry.delete(0,END)
                ωz_entry.insert(0,str(ωz_e))

        #Jacobian Sliders
        T1_velo = Label(J_sw,text=('θ1* ='),font=(5))
        T1_slider = Scale(J_sw,from_=0,to_=3.142,orient=HORIZONTAL,length=100,sliderlength=10)
        T1_unit = Label(J_sw,text=('rad/s'),font=(5))

        T2_velo = Label(J_sw,text=('θ2* ='),font=(5))
        T2_slider = Scale(J_sw,from_=0,to_=3.142,orient=HORIZONTAL,length=100,sliderlength=10)
        T2_unit = Label(J_sw,text=('rad/s'),font=(5))

        T3_velo = Label(J_sw,text=('θ3* ='),font=(5))
        T3_slider = Scale(J_sw,from_=0,to_=3.142,orient=HORIZONTAL,length=100,sliderlength=10)
        T3_unit = Label(J_sw,text=('rad/s'),font=(5))

        T1_velo.grid(row=0,column=0)
        T1_slider.grid(row=0,column=1)
        T1_unit.grid(row=0,column=2)

        T2_velo.grid(row=1,column=0)
        T2_slider.grid(row=1,column=1)
        T2_unit.grid(row=1,column=2)

        T3_velo.grid(row=2,column=0)
        T3_slider.grid(row=2,column=1)
        T3_unit.grid(row=2,column=2)

        #Jacobian Entries and Labels
        x_velo = Label(J_sw,text=('x* ='),font=(5))
        x_entry = Entry(J_sw,width=10,font=(10))
        x_unit = Label(J_sw,text=('cm/s'),font=(5))
        x_velo.grid(row=3,column=0)
        x_entry.grid(row=3,column=1)
        x_unit.grid(row=3,column=2)

        y_velo = Label(J_sw,text=('y* ='),font=(5))
        y_entry = Entry(J_sw,width=10,font=(10))
        y_unit = Label(J_sw,text=('cm/s'),font=(5))
        y_velo.grid(row=4,column=0)
        y_entry.grid(row=4,column=1)
        y_unit.grid(row=4,column=2)

        z_velo = Label(J_sw,text=('z* ='),font=(5))
        z_entry = Entry(J_sw,width=10,font=(10))
        z_unit = Label(J_sw,text=('cm/s'),font=(5))
        z_velo.grid(row=5,column=0)
        z_entry.grid(row=5,column=1)
        z_unit.grid(row=5,column=2)

        ωx_velo = Label(J_sw,text=('ωx ='),font=(5))
        ωx_entry = Entry(J_sw,width=10,font=(10))
        ωx_unit = Label(J_sw,text=('rad/s'),font=(5))
        ωx_velo.grid(row=6,column=0)
        ωx_entry.grid(row=6,column=1)
        ωx_unit.grid(row=6,column=2)

        ωy_velo = Label(J_sw,text=('ωy ='),font=(5))
        ωy_entry = Entry(J_sw,width=10,font=(10))
        ωy_unit = Label(J_sw,text=('rad/s'),font=(5))
        ωy_velo.grid(row=7,column=0)
        ωy_entry.grid(row=7,column=1)
        ωy_unit.grid(row=7,column=2)

        ωz_velo = Label(J_sw,text=('ωz ='),font=(5))
        ωz_entry = Entry(J_sw,width=10,font=(10))
        ωz_unit = Label(J_sw,text=('rad/s'),font=(5))
        ωz_velo.grid(row=8,column=0)
        ωz_entry.grid(row=8,column=1)
        ωz_unit.grid(row=8,column=2)

        #Update Button
        update_but = Button(J_sw,text='Update',bg='green',fg='white',command=update_velo)
        update_but.grid(row=9,column=0)

    #Create links
    #[robot_variable]=DHRobot([RevoluteDH(d,r,alpha,offset)])
        MX3201_Company8_Articulated_withVeloCalculator = DHRobot([
                RevoluteDH(a1/100,0,(90.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
                RevoluteDH(0,a2/100,(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
                RevoluteDH(0,a3/100,(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
                ], name='Articulated')

        #plot joints
        q1 = np.array([T1,T2,T3])
    
        #plot scale
        x1 = -0.5
        x2 = 0.5
        y1 = -0.5
        y2 = 0.5
        z1 = 0.0
        z2 = 0.5
    
        #Plot Command
        MX3201_Company8_Articulated_withVeloCalculator.plot(q1,limits=[x1,x2,y1,y2,z1,z2],block=True)

def i_k() :
        #Inverse Kinematics using Graphical Method

        #Link lengths in cm
        a1 = float(a1_E.get())
        a2 = float(a2_E.get())
        a3 = float(a3_E.get())

        #Position vector in cm
        xe = float(X_E.get())
        ye = float(Y_E.get())
        ze = float(Z_E.get())

        #To Solve for theta 1 or th1
        if xe == 0:
                Th1 = np.pi/2 if ye > 0 else -np.pi/2
        else:
                Th1 = np.arctan(ye/xe) #1
        
        #To Solve for theta 2 or th2
        r1 = np.sqrt(ye**2 + xe**2) #2
        r2 = ze - a1 #3

        if r1 == 0:
                phi1 = np.pi/2 if r2 > 0 else -np.pi/2
        else:
                phi1 = np.arctan(r2/r1) #4

        r3 = np.sqrt(r2**2 + r1**2) #5
        phi2 = np.arccos(np.clip((a3**2 - a2**2 - r3**2)/(-2*a2*r3), -1, 1)) #6
        Th2 = phi1 + phi2 #7

        #To Solve for theta 3 or th3
        phi3 = np.arccos(np.clip((r3**2 - a2**2 - a3**2)/(-2*a2*a3), -1, 1)) #8
        Th3 = phi3 - np.pi #9

        T1_E.delete(0,END)
        T1_E.insert(0,np.around(Th1*180/np.pi,3))

        T2_E.delete(0,END)
        T2_E.insert(0,np.around(Th2*180/np.pi,3))

        T3_E.delete(0,END)
        T3_E.insert(0,np.around(Th3*180/np.pi,3))

#Create links
    #[robot_variable]=DHRobot([RevoluteDH(d,r,alpha,offset)])
        MX3201_Company8_Articulated_withVeloCalculator = DHRobot([
                RevoluteDH(a1/100,0,(90.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
                RevoluteDH(0,a2/100,(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
                RevoluteDH(0,a3/100,(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
                ], name='Articulated')

        #plot joints
        q1 = np.array([Th1,Th2,Th3])
    
        #plot scale
        x1 = -0.5
        x2 = 0.5
        y1 = -0.5
        y2 = 0.5
        z1 = 0.0
        z2 = 0.5
    
        #Plot Command
        MX3201_Company8_Articulated_withVeloCalculator.plot(q1,limits=[x1,x2,y1,y2,z1,z2],block=True)

#Link lengths and Joint Variable Frame
FI = LabelFrame(mygui,text='Link Lengths and Joint Variables', font=(5),bg='#F4A261')
FI.grid(row=0,column=0)

#Link Lengths Label
a1 = Label(FI,text=('a1 = '),font=(10),bg='#2A9D8F')
a1_E = Entry(FI,width=5,font=(10),bg='#E9C46A')
cm1 = Label(FI,text=('cm '),font=(10),bg='#2A9D8F')

a2 = Label(FI,text=('a2 = '),font=(10),bg='#2A9D8F')
a2_E = Entry(FI,width=5,font=(10),bg='#E9C46A')
cm2 = Label(FI,text=('cm '),font=(10),bg='#2A9D8F')

a3 = Label(FI,text=('a3 = '),font=(10),bg='#2A9D8F')
a3_E = Entry(FI,width=5,font=(10),bg='#E9C46A')
cm3 = Label(FI,text=('cm '),font=(10),bg='#2A9D8F')

a1.grid(row=0,column=0)
a1_E.grid(row=0,column=1)
cm1.grid(row=0,column=2)

a2.grid(row=1,column=0)
a2_E.grid(row=1,column=1)
cm2.grid(row=1,column=2)

a3.grid(row=2,column=0)
a3_E.grid(row=2,column=1)
cm3.grid(row=2,column=2)

#Joint Variable Label
T1 = Label(FI,text=('T1 = '),font=(10),bg='#2A9D8F')
T1_E = Entry(FI,width=5,font=(10),bg='#E9C46A')
deg1 = Label(FI,text=('deg'),font=(10),bg='#2A9D8F')

T2 = Label(FI,text=('T2 = '),font=(10),bg='#2A9D8F')
T2_E = Entry(FI,width=5,font=(10),bg='#E9C46A')
deg2 = Label(FI,text=('deg'),font=(10),bg='#2A9D8F')

T3 = Label(FI,text=('T3 = '),font=(10),bg='#2A9D8F')
T3_E = Entry(FI,width=5,font=(10),bg='#E9C46A')
deg3 = Label(FI,text=('deg'),font=(10),bg='#2A9D8F')

T1.grid(row=0,column=3)
T1_E.grid(row=0,column=4)
deg1.grid(row=0,column=5)

T2.grid(row=1,column=3)
T2_E.grid(row=1,column=4)
deg2.grid(row=1,column=5)

T3.grid(row=2,column=3)
T3_E.grid(row=2,column=4)
deg3.grid(row=2,column=5)

#Buttons Frame
BF = LabelFrame(mygui,text='Forward Kinematics',font=(5))
BF.grid(row=1,column=0)

#Buttons
FK = Button(BF,text='ꜜ Forward',font=(10),bg='blue',fg='white',command=f_k)
rst = Button(BF,text='Reset',font=(10),bg='red',fg='white',command=reset)
IK = Button(BF,text='↑ Inverse',font=(10),bg='green',fg='white',command=i_k)

FK.grid(row=0,column=0)
rst.grid(row=0,column=1)
IK.grid(row=0,column=2)

#Position Vectors Frame
PV = LabelFrame(mygui,text='Position Vectors',font=(5))
PV.grid(row=2,column=0)

#Position Vectors Label
X = Label(PV,text=('X = '),font=(10),bg='#2A9D8F')
X_E = Entry(PV,width=5,font=(10))
cm4 = Label(PV,text=('cm'),font=(10),bg='#2A9D8F')

Y = Label(PV,text=('Y = '),font=(10),bg='#2A9D8F')
Y_E = Entry(PV,width=5,font=(10))
cm5 = Label(PV,text=('cm'),font=(10),bg='#2A9D8F')

Z = Label(PV,text=('Z = '),font=(10),bg='#2A9D8F')
Z_E = Entry(PV,width=5,font=(10))
cm6 = Label(PV,text=('cm'),font=(10),bg='#2A9D8F')

X.grid(row=0,column=0)
X_E.grid(row=0,column=1)
cm4.grid(row=0,column=2)

Y.grid(row=1,column=0)
Y_E.grid(row=1,column=1)
cm5.grid(row=1,column=2)

Z.grid(row=2,column=0)
Z_E.grid(row=2,column=1)
cm6.grid(row=2,column=2)

#insert image
img = PhotoImage(file='ARTICULATED.png')
img = img.subsample(3,3)
PI = Label(mygui,image=img)
PI.grid(row=3,column=0)

mygui.mainloop()


  

