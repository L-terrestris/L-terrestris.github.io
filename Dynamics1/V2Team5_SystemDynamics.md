```python
%matplotlib inline
```


```python
global_q = False
use_constraints = True
```


```python
!pip install pypoly2tri idealab_tools foldable_robotics pynamics
```

    Requirement already satisfied: pypoly2tri in c:\users\ansah\anaconda3\lib\site-packages (0.0.3)
    Requirement already satisfied: idealab_tools in c:\users\ansah\anaconda3\lib\site-packages (0.0.22)
    Requirement already satisfied: foldable_robotics in c:\users\ansah\anaconda3\lib\site-packages (0.0.29)
    Requirement already satisfied: pynamics in c:\users\ansah\anaconda3\lib\site-packages (0.0.7)
    Requirement already satisfied: pyyaml in c:\users\ansah\anaconda3\lib\site-packages (from foldable_robotics) (5.3.1)
    Requirement already satisfied: matplotlib in c:\users\ansah\anaconda3\lib\site-packages (from foldable_robotics) (3.3.2)
    Requirement already satisfied: shapely in c:\users\ansah\anaconda3\lib\site-packages (from foldable_robotics) (1.7.1)
    Requirement already satisfied: ezdxf in c:\users\ansah\anaconda3\lib\site-packages (from foldable_robotics) (0.15)
    Requirement already satisfied: numpy in c:\users\ansah\anaconda3\lib\site-packages (from foldable_robotics) (1.19.2)
    Requirement already satisfied: imageio in c:\users\ansah\anaconda3\lib\site-packages (from idealab_tools) (2.9.0)
    Requirement already satisfied: scipy in c:\users\ansah\anaconda3\lib\site-packages (from pynamics) (1.5.2)
    Requirement already satisfied: sympy in c:\users\ansah\anaconda3\lib\site-packages (from pynamics) (1.7.1)
    Requirement already satisfied: pyparsing>=2.0.1 in c:\users\ansah\anaconda3\lib\site-packages (from ezdxf->foldable_robotics) (2.4.7)
    Requirement already satisfied: pillow in c:\users\ansah\anaconda3\lib\site-packages (from imageio->idealab_tools) (8.1.0)
    Requirement already satisfied: cycler>=0.10 in c:\users\ansah\anaconda3\lib\site-packages (from matplotlib->foldable_robotics) (0.10.0)
    Requirement already satisfied: python-dateutil>=2.1 in c:\users\ansah\anaconda3\lib\site-packages (from matplotlib->foldable_robotics) (2.8.1)
    Requirement already satisfied: kiwisolver>=1.0.1 in c:\users\ansah\anaconda3\lib\site-packages (from matplotlib->foldable_robotics) (1.3.0)
    Requirement already satisfied: certifi>=2020.06.20 in c:\users\ansah\anaconda3\lib\site-packages (from matplotlib->foldable_robotics) (2020.12.5)
    Requirement already satisfied: six in c:\users\ansah\anaconda3\lib\site-packages (from cycler>=0.10->matplotlib->foldable_robotics) (1.15.0)
    Requirement already satisfied: mpmath>=0.19 in c:\users\ansah\anaconda3\lib\site-packages (from sympy->pynamics) (1.1.0)
    


```python
# -*- coding: utf-8 -*-
"""
Written by Daniel M. Aukes
Email: danaukes<at>gmail.com
Please see LICENSE for full license.
"""

import pynamics
from pynamics.frame import Frame
from pynamics.variable_types import Differentiable,Constant
from pynamics.system import System
from pynamics.body import Body
from pynamics.dyadic import Dyadic
from pynamics.output import Output,PointsOutput
from pynamics.particle import Particle
import pynamics.integration
import sympy
import numpy
import scipy.optimize
import matplotlib.pyplot as plt
plt.ion()
from math import pi
```


```python
system = System()
pynamics.set_system(__name__,system)
```


```python

```

# Proposed System Kinematics v2





![](https://drive.google.com/uc?export=view&id=1H8GI8lkvuWy7_2MEZjq6CaDyLAzc0f6J)





## Parameterization

### Constants


```python
h = Constant(0.0381, 'h',system) #height off the ground due to the front plate (can be set to 0 in the future)
l0 = Constant(0.0508,'l0',system)
la = Constant(0.0254,'la',system)
lb = Constant(0.0254,'lb',system)
lc = Constant(0.0254,'lc',system)
ld = Constant(0.0254,'ld',system)

#Changed to smaller weight to be more reasonable with the system
mA = Constant(.01,'mA',system)
mB = Constant(.01,'mB',system)
mC = Constant(.01,'mC',system)
mD = Constant(.01,'mC',system)


g = Constant(9.81,'g',system)
b = Constant(1e1,'b',system) #damping
k = Constant(1e1,'k',system) #spring force

#spring preloads to system
preload1 = Constant(20*pi/180,'preload1',system) # At-rest angle of NA (A w.r.t. N)
preload2 = Constant(-40*pi/180,'preload2',system) # At-rest angle of AB (B w.r.t. A)
preload3 = Constant(-40*pi/180,'preload3',system) # At-rest angle of DC (C w.r.t. D)
preload4 = Constant(20*pi/180,'preload4',system) # At-rest angle of ND (D w.r.t. N)
 



Ixx_A = Constant(1,'Ixx_A',system)
Iyy_A = Constant(1,'Iyy_A',system)
Izz_A = Constant(1,'Izz_A',system)
Ixx_B = Constant(1,'Ixx_B',system)
Iyy_B = Constant(1,'Iyy_B',system)
Izz_B = Constant(1,'Izz_B',system)
Ixx_C = Constant(1,'Ixx_C',system)
Iyy_C = Constant(1,'Iyy_C',system)
Izz_C = Constant(1,'Izz_C',system)
Ixx_D = Constant(1,'Ixx_D',system)
Iyy_D = Constant(1,'Iyy_D',system)
Izz_D = Constant(1,'Izz_D',system)

```


```python
tol = 1e-12
```


```python
#Animation stuff
tinitial = 0
tfinal = 10
fps = 30
tstep = 1/fps
t = numpy.r_[tinitial:tfinal:tstep]
```


```python
# Creating dynamic state variables. system argument denotes them as state variables for pynamics system from above
qA,qA_d,qA_dd = Differentiable('qA',system) #Angle between N and A
qB,qB_d,qB_dd = Differentiable('qB',system) #from AB
qC,qC_d,qC_dd = Differentiable('qC',system)
qD,qD_d,qD_dd = Differentiable('qD',system) #from DE

```


```python
#Initial condition based geometrically off of a solidworks model
#given 85 and 170 puts it into the 'compressed' state
#2bar
initialvalues = {}
# Setting initial angle values. See above for angle positions
#Listed are important angle configurations
#Smallest(Compressed) 85,170
#Middle ~45,90
#Largest(Extended) 0,0
initialvalues[qA]=85*pi/180  #45
initialvalues[qA_d]=0*pi/180 
initialvalues[qB]=-170*pi/180   #-90
initialvalues[qB_d]=0*pi/180

initialvalues[qC]=170*pi/180 #90
initialvalues[qC_d]=0*pi/180
initialvalues[qD]=-85*pi/180   #-45
initialvalues[qD_d]=0*pi/180
```


```python
statevariables = system.get_state_variables() # tried running this after vector calculation like i kinematics, but no apparent change...
ini0 = [initialvalues[item] for item in statevariables]
```


```python
N = Frame('N') # Initializing frames
A = Frame('A')
B = Frame('B')
C = Frame('C')
D = Frame('D')
```


```python
system.set_newtonian(N)
```


```python
#if not global_q:
#set rotation logic of frames. Kept all z at one and left the negative value in the initial condition calculation
#changing this caused errors in the system
A.rotate_fixed_axis_directed(N,[0,0,1],qA,system) # A reference frame rotates about N's ref frame in Z direction by qA amount
B.rotate_fixed_axis_directed(A,[0,0,1],qB,system)
C.rotate_fixed_axis_directed(D,[0,0,1],qC,system)
D.rotate_fixed_axis_directed(N,[0,0,1],qD,system)

#rotate each about the global reference frame. Add if else later to see which method is simpler
#else:
#  A.rotate_fixed_axis_directed(N,[0,0,1],qA,system) # A reference frame rotates about N's ref frame in Z direction by qA amount
#  B.rotate_fixed_axis_directed(N,[0,0,1],qB,system)
#  C.rotate_fixed_axis_directed(N,[0,0,1],qC,system)
#  D.rotate_fixed_axis_directed(N,[0,0,1],qD,system)
```

Vectors

claculated by using solving the two 2bar problem. Each vector containts the length constant and the previous point given the local reference frame. 


```python

pA = 0*N.x + (l0 + h)*N.y + 0*N.z

pB = pA + la*A.x
pBtip = pB + lb*B.x

pD = 0*N.x + h*N.y + 0*N.z
pC = pD + ld*D.x
pCtip = pC + lc*C.x

```

Center of mass

Considered the midpoint of each link as they will be uniform in size and material


```python
pAcm=pA+(la/2)*A.x
pBcm=pB+(lb/2)*B.x
pCcm=pC+(lc/2)*C.x
pDcm = pD + (ld/2)*D.x
pBCcm = pCtip + (l0/2)*N.y #get end effector 
```

Angular Velocity


```python
wNA = N.getw_(A) #Angular velocity of vector A w.r.t. N
wAB = A.getw_(B) #Angular velocity of vector B w.r.t. A
wDC = D.getw_(C) #Angular velocity of vector C w.r.t. D
wND = N.getw_(D) #Angular velocity of vector D w.r.t. N
```


```python
#vCtip = pCtip.time_derivative(N,system)
```

Define Inertias & Bodies


```python
IA = Dyadic.build(A,Ixx_A,Iyy_A,Izz_A)
IB = Dyadic.build(B,Ixx_B,Iyy_B,Izz_B)
IC = Dyadic.build(C,Ixx_C,Iyy_C,Izz_C)
ID = Dyadic.build(D,Ixx_D,Iyy_D,Izz_D)


BodyA = Body('BodyA',A,pAcm,mA,IA,system)
BodyB = Body('BodyB',B,pBcm,mB,IB,system)
BodyC = Body('BodyC',C,pCcm,mC,IC,system)
BodyD = Body('BodyD',D,pDcm,mD,ID,system)
BodyBCp = Particle(pBCcm,mB,'ParticleBC',system) #would we also need bodyE 
```

Forces & Torques

Defines the system forces, mainly relying on the spring force


```python
system.addforce(-b*wNA,wNA)
system.addforce(-b*wAB,wAB)
system.addforce(-b*wDC,wDC)
system.addforce(-b*wND,wND)

```




    <pynamics.force.Force at 0x1ad87f0ed30>




```python
##########################################
#Shouldn't we only consider spring force on one point ? 
#########################################
#if not global_q:
system.add_spring_force1(k,(qA-preload1)*N.z,wNA)
system.add_spring_force1(k,(qB-preload2)*A.z,wAB)
system.add_spring_force1(k,(qC-preload3)*D.z,wDC)
system.add_spring_force1(k,(qD-preload4)*N.z,wND)
  
#else:
#    system.add_spring_force1(k,(qA-preload1)*N.z,wNA) 
#    system.add_spring_force1(k,(qB-qA-preload2)*N.z,wAB)
#    system.add_spring_force1(k,(qC-qD-preload3)*N.z,wDC)
#    system.add_spring_force1(k,(qD-preload4)*N.z,wND)
```




    (<pynamics.force.Force at 0x1ad87f9e6d0>,
     <pynamics.spring.Spring at 0x1ad87f9e970>)




```python
system.addforcegravity(-g*N.y)
```

Constraints

Constrains the system based off of our system kinematics. (see image above for proposed system kinematics v2)


```python
#Define the closed loop kinematics of the four bar linkage.
#eq_vector = pB - pC
eq_vector1 = pCtip - pBtip

#overconstriants 
eq_vector2 = pBtip - pA
#eq_vector3 = pCtip - pD
```


```python
eq = []
if use_constraints:
    eq.append((eq_vector1).dot(N.x)) #same x value for B and C tip
    eq.append((eq_vector1).length() - l0) #length constraint between B and C tip
    #eq.append((eq_vector).dot(N.x)) #B and C have same x 

#overconstraints test
    eq.append((eq_vector2).dot(N.y)) #pBtip and pA have same y value (move in x)
    #eq.append((eq_vector3).dot(N.y))
    
eq_d=[(system.derivative(item)) for item in eq]
eq_dd=[(system.derivative(item)) for item in eq_d]
```


```python
qi = [qA]
qd = [qB,qC,qD]
```


```python
constants = system.constant_values.copy() # Recalls link lengths declared near beginning
defined = dict([(item,initialvalues[item]) for item in qi])
constants.update(defined)
```


```python
eq = [item.subs(constants) for item in eq]
```


```python
error = (numpy.array(eq)**2).sum()
```


```python
f = sympy.lambdify(qd,error)

def function(args):
    return f(*args)
```


```python
guess = [initialvalues[item] for item in qd]
```


```python
result = scipy.optimize.minimize(function,guess)
#result <- able to call result in kinematics but not here for some reason
```


```python
ini = []
for item in system.get_state_variables():
    if item in qd:
        ini.append(result.x[qd.index(item)])
    else:
        ini.append(initialvalues[item])
```


```python
points = [pA,pB,pBtip,pCtip,pC,pD]
#points_output = PointsOutput(points,system)
points_output = PointsOutput(points, constant_values=system.constant_values)
#points.calc(numpy.array([ini0,ini]))
```


```python
#ini
```


```python
#ini0 # ini and ini0 returning same values
```

All of this code is from system kinematics. Having this verifies the initial coniditon is valid and correct



```python
points = PointsOutput(points, constant_values=system.constant_values)
points.calc(numpy.array([ini0,ini]))
points.plot_time()
```

    2021-02-28 02:37:39,916 - pynamics.output - INFO - calculating outputs
    2021-02-28 02:37:40,028 - pynamics.output - INFO - done calculating outputs
    


    
![png](output_47_1.png)
    


F = ma


```python
f,ma = system.getdynamics()
```

    2021-02-28 02:37:40,223 - pynamics.system - INFO - getting dynamic equations
    


```python
#f
```


```python
#ma
```

Solve for Acceleration 


```python
func1,lambda1 = system.state_space_post_invert(f,ma,eq_dd,return_lambda = True)
```

    2021-02-28 02:37:40,450 - pynamics.system - INFO - solving a = f/m and creating function
    2021-02-28 02:37:40,453 - pynamics.system - INFO - substituting constrained in Ma-f.
    2021-02-28 02:37:43,815 - pynamics.system - INFO - done solving a = f/m and creating function
    2021-02-28 02:37:43,816 - pynamics.system - INFO - calculating function for lambdas
    

Integrate


```python
states=pynamics.integration.integrate(func1,ini,t,rtol=tol,atol=tol, args=({'constants':system.constant_values},))
```

    2021-02-28 02:37:43,881 - pynamics.integration - INFO - beginning integration
    2021-02-28 02:37:43,883 - pynamics.system - INFO - integration at time 0000.00
    2021-02-28 02:37:51,269 - pynamics.system - INFO - integration at time 0004.83
    2021-02-28 02:37:53,462 - pynamics.integration - INFO - finished integration
    

Outputs

We can see all the angle values appraoch 0 over time. For us 0 referes to the most extended state of the sarrus linkage


```python
plt.figure()
artists = plt.plot(t,states[:,:4])
plt.legend(artists,['qA','qB','qC','qD'])
```




    <matplotlib.legend.Legend at 0x1ad89896d60>




    
![png](output_58_1.png)
    


Energy

Energy decreases to 0 as the system comes to rest


```python
KE = system.get_KE()
PE = system.getPEGravity(pA) - system.getPESprings()
energy_output = Output([KE-PE],system)
energy_output.calc(states)
energy_output.plot_time()
```

    2021-02-28 02:37:53,720 - pynamics.output - INFO - calculating outputs
    2021-02-28 02:37:53,730 - pynamics.output - INFO - done calculating outputs
    


    
![png](output_60_1.png)
    


Motion

The system springs open from compressed to extended. This is how we intend to use springs in our system


```python
#points = [pA,pB,pBtip,pCtip,pC,pD]
#points_output = PointsOutput(points,system)
y = points_output.calc(states)
points_output.plot_time(20)
```

    2021-02-28 02:37:53,839 - pynamics.output - INFO - calculating outputs
    2021-02-28 02:37:53,851 - pynamics.output - INFO - done calculating outputs
    


    
![png](output_62_1.png)
    



```python
points_output.animate(fps = fps,movie_name = 'render.mp4',lw=2,marker='o',color=(1,0,0,1),linestyle='-')
```


    
![png](output_63_0.png)
    



```python
from matplotlib import animation, rc
from IPython.display import HTML
HTML(points_output.anim.to_html5_video())
```




<video width="432" height="288" controls autoplay loop>
  <source type="video/mp4" src="data:video/mp4;base64,AAAAIGZ0eXBNNFYgAAACAE00ViBpc29taXNvMmF2YzEAAAAIZnJlZQAAgMhtZGF0AAACoAYF//+c
3EXpvebZSLeWLNgg2SPu73gyNjQgLSBjb3JlIDE2MSAtIEguMjY0L01QRUctNCBBVkMgY29kZWMg
LSBDb3B5bGVmdCAyMDAzLTIwMjAgLSBodHRwOi8vd3d3LnZpZGVvbGFuLm9yZy94MjY0Lmh0bWwg
LSBvcHRpb25zOiBjYWJhYz0xIHJlZj0zIGRlYmxvY2s9MTowOjAgYW5hbHlzZT0weDM6MHgxMTMg
bWU9aGV4IHN1Ym1lPTcgcHN5PTEgcHN5X3JkPTEuMDA6MC4wMCBtaXhlZF9yZWY9MSBtZV9yYW5n
ZT0xNiBjaHJvbWFfbWU9MSB0cmVsbGlzPTEgOHg4ZGN0PTEgY3FtPTAgZGVhZHpvbmU9MjEsMTEg
ZmFzdF9wc2tpcD0xIGNocm9tYV9xcF9vZmZzZXQ9LTIgdGhyZWFkcz05IGxvb2thaGVhZF90aHJl
YWRzPTEgc2xpY2VkX3RocmVhZHM9MCBucj0wIGRlY2ltYXRlPTEgaW50ZXJsYWNlZD0wIGJsdXJh
eV9jb21wYXQ9MCBjb25zdHJhaW5lZF9pbnRyYT0wIGJmcmFtZXM9MyBiX3B5cmFtaWQ9MiBiX2Fk
YXB0PTEgYl9iaWFzPTAgZGlyZWN0PTEgd2VpZ2h0Yj0xIG9wZW5fZ29wPTAgd2VpZ2h0cD0yIGtl
eWludD0yNTAga2V5aW50X21pbj0yNSBzY2VuZWN1dD00MCBpbnRyYV9yZWZyZXNoPTAgcmNfbG9v
a2FoZWFkPTQwIHJjPWNyZiBtYnRyZWU9MSBjcmY9MjMuMCBxY29tcD0wLjYwIHFwbWluPTAgcXBt
YXg9NjkgcXBzdGVwPTQgaXBfcmF0aW89MS40MCBhcT0xOjEuMDAAgAAACF9liIQAL//+9q78yytH
C5UuHVl7s1Hy6Ely/YgwfWgAAAMAANVp5QZSk1YjvoAAHKEwd6WB86PYATKkYEkyGZeTG+rDQRGh
otygo4hBwqDzG1hGVbSpGO32N+GtpZXHPFTQTcg6D+3+omu8QOJxKf4tQEESwxHLobqZ/eh9S+8v
hzMc5qVWy8VI4TZXfm8p/TTGyRwjo4fbZyURD5zZSQeOFxtncs39LB4ZVaqnN/jr5LRLSiciqgOY
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
AAAAAQAACAAAAAACAAACAAAAAAEAAAgAAAAAAgAAAgAAAAABAAAGAAAAAAEAAAIAAAAAAQAABgAA
AAABAAACAAAAAAEAAAgAAAAAAgAAAgAAAAABAAAKAAAAAAEAAAQAAAAAAQAAAAAAAAABAAACAAAA
AAEAAAoAAAAAAQAABAAAAAABAAAAAAAAAAEAAAIAAAAAAQAACgAAAAABAAAEAAAAAAEAAAAAAAAA
AQAAAgAAAAABAAAKAAAAAAEAAAQAAAAAAQAAAAAAAAABAAACAAAAAAEAAAYAAAAAAQAAAgAAAAAC
AAAEAAAAAAEAAAoAAAAAAQAABAAAAAABAAAAAAAAAAEAAAIAAAAAAQAABgAAAAABAAACAAAAAAEA
AAgAAAAAAgAAAgAAAAABAAAKAAAAAAEAAAQAAAAAAQAAAAAAAAABAAACAAAAAAEAAAoAAAAAAQAA
BAAAAAABAAAAAAAAAAEAAAIAAAAAAQAACAAAAAACAAACAAAAAAEAAAoAAAAAAQAABAAAAAABAAAA
AAAAAAEAAAIAAAAAAQAACgAAAAABAAAEAAAAAAEAAAAAAAAAAQAAAgAAAAABAAAKAAAAAAEAAAQA
AAAAAQAAAAAAAAABAAACAAAAAAEAAAoAAAAAAQAABAAAAAABAAAAAAAAAAEAAAIAAAAAAQAACgAA
AAABAAAEAAAAAAEAAAAAAAAAAQAAAgAAAAABAAAKAAAAAAEAAAQAAAAAAQAAAAAAAAABAAACAAAA
AAEAAAoAAAAAAQAABAAAAAABAAAAAAAAAAEAAAIAAAAAAQAACgAAAAABAAAEAAAAAAEAAAAAAAAA
AQAAAgAAAAABAAAKAAAAAAEAAAQAAAAAAQAAAAAAAAABAAACAAAAAAQAAAQAAAAAAQAACgAAAAAB
AAAEAAAAAAEAAAAAAAAAAQAAAgAAAAADAAAEAAAAAAEAAAoAAAAAAQAABAAAAAABAAAAAAAAAAEA
AAIAAAAAAQAABgAAAAABAAACAAAAAAEAAAYAAAAAAQAAAgAAAAABAAAGAAAAAAEAAAIAAAAAAQAA
BgAAAAABAAACAAAAAAEAAAYAAAAAAQAAAgAAAAABAAAIAAAAAAIAAAIAAAAAAQAABgAAAAABAAAC
AAAAAAEAAAYAAAAAAQAAAgAAAAABAAAKAAAAAAEAAAQAAAAAAQAAAAAAAAABAAACAAAAAAEAAAoA
AAAAAQAABAAAAAABAAAAAAAAAAEAAAIAAAAAAQAACgAAAAABAAAEAAAAAAEAAAAAAAAAAQAAAgAA
AAABAAAKAAAAAAEAAAQAAAAAAQAAAAAAAAABAAACAAAAAAEAAAoAAAAAAQAABAAAAAABAAAAAAAA
AAEAAAIAAAAAAQAACgAAAAABAAAEAAAAAAEAAAAAAAAAAQAAAgAAAAABAAAKAAAAAAEAAAQAAAAA
AQAAAAAAAAABAAACAAAAAAEAAAoAAAAAAQAABAAAAAABAAAAAAAAAAEAAAIAAAAAAQAACgAAAAAB
AAAEAAAAAAEAAAAAAAAAAQAAAgAAAAABAAAKAAAAAAEAAAQAAAAAAQAAAAAAAAABAAACAAAAAAEA
AAoAAAAAAQAABAAAAAABAAAAAAAAAAEAAAIAAAAAAQAACgAAAAABAAAEAAAAAAEAAAAAAAAAAQAA
AgAAAAABAAAKAAAAAAEAAAQAAAAAAQAAAAAAAAABAAACAAAAAAEAAAoAAAAAAQAABAAAAAABAAAA
AAAAAAEAAAIAAAAAAQAACgAAAAABAAAEAAAAAAEAAAAAAAAAAQAAAgAAAAABAAAKAAAAAAEAAAQA
AAAAAQAAAAAAAAABAAACAAAAAAEAAAoAAAAAAQAABAAAAAABAAAAAAAAAAEAAAIAAAAAAQAACgAA
AAABAAAEAAAAAAEAAAAAAAAAAQAAAgAAAAABAAAKAAAAAAEAAAQAAAAAAQAAAAAAAAABAAACAAAA
AAEAAAoAAAAAAQAABAAAAAABAAAAAAAAAAEAAAIAAAAAAQAACgAAAAABAAAEAAAAAAEAAAAAAAAA
AQAAAgAAAAABAAAKAAAAAAEAAAQAAAAAAQAAAAAAAAABAAACAAAAAAEAAAoAAAAAAQAABAAAAAAB
AAAAAAAAAAEAAAIAAAAAAQAACgAAAAABAAAEAAAAAAEAAAAAAAAAAQAAAgAAAAABAAAKAAAAAAEA
AAQAAAAAAQAAAAAAAAABAAACAAAAAAEAAAoAAAAAAQAABAAAAAABAAAAAAAAAAEAAAIAAAAAAQAA
CgAAAAABAAAEAAAAAAEAAAAAAAAAAQAAAgAAAAABAAAKAAAAAAEAAAQAAAAAAQAAAAAAAAABAAAC
AAAAAAEAAAQAAAAAAQAACgAAAAABAAAEAAAAAAEAAAAAAAAAAQAAAgAAAAABAAAKAAAAAAEAAAQA
AAAAAQAAAAAAAAABAAACAAAAAAEAAAoAAAAAAQAABAAAAAABAAAAAAAAAAEAAAIAAAAAAQAACgAA
AAABAAAEAAAAAAEAAAAAAAAAAQAAAgAAAAABAAAKAAAAAAEAAAQAAAAAAQAAAAAAAAABAAACAAAA
AAEAAAoAAAAAAQAABAAAAAABAAAAAAAAAAEAAAIAAAAAAQAACgAAAAABAAAEAAAAAAEAAAAAAAAA
AQAAAgAAAAABAAAKAAAAAAEAAAQAAAAAAQAAAAAAAAABAAACAAAAAAEAAAoAAAAAAQAABAAAAAAB
AAAAAAAAAAEAAAIAAAAAAQAACgAAAAABAAAEAAAAAAEAAAAAAAAAAQAAAgAAAAABAAAKAAAAAAEA
AAQAAAAAAQAAAAAAAAABAAACAAAAAAEAAAoAAAAAAQAABAAAAAABAAAAAAAAAAEAAAIAAAAAAQAA
BAAAAAAcc3RzYwAAAAAAAAABAAAAAQAAASwAAAABAAAExHN0c3oAAAAAAAAAAAAAASwAAAsHAAAC
bwAAAb8AAAF9AAABsQAAAXwAAAFQAAACDQAAAJsAAAE6AAABiwAAAdEAAADOAAAB+wAAALUAAAIH
AAAAsAAAArYAAAC7AAAAlgAAAlQAAAC0AAAAgAAAAgoAAACPAAACDwAAAHcAAAJDAAAAhgAAAGsA
AAJiAAAAfgAAAFMAAABiAAACYgAAAKIAAABKAAAAaQAAAfcAAACOAAAAbQAAAGEAAAInAAAAmQAA
AGkAAABOAAABiwAAADkAAAENAAABaQAAAe4AAACOAAAAMwAAAEQAAAExAAAAIgAAAZwAAAA/AAAA
MQAAAV4AAABaAAAAKwAAAC8AAAFwAAAAYAAAACoAAAAoAAAA8AAAAEcAAAAfAAABMQAAAE8AAAAg
AAAAJQAAARIAAABMAAAAKAAAACEAAADtAAAARwAAACQAAAAkAAABEAAAADEAAAAqAAAAIAAAANAA
AAA8AAAAMQAAAB0AAAC2AAAAKwAAACYAAAAZAAAAvAAAADgAAAAmAAAAGwAAALsAAAAmAAAAIgAA
ABkAAACzAAAAIgAAAB0AAAAZAAAAVgAAAGwAAAByAAAAWwAAAK8AAAAdAAAAGAAAABUAAAA8AAAA
WwAAAF0AAAClAAAAHQAAABsAAAATAAAAbgAAABYAAABrAAAAFgAAAG4AAAATAAAAcwAAABMAAABP
AAAADwAAAHsAAAAZAAAADwAAAFcAAAAUAAAAaQAAABIAAACCAAAAEwAAABYAAAARAAAAbwAAAB0A
AAAYAAAAFQAAAGsAAAAeAAAAFgAAABEAAABPAAAAGwAAABEAAAARAAAAZgAAABsAAAAVAAAAFQAA
AGMAAAAdAAAAFQAAABYAAABIAAAAGwAAABMAAAATAAAASwAAAB4AAAATAAAAFwAAAGcAAAAdAAAA
GAAAABEAAABgAAAAGwAAABQAAAARAAAAMQAAABUAAAARAAAAEQAAAFYAAAAZAAAAEwAAABMAAAAz
AAAAFwAAABMAAAARAAAAMAAAABkAAAAVAAAAEwAAAFEAAAAeAAAAGAAAABEAAAAnAAAAGQAAABEA
AAARAAAAKAAAABcAAAARAAAAEQAAACMAAAAXAAAAEQAAABEAAAA3AAAAGwAAABMAAAARAAAAJQAA
ABcAAAARAAAAEQAAACkAAAAXAAAAEQAAABEAAAAeAAAAFwAAABEAAAARAAAAGAAAABcAAAARAAAA
EQAAABsAAAAZAAAAEwAAABEAAAAyAAAAFwAAABEAAAARAAAAIQAAABcAAAARAAAAEQAAADoAAAAZ
AAAAEQAAABEAAAAiAAAAFwAAABEAAAARAAAJYAAAAK0AAAAeAAAAHgAAABQAAAAmAAAAFwAAABUA
AAAPAAAAGAAAABEAAAAPAAAADwAAABgAAAARAAAADwAAAA8AAAAYAAAAEQAAAA8AAAAPAAAAGAAA
ABEAAAAPAAAADwAAABgAAAARAAAADwAAAA8AAAAYAAAAEQAAAA8AAAAPAAAAGAAAABEAAAAPAAAA
DwAAABgAAAARAAAADwAAAA8AAAAXAAAAEQAAAA8AAAAPAAAAFwAAABEAAAAPAAAADwAAABgAAAAU
c3RjbwAAAAAAAAABAAAAMAAAAGJ1ZHRhAAAAWm1ldGEAAAAAAAAAIWhkbHIAAAAAAAAAAG1kaXJh
cHBsAAAAAAAAAAAAAAAALWlsc3QAAAAlqXRvbwAAAB1kYXRhAAAAAQAAAABMYXZmNTguNDUuMTAw
">
  Your browser does not support the video tag.
</video>



Next Steps:

How to control the spring force to 'oscillate' around a point

Contain the spring so that is settles before the fully extended position to prevent singularities. 

Factor in a cable (actuator) force to oppose spring force


