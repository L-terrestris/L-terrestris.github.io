---
title: System Dynamics
---

# System Dynamics

## Description
[Assignment Instructions](https://egr557.github.io/assignments/system-dynamics-I.html)

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

    Requirement already satisfied: pypoly2tri in /usr/local/lib/python3.7/dist-packages (0.0.3)
    Requirement already satisfied: idealab_tools in /usr/local/lib/python3.7/dist-packages (0.0.22)
    Requirement already satisfied: foldable_robotics in /usr/local/lib/python3.7/dist-packages (0.0.29)
    Requirement already satisfied: pynamics in /usr/local/lib/python3.7/dist-packages (0.0.8)
    Requirement already satisfied: imageio in /usr/local/lib/python3.7/dist-packages (from idealab_tools) (2.4.1)
    Requirement already satisfied: numpy in /usr/local/lib/python3.7/dist-packages (from foldable_robotics) (1.19.5)
    Requirement already satisfied: shapely in /usr/local/lib/python3.7/dist-packages (from foldable_robotics) (1.7.1)
    Requirement already satisfied: pyyaml in /usr/local/lib/python3.7/dist-packages (from foldable_robotics) (3.13)
    Requirement already satisfied: matplotlib in /usr/local/lib/python3.7/dist-packages (from foldable_robotics) (3.2.2)
    Requirement already satisfied: ezdxf in /usr/local/lib/python3.7/dist-packages (from foldable_robotics) (0.15.2)
    Requirement already satisfied: scipy in /usr/local/lib/python3.7/dist-packages (from pynamics) (1.4.1)
    Requirement already satisfied: sympy in /usr/local/lib/python3.7/dist-packages (from pynamics) (1.7.1)
    Requirement already satisfied: pillow in /usr/local/lib/python3.7/dist-packages (from imageio->idealab_tools) (7.0.0)
    Requirement already satisfied: kiwisolver>=1.0.1 in /usr/local/lib/python3.7/dist-packages (from matplotlib->foldable_robotics) (1.3.1)
    Requirement already satisfied: pyparsing!=2.0.4,!=2.1.2,!=2.1.6,>=2.0.1 in /usr/local/lib/python3.7/dist-packages (from matplotlib->foldable_robotics) (2.4.7)
    Requirement already satisfied: cycler>=0.10 in /usr/local/lib/python3.7/dist-packages (from matplotlib->foldable_robotics) (0.10.0)
    Requirement already satisfied: python-dateutil>=2.1 in /usr/local/lib/python3.7/dist-packages (from matplotlib->foldable_robotics) (2.8.1)
    Requirement already satisfied: mpmath>=0.19 in /usr/local/lib/python3.7/dist-packages (from sympy->pynamics) (1.2.1)
    Requirement already satisfied: six in /usr/local/lib/python3.7/dist-packages (from cycler>=0.10->matplotlib->foldable_robotics) (1.15.0)
    


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





![](systemkinematicsv2.PNG)





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




    <pynamics.force.Force at 0x7efdccdb2210>




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




    (<pynamics.force.Force at 0x7efdccd4eb50>,
     <pynamics.spring.Spring at 0x7efdccd4eb10>)




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

    2021-02-27 17:38:53,309 - pynamics.output - INFO - calculating outputs
    2021-02-27 17:38:53,311 - pynamics.output - INFO - done calculating outputs
    




    <matplotlib.axes._subplots.AxesSubplot at 0x7efdccbd2d10>




    
![png](output_47_2.png)
    


F = ma


```python
f,ma = system.getdynamics()
```

    2021-02-27 17:38:53,510 - pynamics.system - INFO - getting dynamic equations
    


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

    2021-02-27 17:38:53,906 - pynamics.system - INFO - solving a = f/m and creating function
    2021-02-27 17:38:53,912 - pynamics.system - INFO - substituting constrained in Ma-f.
    2021-02-27 17:38:59,095 - pynamics.system - INFO - done solving a = f/m and creating function
    2021-02-27 17:38:59,097 - pynamics.system - INFO - calculating function for lambdas
    

Integrate


```python
states=pynamics.integration.integrate(func1,ini,t,rtol=tol,atol=tol, args=({'constants':system.constant_values},))
```

    2021-02-27 17:38:59,129 - pynamics.integration - INFO - beginning integration
    2021-02-27 17:38:59,131 - pynamics.system - INFO - integration at time 0000.00
    2021-02-27 17:39:08,032 - pynamics.system - INFO - integration at time 0004.83
    2021-02-27 17:39:10,709 - pynamics.integration - INFO - finished integration
    

Outputs

We can see all the angle values appraoch 0 over time. For us 0 referes to the most extended state of the sarrus linkage


```python
plt.figure()
artists = plt.plot(t,states[:,:4])
plt.legend(artists,['qA','qB','qC','qD'])
```




    <matplotlib.legend.Legend at 0x7efdcc68aa90>




    
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

    2021-02-27 17:39:11,047 - pynamics.output - INFO - calculating outputs
    2021-02-27 17:39:11,061 - pynamics.output - INFO - done calculating outputs
    


    
![png](output_60_1.png)
    


Motion

The system springs open from compressed to extended. This is how we intend to use springs in our system


```python
#points = [pA,pB,pBtip,pCtip,pC,pD]
#points_output = PointsOutput(points,system)
y = points_output.calc(states)
points_output.plot_time(20)
```

    2021-02-27 17:39:11,226 - pynamics.output - INFO - calculating outputs
    2021-02-27 17:39:11,245 - pynamics.output - INFO - done calculating outputs
    




    <matplotlib.axes._subplots.AxesSubplot at 0x7efdccd63cd0>




    
![png](output_62_2.png)
    



```python
points_output.animate(fps = fps,movie_name = 'render.mp4',lw=2,marker='o',color=(1,0,0,1),linestyle='-')
```




    <matplotlib.axes._subplots.AxesSubplot at 0x7efdc9961390>




    
![png](output_63_1.png)
    



```python
from matplotlib import animation, rc
from IPython.display import HTML
HTML(points_output.anim.to_html5_video())
```




<video width="432" height="288" controls autoplay loop>
  <source type="video/mp4" src="data:video/mp4;base64,AAAAHGZ0eXBNNFYgAAACAGlzb21pc28yYXZjMQAAAAhmcmVlAACDcW1kYXQAAAKuBgX//6rcRem9
5tlIt5Ys2CDZI+7veDI2NCAtIGNvcmUgMTUyIHIyODU0IGU5YTU5MDMgLSBILjI2NC9NUEVHLTQg
QVZDIGNvZGVjIC0gQ29weWxlZnQgMjAwMy0yMDE3IC0gaHR0cDovL3d3dy52aWRlb2xhbi5vcmcv
eDI2NC5odG1sIC0gb3B0aW9uczogY2FiYWM9MSByZWY9MyBkZWJsb2NrPTE6MDowIGFuYWx5c2U9
MHgzOjB4MTEzIG1lPWhleCBzdWJtZT03IHBzeT0xIHBzeV9yZD0xLjAwOjAuMDAgbWl4ZWRfcmVm
PTEgbWVfcmFuZ2U9MTYgY2hyb21hX21lPTEgdHJlbGxpcz0xIDh4OGRjdD0xIGNxbT0wIGRlYWR6
b25lPTIxLDExIGZhc3RfcHNraXA9MSBjaHJvbWFfcXBfb2Zmc2V0PS0yIHRocmVhZHM9MyBsb29r
YWhlYWRfdGhyZWFkcz0xIHNsaWNlZF90aHJlYWRzPTAgbnI9MCBkZWNpbWF0ZT0xIGludGVybGFj
ZWQ9MCBibHVyYXlfY29tcGF0PTAgY29uc3RyYWluZWRfaW50cmE9MCBiZnJhbWVzPTMgYl9weXJh
bWlkPTIgYl9hZGFwdD0xIGJfYmlhcz0wIGRpcmVjdD0xIHdlaWdodGI9MSBvcGVuX2dvcD0wIHdl
aWdodHA9MiBrZXlpbnQ9MjUwIGtleWludF9taW49MjUgc2NlbmVjdXQ9NDAgaW50cmFfcmVmcmVz
aD0wIHJjX2xvb2thaGVhZD00MCByYz1jcmYgbWJ0cmVlPTEgY3JmPTIzLjAgcWNvbXA9MC42MCBx
cG1pbj0wIHFwbWF4PTY5IHFwc3RlcD00IGlwX3JhdGlvPTEuNDAgYXE9MToxLjAwAIAAAAhQZYiE
AC///vau/MsrRwuVLh1Ze7NR8uhJcv2IMH1oAAADAADVaeUGUpNWI76AAByhMHelgfOj2AEypGBJ
MhmXkxvqw0ERoaLcoKOIQcKg8xtYRlW0qRjt9jfhraWVxzxU0E3IOg/t/qJrvEDicSn+LUBBEsMR
y6G6mf3ofUvvL4czHOalVsvFSOE2V35b4QiaY2Z4AFOddHza2ORwcdFu1ihFykK+rgMHXY0v5wGv
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
AAAAAAAAAAAAAAEAAAAAAAAAAAAAAAAAAEAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAC
AAAP1HRyYWsAAABcdGtoZAAAAAMAAAAAAAAAAAAAAAEAAAAAAAAnEAAAAAAAAAAAAAAAAAAAAAAA
AQAAAAAAAAAAAAAAAAAAAAEAAAAAAAAAAAAAAAAAAEAAAAABsAAAASAAAAAAACRlZHRzAAAAHGVs
c3QAAAAAAAAAAQAAJxAAAAQAAAEAAAAAD0xtZGlhAAAAIG1kaGQAAAAAAAAAAAAAAAAAADwAAAJY
AFXEAAAAAAAtaGRscgAAAAAAAAAAdmlkZQAAAAAAAAAAAAAAAFZpZGVvSGFuZGxlcgAAAA73bWlu
ZgAAABR2bWhkAAAAAQAAAAAAAAAAAAAAJGRpbmYAAAAcZHJlZgAAAAAAAAABAAAADHVybCAAAAAB
AAAOt3N0YmwAAACzc3RzZAAAAAAAAAABAAAAo2F2YzEAAAAAAAAAAQAAAAAAAAAAAAAAAAAAAAAB
sAEgAEgAAABIAAAAAAAAAAEAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAY//8AAAAx
YXZjQwFkABX/4QAYZ2QAFazZQbCWhAAAAwAEAAADAPA8WLZYAQAGaOvjyyLAAAAAHHV1aWRraEDy
XyRPxbo5pRvPAyPzAAAAAAAAABhzdHRzAAAAAAAAAAEAAAEsAAACAAAAABhzdHNzAAAAAAAAAAIA
AAABAAAA+wAACNhjdHRzAAAAAAAAARkAAAAHAAAEAAAAAAEAAAYAAAAAAQAAAgAAAAACAAAEAAAA
AAEAAAYAAAAAAQAAAgAAAAABAAAGAAAAAAEAAAIAAAAAAQAABgAAAAABAAACAAAAAAEAAAgAAAAA
AgAAAgAAAAABAAAIAAAAAAIAAAIAAAAAAQAABgAAAAABAAACAAAAAAEAAAYAAAAAAQAAAgAAAAAB
AAAIAAAAAAIAAAIAAAAAAQAACgAAAAABAAAEAAAAAAEAAAAAAAAAAQAAAgAAAAABAAAKAAAAAAEA
AAQAAAAAAQAAAAAAAAABAAACAAAAAAEAAAoAAAAAAQAABAAAAAABAAAAAAAAAAEAAAIAAAAAAQAA
CgAAAAABAAAEAAAAAAEAAAAAAAAAAQAAAgAAAAABAAAGAAAAAAEAAAIAAAAAAgAABAAAAAABAAAK
AAAAAAEAAAQAAAAAAQAAAAAAAAABAAACAAAAAAEAAAYAAAAAAQAAAgAAAAABAAAIAAAAAAIAAAIA
AAAAAQAACgAAAAABAAAEAAAAAAEAAAAAAAAAAQAAAgAAAAABAAAKAAAAAAEAAAQAAAAAAQAAAAAA
AAABAAACAAAAAAEAAAgAAAAAAgAAAgAAAAABAAAKAAAAAAEAAAQAAAAAAQAAAAAAAAABAAACAAAA
AAEAAAoAAAAAAQAABAAAAAABAAAAAAAAAAEAAAIAAAAAAQAACgAAAAABAAAEAAAAAAEAAAAAAAAA
AQAAAgAAAAABAAAKAAAAAAEAAAQAAAAAAQAAAAAAAAABAAACAAAAAAEAAAoAAAAAAQAABAAAAAAB
AAAAAAAAAAEAAAIAAAAAAQAACgAAAAABAAAEAAAAAAEAAAAAAAAAAQAAAgAAAAABAAAKAAAAAAEA
AAQAAAAAAQAAAAAAAAABAAACAAAAAAEAAAoAAAAAAQAABAAAAAABAAAAAAAAAAEAAAIAAAAAAQAA
CgAAAAABAAAEAAAAAAEAAAAAAAAAAQAAAgAAAAAEAAAEAAAAAAEAAAoAAAAAAQAABAAAAAABAAAA
AAAAAAEAAAIAAAAAAwAABAAAAAABAAAKAAAAAAEAAAQAAAAAAQAAAAAAAAABAAACAAAAAAEAAAYA
AAAAAQAAAgAAAAABAAAGAAAAAAEAAAIAAAAAAQAABgAAAAABAAACAAAAAAEAAAYAAAAAAQAAAgAA
AAABAAAGAAAAAAEAAAIAAAAAAQAACAAAAAACAAACAAAAAAEAAAYAAAAAAQAAAgAAAAABAAAGAAAA
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
AAAAAAEAAAoAAAAAAQAABAAAAAABAAAAAAAAAAEAAAIAAAAAAQAACgAAAAABAAAEAAAAAAEAAAAA
AAAAAQAAAgAAAAABAAAKAAAAAAEAAAQAAAAAAQAAAAAAAAABAAACAAAAAAEAAAoAAAAAAQAABAAA
AAABAAAAAAAAAAEAAAIAAAAAAQAACgAAAAABAAAEAAAAAAEAAAAAAAAAAQAAAgAAAAABAAAEAAAA
AAEAAAoAAAAAAQAABAAAAAABAAAAAAAAAAEAAAIAAAAAAQAACgAAAAABAAAEAAAAAAEAAAAAAAAA
AQAAAgAAAAABAAAKAAAAAAEAAAQAAAAAAQAAAAAAAAABAAACAAAAAAEAAAoAAAAAAQAABAAAAAAB
AAAAAAAAAAEAAAIAAAAAAQAACgAAAAABAAAEAAAAAAEAAAAAAAAAAQAAAgAAAAABAAAKAAAAAAEA
AAQAAAAAAQAAAAAAAAABAAACAAAAAAEAAAoAAAAAAQAABAAAAAABAAAAAAAAAAEAAAIAAAAAAQAA
CgAAAAABAAAEAAAAAAEAAAAAAAAAAQAAAgAAAAABAAAKAAAAAAEAAAQAAAAAAQAAAAAAAAABAAAC
AAAAAAEAAAoAAAAAAQAABAAAAAABAAAAAAAAAAEAAAIAAAAAAQAACgAAAAABAAAEAAAAAAEAAAAA
AAAAAQAAAgAAAAABAAAKAAAAAAEAAAQAAAAAAQAAAAAAAAABAAACAAAAAAEAAAQAAAAAHHN0c2MA
AAAAAAAAAQAAAAEAAAEsAAAAAQAABMRzdHN6AAAAAAAAAAAAAAEsAAALBgAAAlsAAAHPAAABjwAA
AY8AAAGGAAABZwAAAhEAAACQAAABPgAAAYMAAAHNAAAA3gAAAeIAAACgAAACAQAAAKUAAALCAAAA
qgAAAI0AAAI8AAAAmgAAAH4AAAHwAAAAhwAAAhAAAABqAAACTQAAAJ8AAABuAAACUAAAAIgAAABg
AAAAbwAAAlwAAAC0AAAAUwAAAFAAAAHwAAAAggAAAF0AAABcAAACOAAAAJoAAABqAAAAWQAAAY0A
AAA8AAABAAAAAVIAAAIOAAAAjAAAAEQAAABKAAABMwAAACYAAAGjAAAARwAAADIAAAFUAAAAXgAA
ACkAAAA4AAABhQAAAGAAAAAnAAAAIwAAAPQAAABKAAAAHgAAARwAAABGAAAAJwAAACIAAAEKAAAA
RAAAAB0AAAAWAAABDgAAADwAAAAjAAAAJAAAASMAAABAAAAAJwAAABgAAADVAAAAOwAAAC8AAAAp
AAAAwwAAAD4AAAApAAAAHAAAAMEAAAAzAAAAKgAAACYAAADeAAAANQAAACEAAAAgAAAAuAAAACkA
AAAfAAAAHgAAAHgAAABvAAAAWwAAAG8AAAC6AAAAJgAAABgAAAAZAAAARQAAAGMAAABuAAAAsAAA
ACQAAAAUAAAAGQAAAGoAAAAaAAAAXQAAABcAAAB1AAAAFQAAAHEAAAATAAAASAAAABEAAACDAAAA
FQAAABQAAABeAAAAFgAAAIcAAAAUAAAAeQAAAB4AAAAWAAAAFgAAAGYAAAAeAAAAFwAAABYAAABe
AAAAHQAAABQAAAAUAAAAfAAAACAAAAAXAAAAGAAAAGMAAAAhAAAAGgAAABgAAACAAAAAJgAAAB4A
AAAYAAAASAAAAB4AAAAXAAAAFgAAAFQAAAAfAAAAGAAAABUAAABBAAAAHgAAABUAAAAVAAAAUAAA
AB0AAAAVAAAAFgAAAEYAAAAfAAAAFwAAABQAAAAvAAAAHAAAABQAAAAUAAAAPAAAAB0AAAAUAAAA
FAAAAE8AAAAfAAAAFwAAABUAAAA+AAAAHgAAABUAAAAXAAAAKQAAAB0AAAAWAAAAFAAAADIAAAAc
AAAAFAAAABQAAABDAAAAHgAAABYAAAAUAAAAIQAAAB4AAAAVAAAAFAAAADUAAAAcAAAAFAAAABQA
AAAbAAAAHgAAABYAAAAUAAAAGAAAABwAAAAUAAAAFAAAAB8AAAAcAAAAFAAAABQAAAAYAAAAHAAA
ABQAAAAUAAAAGAAAABwAAAAUAAAAFAAAAEEAAAAfAAAAFAAAABQAAAAyAAAAHAAAABQAAAAUAAAA
JQAAABwAAAAUAAAAFAAACY8AAAB6AAAAHAAAABkAAAAWAAAALgAAABwAAAAXAAAAEwAAABgAAAAY
AAAAEwAAABMAAAAYAAAAGAAAABMAAAATAAAAGAAAABgAAAATAAAAEwAAABgAAAAYAAAAEwAAABMA
AAAYAAAAGAAAABMAAAATAAAAGAAAABgAAAATAAAAEwAAABgAAAAYAAAAEwAAABMAAAAYAAAAGAAA
ABMAAAATAAAAFwAAABgAAAATAAAAEwAAABcAAAAYAAAAEwAAABMAAAAYAAAAFHN0Y28AAAAAAAAA
AQAAACwAAABidWR0YQAAAFptZXRhAAAAAAAAACFoZGxyAAAAAAAAAABtZGlyYXBwbAAAAAAAAAAA
AAAAAC1pbHN0AAAAJal0b28AAAAdZGF0YQAAAAEAAAAATGF2ZjU3LjgzLjEwMA==
">
  Your browser does not support the video tag.
</video>



Next Steps:

How to control the spring force to 'oscillate' around a point

Contain the spring so that is settles before the fully extended position to prevent singularities. 

Factor in a cable (actuator) force to oppose spring force




```python

```
