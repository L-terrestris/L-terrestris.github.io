---
title: System Kinematics
---

# System Kinematics
## Description 
[Assignment Instructions](https://egr557.github.io/assignments/system-kinematics.html)

## Team Response

Team 5

Gilgal Ansah
gjansah@asu.edu

Javon Grimes
jdgrime1@asu.edu

Jonathan Nguyen
jrnguyen@asu.edu

Jacob Sindorf 
jsindorf@asu.edu


## Device Figure

### System Kinematics

![](https://drive.google.com/uc?export=view&id=1H8GI8lkvuWy7_2MEZjq6CaDyLAzc0f6J)

## Paper Model 

The following paper model was designed using the desired lengths as seen in the systems kinematics figure. It takes 4 sarrus linkages, all with four joint arms of equal length, to allow for translational movement in one direction. 

To simplify the kinematics, only one sarrus linkage is considered, on a 2D plane. 

![picture](papermodel.PNG)

## Kinematic Model

Code of the kinematic model (see HW step 3)


```python
!pip install pypoly2tri idealab_tools foldable_robotics pynamics
```

    Collecting pypoly2tri
      Downloading https://files.pythonhosted.org/packages/11/33/079f13c72507fb2015e990f463f1766a756be1563a7e370e948dd2cbaafb/pypoly2tri-0.0.3-py2.py3-none-any.whl
    Collecting idealab_tools
      Downloading https://files.pythonhosted.org/packages/c7/00/d34a536ff276a23e347e903ad52d14625d87411dff587d87fbeff33fbef2/idealab_tools-0.0.22-py2.py3-none-any.whl
    Collecting foldable_robotics
      Downloading https://files.pythonhosted.org/packages/5d/af/a1cef62e833cca7060da3f1f25dd2f7b6afc7a9a3735862dcc12ff73b644/foldable_robotics-0.0.29-py2.py3-none-any.whl
    Collecting pynamics
    [?25l  Downloading https://files.pythonhosted.org/packages/4e/ef/c2d877610893cf7ed50c369ae5889bc119cb3c796ee678f3025fb4f2f8d6/pynamics-0.0.8-py2.py3-none-any.whl (87kB)
    [K     |████████████████████████████████| 92kB 4.4MB/s 
    [?25hRequirement already satisfied: imageio in /usr/local/lib/python3.7/dist-packages (from idealab_tools) (2.4.1)
    Requirement already satisfied: shapely in /usr/local/lib/python3.7/dist-packages (from foldable_robotics) (1.7.1)
    Requirement already satisfied: numpy in /usr/local/lib/python3.7/dist-packages (from foldable_robotics) (1.19.5)
    Requirement already satisfied: pyyaml in /usr/local/lib/python3.7/dist-packages (from foldable_robotics) (3.13)
    Collecting ezdxf
    [?25l  Downloading https://files.pythonhosted.org/packages/6c/67/1a6715d910cd4051e135c386df391fd45720fcf0c526f313958c3e43fe16/ezdxf-0.15.2-cp37-cp37m-manylinux2010_x86_64.whl (1.8MB)
    [K     |████████████████████████████████| 1.8MB 17.7MB/s 
    [?25hRequirement already satisfied: matplotlib in /usr/local/lib/python3.7/dist-packages (from foldable_robotics) (3.2.2)
    Requirement already satisfied: scipy in /usr/local/lib/python3.7/dist-packages (from pynamics) (1.4.1)
    Requirement already satisfied: sympy in /usr/local/lib/python3.7/dist-packages (from pynamics) (1.7.1)
    Requirement already satisfied: pillow in /usr/local/lib/python3.7/dist-packages (from imageio->idealab_tools) (7.0.0)
    Requirement already satisfied: pyparsing>=2.0.1 in /usr/local/lib/python3.7/dist-packages (from ezdxf->foldable_robotics) (2.4.7)
    Requirement already satisfied: cycler>=0.10 in /usr/local/lib/python3.7/dist-packages (from matplotlib->foldable_robotics) (0.10.0)
    Requirement already satisfied: python-dateutil>=2.1 in /usr/local/lib/python3.7/dist-packages (from matplotlib->foldable_robotics) (2.8.1)
    Requirement already satisfied: kiwisolver>=1.0.1 in /usr/local/lib/python3.7/dist-packages (from matplotlib->foldable_robotics) (1.3.1)
    Requirement already satisfied: mpmath>=0.19 in /usr/local/lib/python3.7/dist-packages (from sympy->pynamics) (1.2.1)
    Requirement already satisfied: six in /usr/local/lib/python3.7/dist-packages (from cycler>=0.10->matplotlib->foldable_robotics) (1.15.0)
    Installing collected packages: pypoly2tri, idealab-tools, ezdxf, foldable-robotics, pynamics
    Successfully installed ezdxf-0.15.2 foldable-robotics-0.0.29 idealab-tools-0.0.22 pynamics-0.0.8 pypoly2tri-0.0.3
    


```python
%matplotlib inline
```


```python
import pynamics
from pynamics.frame import Frame
from pynamics.variable_types import Differentiable,Constant
from pynamics.system import System
#from pynamics.body import Body
#from pynamics.dyadic import Dyadic
from pynamics.output import Output,PointsOutput
#from pynamics.particle import Particle
import pynamics.integration
import sympy
import numpy
import matplotlib.pyplot as plt
plt.ion()
from math import pi
#from pynamics.constraint import Constraint
import scipy.optimize
import math as m
```


```python
system = System()
pynamics.set_system(__name__,system)
```

Creates the required lengths of each link and labels them  l0 to l5. The sizes of the links are the same as seen in the paper model and diagram above converted to meters. Visually we left them as inches as it is easier to read. 


```python
h = Constant(0.0381, 'h',system) #height off the ground due to the front plate
l0 = Constant(0.0508,'l0',system)
la = Constant(0.0254,'la',system)
lb = Constant(0.0254,'lb',system)
lc = Constant(0.0254,'lc',system)
ld = Constant(0.0254,'ld',system)
```

Our system will be a sideways sarrus mechanism (4 combined, one modeled here) that acheives translational motion in the x direction. The bottom link of the mechanism, depicted as pNA is what touches the ground and attaches to the newtonian frame. 
Below we describe the angles and their differentiables based off the system kinematics diagram. These are needed to solve the Jacobian and final system kinematics. 


```python
# Creating dynamic state variables. system argument denotes them as state variables for pynamics system from above
#Top 2 bar
qA,qA_d,qA_dd = Differentiable('qA',system) #Angle between N and A
qB,qB_d,qB_dd = Differentiable('qB',system) #from AB


#bottom 2bar
qC,qC_d,qC_dd = Differentiable('qC',system)
qD,qD_d,qD_dd = Differentiable('qD',system) #from DE
```

The following guess the initial values of the mechanism. Here we want to create it in a point of interesting motion, so our guesses are based on the mechanism in a halfway point. That being halfway between either extenstion or contraction. Essentially this means that the two flat plates that translate in the x direction are half way to one another giving us about a 90 degree angle between the top and bottom links. 


```python
#Create an initial guess for their starting positions; not necessarily accurate, given the constraint that they are
#supposed to be connected with given, constant length
#Guesses are close but not quite exact for the desired configuration to show that it solves correctly
#3bar
initialvalues = {}
initialvalues[qA]=60*pi/180 
# Setting initial angle values. See above for angle positions
initialvalues[qA_d]=0*pi/180
initialvalues[qB]=-90*pi/180  
initialvalues[qB_d]=0*pi/180

initialvalues[qC]=90*pi/180
initialvalues[qC_d]=0*pi/180
initialvalues[qD]=-45*pi/180   #90
initialvalues[qD_d]=0*pi/180
```


```python
statevariables = system.get_state_variables() #Retrieve state variables in the order they are stored in the system
```

All references frames needed to define the system. Set the N frame as the newtonian frame that is fixed. We then rotate each frame from the frame prior (seperatley for each side) about the z axis by an amount of q. q relates to the initial angles we guessed. 


```python
N = Frame('N') # Initializing frames
A = Frame('A')
B = Frame('B')
C = Frame('C')
D = Frame('D')
```


```python
system.set_newtonian(N) #Set N frame as the newtonian
```


```python
#3bars
A.rotate_fixed_axis_directed(N,[0,0,1],qA,system) # A reference frame rotates about N's ref frame in Z direction by qA amount
B.rotate_fixed_axis_directed(A,[0,0,1],qB,system)
D.rotate_fixed_axis_directed(N,[0,0,1],qD,system)
C.rotate_fixed_axis_directed(D,[0,0,1],qC,system)
```

Here we define the necessary points to create the mechanism. 

We need the bottom link to touch the ground, giving us the two flat plates parralell to the y axis (enforced with constraints). To do this we consider two three bar mechanisms that mirror one another. We had to use two 3 bar mechanisms as the top angles (that allow pC and pD to combine) have to be specified. 
Each 3 bar is based off of pNA, then using distances and the respective x axis of the specified frame, we build to PCtip and PDtip which are the end effectors of the 3 bar mechanisms we need to combine. 


```python

pA = 0*N.x + (l0 + h)*N.y + 0*N.z

pB = pA + la*A.x
pBtip = pB + lb*B.x

pD = 0*N.x + h*N.y + 0*N.z
pC = pD + ld*D.x
pCtip = pC + lc*C.x
```


```python
#Arbitrary pout that represents the end effector, here it is  pFE. The system moves in one direction
pout = pBtip
```


```python
points = [pA,pB,pBtip,pCtip,pC,pD]
```

Create list of initial values 


```python
statevariables = system.get_state_variables()
ini0 = [initialvalues[item] for item in statevariables]
```

Create constraints. 

Our system has 6 inputs, which we defined as the q values earlier. We know that our independant values depend on qA, leaving dependant values for qB,qE,qF,qC,qD. With that we have 5 unknowns and will need 5 constraint equations. 

The following vectors are the ones we need to use for constraints. eq_vector represents the distance between pD and pBC, eq_vector2 is used for pBC pAB, and eq_vector3 is used for pD, pFE. 



```python
#Define the closed loop kinematics of the four bar linkage.
eq_vector = pB - pC
eq_vector1 = pCtip - pBtip
```

The actual constraint equations (5 total for 5 unknowns) are below

The first constraint makes sure that the two flat plates are parallel to one another. They are defined by the vectors pAB to pBC and pFE to pDE respectively. To make them parallel the dot product must equal the multiplied value of the magnitude. 
The second constraint makes the pBC,pAB vector parallel with the newtonian y axis, constraining the pFE,pDE vector too. 

The third and forth constraints make sure that both end effectors of the 3bar mechanisms combine and stay at the same x value in the newtonian frame

The final constraint keeps pBC and pD on the same newtonian y value so that the mechanism cannot rotate.

These constraints together allow us to acheive translational motion


```python

eq = [] # eq -> equation

eq.append((eq_vector1).dot(N.x)) #same x value for B and C tip
eq.append((eq_vector1).length() - l0) #length constraint between B and C tip
eq.append((eq_vector).dot(N.x)) #B and C have same x 

eq_d=[(system.derivative(item)) for item in eq]
```

Here the independant and dependant values are officially defined. We now have the 5 constraint equations to satisfy the 5 unknowns. 


```python
qi = [qA]
qd = [qB,qC,qD] #the number of items in qd should = the number of constraints above
type(qd)
```


```python
constants = system.constant_values.copy() # Recalls link lengths declared near beginning
defined = dict([(item,initialvalues[item]) for item in qi])
constants.update(defined)
constants # Stores the names of constants/known values along with their values
```




    {h: 0.0381, l₀: 0.0508, la: 0.0254, lb: 0.0254, lc: 0.0254, ld: 0.0254, qA: 1.
    0471975511965976}




```python
#substitute constants in equation
eq = [item.subs(constants) for item in eq] # substitutes the values from constants into the equation eq
```


```python
#convert to numpy array and sum the error
error = (numpy.array(eq)**2).sum()
```


```python
#Convert to a function that scipy can use. Sympy has a “labmdify” function that
#evaluates an expression, but scipy needs a slightly different format.
f = sympy.lambdify(qd,error)

def function(args):
    return f(*args)
```


```python
guess = [initialvalues[item] for item in qd]
```


```python
result = scipy.optimize.minimize(function,guess)
#if result.fun>1e-3:
    #raise(Exception("out of tolerance"))

result
```




          fun: 4.467843031864958e-08
     hess_inv: array([[ 2307.32494004, -2260.85186437,  1221.86241826],
           [-2260.85186437,  2217.56032316, -1186.4196172 ],
           [ 1221.86241826, -1186.4196172 ,  1101.66515492]])
          jac: array([-9.18740800e-06, -9.38689357e-06,  1.90081940e-07])
      message: 'Optimization terminated successfully.'
         nfev: 130
          nit: 20
         njev: 26
       status: 0
      success: True
            x: array([-2.09860814,  2.08900911, -1.04676818])



Here the code solved for the desired mechanism (orange) given the initial guesses (blue). We purposely gave a poor initial guess for one angle so that it is easy to visualize that the system solved correctly


```python
ini = []
for item in system.get_state_variables():
    if item in qd:
        ini.append(result.x[qd.index(item)])
    else:
        ini.append(initialvalues[item])
```


```python
points = PointsOutput(points, constant_values=system.constant_values)
points.calc(numpy.array([ini0,ini]))
points.plot_time()
```

    2021-02-26 23:00:40,720 - pynamics.output - INFO - calculating outputs
    2021-02-26 23:00:40,722 - pynamics.output - INFO - done calculating outputs
    




    <matplotlib.axes._subplots.AxesSubplot at 0x7f40b77d7dd0>




    
![png](output_39_2.png)
    



```python
points.calc(numpy.array([ini0,ini]))
```

    2021-02-26 23:00:40,968 - pynamics.output - INFO - calculating outputs
    2021-02-26 23:00:40,970 - pynamics.output - INFO - done calculating outputs
    




    array([[[0.        , 0.0889    ],
            [0.0127    , 0.11089705],
            [0.03469705, 0.09819705],
            [0.03592102, 0.0381    ],
            [0.01796051, 0.02013949],
            [0.        , 0.0381    ]],
    
           [[0.        , 0.0889    ],
            [0.0127    , 0.11089705],
            [0.02530721, 0.08884669],
            [0.02551832, 0.03804224],
            [0.01270944, 0.01610841],
            [0.        , 0.0381    ]]])



 Find Internal Jacobian

 To do so we turn the constraint equations into vectors, giving us two equations for independant and dependant. We then take the derivative of those constraint equation vectors and solve for the internal input/output Jacobian


```python
eq_d = sympy.Matrix(eq_d)
```


```python
qi = sympy.Matrix([qA_d])
qd = sympy.Matrix([qB_d,qC_d,qD_d])
```


```python
AA = eq_d.jacobian(qi)
BB = eq_d.jacobian(qd)
```


```python
BB.simplify() #simplify expression so we can actually run jacobian smoothly
BB
```




    ⎡                                                                             
    ⎢                                                                             
    ⎢                                                                             
    ⎢                                                                             
    ⎢1.0⋅lb⋅(l₀⋅cos(qA + qB) - la⋅sin(qB) + lc⋅sin(qA + qB - qC - qD) + ld⋅sin(qA 
    ⎢                                                                             
    ⎣                                                                             
    
                                                                                  
                                                                                  
                                                                                  
                ⎛  2                                                              
    + qB - qD))⋅⎝l₀  + 2⋅l₀⋅la⋅sin(qA) + 2⋅l₀⋅lb⋅sin(qA + qB) - 2⋅l₀⋅lc⋅sin(qC + q
                                                                                  
                                                                                  
    
                                                                               lb⋅
                                                                                  
                                                                                  
                             2                                                    
    D) - 2⋅l₀⋅ld⋅sin(qD) + la  + 2⋅la⋅lb⋅cos(qB) - 2⋅la⋅lc⋅cos(-qA + qC + qD) - 2⋅
                                                                                  
                                                                                  
    
    sin(qA + qB)                                                                  
                                                                                  
                                                                                  
                           2                                                      
    la⋅ld⋅cos(qA - qD) + lb  + 2⋅lb⋅lc⋅sin(qA)⋅sin(qB)⋅cos(qC + qD) - 2⋅lb⋅lc⋅sin(
                                                                                  
        0                                                                         
    
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    qC)⋅sin(qA - qD)⋅cos(qB) - 2⋅lb⋅lc⋅sin(qD)⋅sin(qA + qB)⋅cos(qC) - 2⋅lb⋅lc⋅cos(
                                                                                  
                                                                                  
    
                                                                                  
                                                                                  
                                                                                  
                                                             2                    
    qA)⋅cos(qD)⋅cos(qB - qC) - 2⋅lb⋅ld⋅cos(qA + qB - qD) + lc  + 2⋅lc⋅ld⋅cos(qC) +
                                                                                  
                                                                                  
    
                                                                                  
                                                                                  
         -0.5                                                                     
       2⎞                                                                         
     ld ⎠      1.0⋅lc⋅(-l₀⋅cos(qC + qD) + la⋅sin(-qA + qC + qD) - lb⋅sin(qA + qB -
                                                                                  
                                                                                  
    
                                                                                  
                                                                                  
                                                                                  
                            ⎛  2                                                  
     qC - qD) - ld⋅sin(qC))⋅⎝l₀  + 2⋅l₀⋅la⋅sin(qA) + 2⋅l₀⋅lb⋅sin(qA + qB) - 2⋅l₀⋅l
                                                                                  
                                                                                  
    
                                                                                  
                                                                                  
                                                                                  
                                         2                                        
    c⋅sin(qC + qD) - 2⋅l₀⋅ld⋅sin(qD) + la  + 2⋅la⋅lb⋅cos(qB) - 2⋅la⋅lc⋅cos(-qA + q
                                                                                  
                                                                                  
    
            -lc⋅sin(qC + qD)                                                      
                                                                                  
                                                                                  
                                       2                                          
    C + qD) - 2⋅la⋅ld⋅cos(qA - qD) + lb  + 2⋅lb⋅lc⋅sin(qA)⋅sin(qB)⋅cos(qC + qD) - 
                                                                                  
                   0                                                              
    
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    2⋅lb⋅lc⋅sin(qC)⋅sin(qA - qD)⋅cos(qB) - 2⋅lb⋅lc⋅sin(qD)⋅sin(qA + qB)⋅cos(qC) - 
                                                                                  
                                                                                  
    
                                                                                  
                                                                                  
                                                                                  
                                                                         2        
    2⋅lb⋅lc⋅cos(qA)⋅cos(qD)⋅cos(qB - qC) - 2⋅lb⋅ld⋅cos(qA + qB - qD) + lc  + 2⋅lc⋅
                                                                                  
                                                                                  
    
                                                                                  
                                                                                  
                     -0.5                                                         
                   2⎞                                                             
    ld⋅cos(qC) + ld ⎠      1.0⋅(-l₀⋅lc⋅cos(qC + qD) - l₀⋅ld⋅cos(qD) + la⋅lc⋅sin(-q
                                                                                  
                                                                                  
    
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    A + qC + qD) - la⋅ld⋅sin(qA - qD) - lb⋅lc⋅sin(qA + qB - qC - qD) - lb⋅ld⋅sin(q
                                                                                  
                                                                                  
    
                                                                                  
                                                                                  
                                                                                  
                  ⎛  2                                                            
    A + qB - qD))⋅⎝l₀  + 2⋅l₀⋅la⋅sin(qA) + 2⋅l₀⋅lb⋅sin(qA + qB) - 2⋅l₀⋅lc⋅sin(qC +
                                                                                  
                                                                                  
    
                                             -lc⋅sin(qC + qD) - ld⋅sin(qD)        
                                                                                  
                                                                                  
                               2                                                  
     qD) - 2⋅l₀⋅ld⋅sin(qD) + la  + 2⋅la⋅lb⋅cos(qB) - 2⋅la⋅lc⋅cos(-qA + qC + qD) - 
                                                                                  
                                                       ld⋅sin(qD)                 
    
                                                                                  
                                                                                  
                                                                                  
                             2                                                    
    2⋅la⋅ld⋅cos(qA - qD) + lb  + 2⋅lb⋅lc⋅sin(qA)⋅sin(qB)⋅cos(qC + qD) - 2⋅lb⋅lc⋅si
                                                                                  
                                                                                  
    
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    n(qC)⋅sin(qA - qD)⋅cos(qB) - 2⋅lb⋅lc⋅sin(qD)⋅sin(qA + qB)⋅cos(qC) - 2⋅lb⋅lc⋅co
                                                                                  
                                                                                  
    
                                                                                  
                                                                                  
                                                                                  
                                                               2                  
    s(qA)⋅cos(qD)⋅cos(qB - qC) - 2⋅lb⋅ld⋅cos(qA + qB - qD) + lc  + 2⋅lc⋅ld⋅cos(qC)
                                                                                  
                                                                                  
    
               ⎤
               ⎥
           -0.5⎥
         2⎞    ⎥
     + ld ⎠    ⎥
               ⎥
               ⎦




```python
J = -BB.inv()*AA
J  #note, very long and very complex. simplify is necessary to comprehend
```




    ⎡                                                                             
    ⎢                                                                             
    ⎢                                                                             
    ⎢                                                                             
    ⎢                                                                             
    ⎢                                                                             
    ⎢                                                                             
    ⎢                                                                             
    ⎢   ⎛                                                                         
    ⎢la⋅⎝1.0⋅l₀⋅lc⋅sin(qA + qB)⋅cos(qC + qD) - l₀⋅lc⋅sin(qC + qD)⋅cos(qA + qB) - l
    ⎢─────────────────────────────────────────────────────────────────────────────
    ⎢                                                                             
    ⎢                                                                    -l₀⋅lc⋅ld
    ⎢                                                                             
    ⎢                                                                             
    ⎢                                                                             
    ⎣                                                                             
    
                                                                                  
                                                                                  
                                                          la⋅(1.0⋅l₀⋅sin(qD)⋅cos(q
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
    1.0⋅l₀⋅lb⋅sin(qD)⋅sin(qA + qB)⋅cos(qC + qD) - l₀⋅lb⋅sin(qD)⋅sin(qC + qD)⋅cos(q
                                                                                  
                                                                                  
                                                                                  
    ₀⋅ld⋅sin(qD)⋅cos(qA + qB) + 1.0⋅l₀⋅ld⋅sin(qA + qB)⋅cos(qD) + 1.0⋅la⋅lc⋅sin(qB)
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
    ⋅sin(qD)⋅sin(qA + qB)⋅cos(qC + qD) + 1.0⋅l₀⋅lc⋅ld⋅sin(qD)⋅sin(qC + qD)⋅cos(qA 
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                                                                                  
                                                                                  
    C + qD) - l₀⋅sin(qC + qD)⋅cos(qD) - la⋅sin(qD)⋅sin(-qA + qC + qD) - la⋅sin(qA 
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
    A + qB) + 1.0⋅la⋅lb⋅sin(qB)⋅sin(qD)⋅sin(qC + qD) - la⋅lb⋅sin(qD)⋅sin(qA + qB)⋅
                                                                                  
                                                                                  
                                                                                  
    ⋅sin(qC + qD) - la⋅lc⋅sin(qA + qB)⋅sin(-qA + qC + qD) + 1.0⋅la⋅ld⋅sin(qB)⋅sin(
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
    + qB) - la⋅lc⋅ld⋅sin(qB)⋅sin(qD)⋅sin(qC + qD) + 1.0⋅la⋅lc⋅ld⋅sin(qD)⋅sin(qA + 
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                                                                                  
                                                                                  
    - qD)⋅sin(qC + qD) + 1.0⋅lb⋅sin(qD)⋅sin(qA + qB - qC - qD) - lb⋅sin(qC + qD)⋅s
    ──────────────────────────────────────────────────────────────────────────────
                               2                                                  
    sin(-qA + qC + qD) + 1.0⋅lb ⋅sin(qD)⋅sin(qA + qB)⋅sin(qA + qB - qC - qD) - lb⋅
                                                                                  
                                                                                  
                                                                                  
    qD) + 1.0⋅la⋅ld⋅sin(qA + qB)⋅sin(qA - qD) + 1.0⋅lb⋅lc⋅sin(qA + qB)⋅sin(qA + qB
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
    qB)⋅sin(-qA + qC + qD) - lb⋅lc⋅ld⋅sin(qD)⋅sin(qA + qB)⋅sin(qA + qB - qC - qD) 
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                                                                                  
                                                                                  
    in(qA + qB - qD) + 1.0⋅lc⋅sin(qC)⋅sin(qC + qD) + 1.0⋅ld⋅sin(qC)⋅sin(qD))⋅sin(q
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
    lc⋅sin(qD)⋅sin(qC + qD)⋅sin(qA + qB - qC - qD) + 1.0⋅lb⋅ld⋅sin(qC)⋅sin(qD)⋅sin
                                                                                  
                                                                                  
                                                               2                  
     - qC - qD) + 1.0⋅lb⋅ld⋅sin(qA + qB)⋅sin(qA + qB - qD) - lc ⋅sin(qC + qD)⋅sin(
    ──────────────────────────────────────────────────────────────────────────────
            2                                                       2             
    + 1.0⋅lc ⋅ld⋅sin(qD)⋅sin(qC + qD)⋅sin(qA + qB - qC - qD) - lc⋅ld ⋅sin(qC)⋅sin(
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                                                                                  
                                                                                  
    A)                                                                            
    ──────────────────────────────────────────────────────── - ───────────────────
                                                                                  
    (qA + qB) - lb⋅ld⋅sin(qD)⋅sin(qC + qD)⋅sin(qA + qB - qD)   1.0⋅l₀⋅lb⋅sin(qA + 
                                                                                  
                                                                                  
                                                                                  
    qA + qB - qC - qD) - lc⋅ld⋅sin(qD)⋅sin(qA + qB - qC - qD) - lc⋅ld⋅sin(qC + qD)
    ──────────────────────────────────────────────────────────────────────────────
                                2                                                 
    qD)⋅sin(qA + qB) + 1.0⋅lc⋅ld ⋅sin(qD)⋅sin(qC + qD)⋅sin(qA + qB - qD)          
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                                                                                  
                                                                                  
                                                              (la⋅sin(qA) + lb⋅sin
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
    qB)⋅cos(qC + qD) - l₀⋅lb⋅sin(qC + qD)⋅cos(qA + qB) + 1.0⋅la⋅lb⋅sin(qB)⋅sin(qC 
                                                                                  
                                                                                  
                           2                          ⎞                           
    ⋅sin(qA + qB - qD) - ld ⋅sin(qD)⋅sin(qA + qB - qD)⎠⋅sin(qA)                   
    ─────────────────────────────────────────────────────────── - ────────────────
                                                                                  
                                                                  -l₀⋅lc⋅sin(qA + 
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                                                                                  
                                                                                  
    (qA)⋅cos(qB) + lb⋅sin(qB)⋅cos(qA))⋅(1.0⋅l₀⋅cos(qC + qD) - la⋅sin(-qA + qC + qD
    ──────────────────────────────────────────────────────────────────────────────
                                                          2                       
    + qD) - la⋅lb⋅sin(qA + qB)⋅sin(-qA + qC + qD) + 1.0⋅lb ⋅sin(qA + qB)⋅sin(qA + 
                                                                                  
                                                                                  
                                                                                  
                                                                      (la⋅sin(qA) 
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
    qB)⋅cos(qC + qD) + 1.0⋅l₀⋅lc⋅sin(qC + qD)⋅cos(qA + qB) - la⋅lc⋅sin(qB)⋅sin(qC 
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                                                                                  
                                                                                  
    ) + 1.0⋅lb⋅sin(qA + qB - qC - qD) + 1.0⋅ld⋅sin(qC))                           
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
    qB - qC - qD) - lb⋅lc⋅sin(qC + qD)⋅sin(qA + qB - qC - qD) + 1.0⋅lb⋅ld⋅sin(qC)⋅
                                                                                  
                                                                                  
                                                                                  
    + lb⋅sin(qA)⋅cos(qB) + lb⋅sin(qB)⋅cos(qA))⋅(-l₀⋅cos(qA + qB) + 1.0⋅la⋅sin(qB) 
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
    + qD) + 1.0⋅la⋅lc⋅sin(qA + qB)⋅sin(-qA + qC + qD) - lb⋅lc⋅sin(qA + qB)⋅sin(qA 
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                                                                                  
                                                                                  
                                                          1.0⋅(1.0⋅l₀⋅la⋅cos(qA) -
    ─────────────────────────────────────────────────── + ────────────────────────
                                                                                  
    sin(qA + qB) - lb⋅ld⋅sin(qC + qD)⋅sin(qA + qB - qD)                           
                                                                                  
                                                                                  
                                                                                  
    - lc⋅sin(qA + qB - qC - qD) - ld⋅sin(qA + qB - qD))                           
    ──────────────────────────────────────────────────────────────────────────────
                            2                                                     
    + qB - qC - qD) + 1.0⋅lc ⋅sin(qC + qD)⋅sin(qA + qB - qC - qD) - lc⋅ld⋅sin(qC)⋅
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                                                                                  
                                                                                  
     l₀⋅lb⋅sin(qA)⋅sin(qB) + 1.0⋅l₀⋅lb⋅cos(qA)⋅cos(qB) + 0.5⋅la⋅lc⋅(-sin(qA)⋅sin(q
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                              (1.0⋅l₀⋅la⋅cos(qA) -
    ─────────────────────────────────────────────────────── - ────────────────────
                                                                                  
    sin(qA + qB) + 1.0⋅lc⋅ld⋅sin(qC + qD)⋅sin(qA + qB - qD)                       
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                                                                                  
                                                                                  
    D) - cos(qA)⋅cos(qD))⋅sin(qC) + 0.5⋅la⋅lc⋅(sin(qA)⋅cos(qD) - sin(qD)⋅cos(qA))⋅
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
     l₀⋅lb⋅sin(qA)⋅sin(qB) + 1.0⋅l₀⋅lb⋅cos(qA)⋅cos(qB) + 0.5⋅la⋅lc⋅(-sin(qA)⋅sin(q
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                                                                                  
                                                                                  
    cos(qC) - 0.5⋅la⋅lc⋅(sin(qC)⋅sin(qD) - cos(qC)⋅cos(qD))⋅sin(qA) + 0.5⋅la⋅lc⋅(-
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    D) - cos(qA)⋅cos(qD))⋅sin(qC) + 0.5⋅la⋅lc⋅(sin(qA)⋅cos(qD) - sin(qD)⋅cos(qA))⋅
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                                                                                  
                                                                                  
    sin(qC)⋅cos(qD) - sin(qD)⋅cos(qC))⋅cos(qA) + 1.0⋅la⋅ld⋅sin(qA)⋅cos(qD) - la⋅ld
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    cos(qC) - 0.5⋅la⋅lc⋅(sin(qC)⋅sin(qD) - cos(qC)⋅cos(qD))⋅sin(qA) + 0.5⋅la⋅lc⋅(-
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                                                                                  
                                                                                  
    ⋅sin(qD)⋅cos(qA) + 0.5⋅lb⋅lc⋅(2⋅sin(qA)⋅sin(qB) - 2⋅cos(qA)⋅cos(qB))⋅(sin(qC)⋅
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    sin(qC)⋅cos(qD) - sin(qD)⋅cos(qC))⋅cos(qA) + 1.0⋅la⋅ld⋅sin(qA)⋅cos(qD) - la⋅ld
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                                                                                  
                                                                                  
    cos(qD) + sin(qD)⋅cos(qC)) + 0.5⋅lb⋅lc⋅(2⋅sin(qA)⋅cos(qB) + 2⋅sin(qB)⋅cos(qA))
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    ⋅sin(qD)⋅cos(qA) + 0.5⋅lb⋅lc⋅(2⋅sin(qA)⋅sin(qB) - 2⋅cos(qA)⋅cos(qB))⋅(sin(qC)⋅
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                                                                                  
                                                                                  
    ⋅(-sin(qC)⋅sin(qD) + cos(qC)⋅cos(qD)) + 0.5⋅lb⋅ld⋅(sin(qA)⋅sin(qB) - cos(qA)⋅c
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    cos(qD) + sin(qD)⋅cos(qC)) + 0.5⋅lb⋅lc⋅(2⋅sin(qA)⋅cos(qB) + 2⋅sin(qB)⋅cos(qA))
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                                                                                  
                                                                                  
    os(qB))⋅sin(qD) + 0.5⋅lb⋅ld⋅(sin(qA)⋅sin(qD) + cos(qA)⋅cos(qD))⋅sin(qB) + 0.5⋅
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    ⋅(-sin(qC)⋅sin(qD) + cos(qC)⋅cos(qD)) + 0.5⋅lb⋅ld⋅(sin(qA)⋅sin(qB) - cos(qA)⋅c
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                                                                                  
                                                                                  
    lb⋅ld⋅(sin(qA)⋅cos(qB) + sin(qB)⋅cos(qA))⋅cos(qD) + 0.5⋅lb⋅ld⋅(sin(qA)⋅cos(qD)
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    os(qB))⋅sin(qD) + 0.5⋅lb⋅ld⋅(sin(qA)⋅sin(qD) + cos(qA)⋅cos(qD))⋅sin(qB) + 0.5⋅
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
             la⋅sin(qA)                                                           
             ──────────                                                           
             ld⋅sin(qD)                                                           
    
                                                                                  
                                 ⎛  2                                             
     - sin(qD)⋅cos(qA))⋅cos(qB))⋅⎝l₀  + 2⋅l₀⋅la⋅sin(qA) + 2⋅l₀⋅lb⋅sin(qA + qB) - 2
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    lb⋅ld⋅(sin(qA)⋅cos(qB) + sin(qB)⋅cos(qA))⋅cos(qD) + 0.5⋅lb⋅ld⋅(sin(qA)⋅cos(qD)
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                                                                                  
                                              2                                   
    ⋅l₀⋅lc⋅sin(qC + qD) - 2⋅l₀⋅ld⋅sin(qD) + la  + 2⋅la⋅lb⋅cos(qB) - 2⋅la⋅lc⋅cos(-q
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                 1.0⋅l₀⋅lb⋅sin(qA + qB)⋅cos(qC + q
                                                                                  
                                                                                  
                                 ⎛  2                                             
     - sin(qD)⋅cos(qA))⋅cos(qB))⋅⎝l₀  + 2⋅l₀⋅la⋅sin(qA) + 2⋅l₀⋅lb⋅sin(qA + qB) - 2
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                                                                                  
                                            2                                     
    A + qC + qD) - 2⋅la⋅ld⋅cos(qA - qD) + lb  + 2⋅lb⋅lc⋅sin(qA)⋅sin(qB)⋅cos(qC + q
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
    D) - l₀⋅lb⋅sin(qC + qD)⋅cos(qA + qB) + 1.0⋅la⋅lb⋅sin(qB)⋅sin(qC + qD) - la⋅lb⋅
                                                                                  
                                                                                  
                                              2                                   
    ⋅l₀⋅lc⋅sin(qC + qD) - 2⋅l₀⋅ld⋅sin(qD) + la  + 2⋅la⋅lb⋅cos(qB) - 2⋅la⋅lc⋅cos(-q
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                  -l₀⋅lc⋅sin(qA + qB)⋅cos(qC + qD)
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                                                                                  
                                                                                  
    D) - 2⋅lb⋅lc⋅sin(qC)⋅sin(qA - qD)⋅cos(qB) - 2⋅lb⋅lc⋅sin(qD)⋅sin(qA + qB)⋅cos(q
    ──────────────────────────────────────────────────────────────────────────────
                                            2                                     
    sin(qA + qB)⋅sin(-qA + qC + qD) + 1.0⋅lb ⋅sin(qA + qB)⋅sin(qA + qB - qC - qD) 
                                                                                  
                                                                                  
                                            2                                     
    A + qC + qD) - 2⋅la⋅ld⋅cos(qA - qD) + lb  + 2⋅lb⋅lc⋅sin(qA)⋅sin(qB)⋅cos(qC + q
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
     + 1.0⋅l₀⋅lc⋅sin(qC + qD)⋅cos(qA + qB) - la⋅lc⋅sin(qB)⋅sin(qC + qD) + 1.0⋅la⋅l
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                                                                                  
                                                                              2   
    C) - 2⋅lb⋅lc⋅cos(qA)⋅cos(qD)⋅cos(qB - qC) - 2⋅lb⋅ld⋅cos(qA + qB - qD) + lc  + 
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
    - lb⋅lc⋅sin(qC + qD)⋅sin(qA + qB - qC - qD) + 1.0⋅lb⋅ld⋅sin(qC)⋅sin(qA + qB) -
                                                                                  
                                                                                  
                                                                                  
    D) - 2⋅lb⋅lc⋅sin(qC)⋅sin(qA - qD)⋅cos(qB) - 2⋅lb⋅lc⋅sin(qD)⋅sin(qA + qB)⋅cos(q
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
    c⋅sin(qA + qB)⋅sin(-qA + qC + qD) - lb⋅lc⋅sin(qA + qB)⋅sin(qA + qB - qC - qD) 
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                          0.5                                                     
                        2⎞    ⎛  2                                                
    2⋅lc⋅ld⋅cos(qC) + ld ⎠   ⋅⎝l₀  + 2⋅l₀⋅la⋅sin(qA) + 2⋅l₀⋅lb⋅sin(qA)⋅cos(qB) + 2
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
     lb⋅ld⋅sin(qC + qD)⋅sin(qA + qB - qD)                                         
                                                                                  
                                                                                  
                                                                              2   
    C) - 2⋅lb⋅lc⋅cos(qA)⋅cos(qD)⋅cos(qB - qC) - 2⋅lb⋅ld⋅cos(qA + qB - qD) + lc  + 
    ──────────────────────────────────────────────────────────────────────────────
            2                                                                     
    + 1.0⋅lc ⋅sin(qC + qD)⋅sin(qA + qB - qC - qD) - lc⋅ld⋅sin(qC)⋅sin(qA + qB) + 1
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                                                                                  
                                                                                  
    ⋅l₀⋅lb⋅sin(qB)⋅cos(qA) - 2⋅l₀⋅lc⋅sin(qC)⋅cos(qD) - 2⋅l₀⋅lc⋅sin(qD)⋅cos(qC) - 2
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                          0.5                                                     
                        2⎞    ⎛  2                                                
    2⋅lc⋅ld⋅cos(qC) + ld ⎠   ⋅⎝l₀  + 2⋅l₀⋅la⋅sin(qA) + 2⋅l₀⋅lb⋅sin(qA)⋅cos(qB) + 2
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
    .0⋅lc⋅ld⋅sin(qC + qD)⋅sin(qA + qB - qD)                                       
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                                                                                  
                       2                                                          
    ⋅l₀⋅ld⋅sin(qD) + la  + 2⋅la⋅lb⋅cos(qB) - la⋅lc⋅(sin(qA)⋅sin(qD) + cos(qA)⋅cos(
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    ⋅l₀⋅lb⋅sin(qB)⋅cos(qA) - 2⋅l₀⋅lc⋅sin(qC)⋅cos(qD) - 2⋅l₀⋅lc⋅sin(qD)⋅cos(qC) - 2
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                                                                                  
                                                                                  
    qD))⋅cos(qC) - la⋅lc⋅(sin(qA)⋅cos(qD) - sin(qD)⋅cos(qA))⋅sin(qC) - la⋅lc⋅(-sin
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                       2                                                          
    ⋅l₀⋅ld⋅sin(qD) + la  + 2⋅la⋅lb⋅cos(qB) - la⋅lc⋅(sin(qA)⋅sin(qD) + cos(qA)⋅cos(
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                                                                                  
                                                                                  
    (qC)⋅sin(qD) + cos(qC)⋅cos(qD))⋅cos(qA) - la⋅lc⋅(sin(qC)⋅cos(qD) + sin(qD)⋅cos
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    qD))⋅cos(qC) - la⋅lc⋅(sin(qA)⋅cos(qD) - sin(qD)⋅cos(qA))⋅sin(qC) - la⋅lc⋅(-sin
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                                                                                  
                                                                          2       
    (qC))⋅sin(qA) - 2⋅la⋅ld⋅sin(qA)⋅sin(qD) - 2⋅la⋅ld⋅cos(qA)⋅cos(qD) + lb  - 2⋅lb
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    (qC)⋅sin(qD) + cos(qC)⋅cos(qD))⋅cos(qA) - la⋅lc⋅(sin(qC)⋅cos(qD) + sin(qD)⋅cos
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                                                                                  
                                                                                  
    ⋅lc⋅(-sin(qA)⋅sin(qB) + cos(qA)⋅cos(qB))⋅(-sin(qC)⋅sin(qD) + cos(qC)⋅cos(qD)) 
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                          2       
    (qC))⋅sin(qA) - 2⋅la⋅ld⋅sin(qA)⋅sin(qD) - 2⋅la⋅ld⋅cos(qA)⋅cos(qD) + lb  - 2⋅lb
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                                                                                  
                                                                                  
    - 2⋅lb⋅lc⋅(sin(qA)⋅cos(qB) + sin(qB)⋅cos(qA))⋅(sin(qC)⋅cos(qD) + sin(qD)⋅cos(q
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    ⋅lc⋅(-sin(qA)⋅sin(qB) + cos(qA)⋅cos(qB))⋅(-sin(qC)⋅sin(qD) + cos(qC)⋅cos(qD)) 
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                                                                                  
                                                                                  
    C)) - lb⋅ld⋅(-sin(qA)⋅sin(qB) + cos(qA)⋅cos(qB))⋅cos(qD) - lb⋅ld⋅(sin(qA)⋅sin(
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    - 2⋅lb⋅lc⋅(sin(qA)⋅cos(qB) + sin(qB)⋅cos(qA))⋅(sin(qC)⋅cos(qD) + sin(qD)⋅cos(q
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                                                                                  
                                                                                  
    qD) + cos(qA)⋅cos(qD))⋅cos(qB) - lb⋅ld⋅(sin(qA)⋅cos(qB) + sin(qB)⋅cos(qA))⋅sin
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    C)) - lb⋅ld⋅(-sin(qA)⋅sin(qB) + cos(qA)⋅cos(qB))⋅cos(qD) - lb⋅ld⋅(sin(qA)⋅sin(
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                                                                                  
                                                                  2               
    (qD) - lb⋅ld⋅(-sin(qA)⋅cos(qD) + sin(qD)⋅cos(qA))⋅sin(qB) + lc  + 2⋅lc⋅ld⋅cos(
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    qD) + cos(qA)⋅cos(qD))⋅cos(qB) - lb⋅ld⋅(sin(qA)⋅cos(qB) + sin(qB)⋅cos(qA))⋅sin
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
              -0.5                                                                
            2⎞                                                                    
    qC) + ld ⎠    ⋅sin(qC + qD)                                                   
    ───────────────────────────                                                   
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                  2               
    (qD) - lb⋅ld⋅(-sin(qA)⋅cos(qD) + sin(qD)⋅cos(qA))⋅sin(qB) + lc  + 2⋅lc⋅ld⋅cos(
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                               ⎤
                               ⎥
                               ⎥
                               ⎥
                               ⎥
                               ⎥
                               ⎥
              -0.5             ⎥
            2⎞                 ⎥
    qC) + ld ⎠    ⋅sin(qA + qB)⎥
    ───────────────────────────⎥
                               ⎥
                               ⎥
                               ⎥
                               ⎥
                               ⎥
                               ⎦



We can then simplify the jacobian into a clean looking form that we can utulize for calculations. qd2 is the dependant variables we solve for and subs substitutes the values for us. An example finding vout is shown below that is reused in part 8 of this assignment


```python
J.simplify()
J
```




    ⎡    ⎛                                                                        
    ⎢1.0⋅⎝-2.0⋅l₀⋅la⋅cos(qA + qC) + 2.0⋅l₀⋅la⋅cos(-qA + qC + 2⋅qD) - 2.0⋅l₀⋅lb⋅cos
    ⎢─────────────────────────────────────────────────────────────────────────────
    ⎢                                                                             
    ⎢                                                                             
    ⎢          ⎛                                                                  
    ⎢   1.0⋅la⋅⎝2.0⋅l₀⋅lc⋅cos(-qB + qC + qD) - 2.0⋅l₀⋅lc⋅cos(2⋅qA + qB - qC - qD) 
    ⎢   ──────────────────────────────────────────────────────────────────────────
    ⎢                                                                             
    ⎢                                                                             
    ⎢                                                                             
    ⎢                                                                             
    ⎣                                                                             
    
                                                                2             2   
    (qA + qB - qC) + 2.0⋅l₀⋅lb⋅cos(qA + qB - qC - 2⋅qD) + 1.0⋅la ⋅sin(qC) - la ⋅si
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
    + 2.0⋅l₀⋅ld⋅cos(qB + qD) - 2.0⋅l₀⋅ld⋅cos(2⋅qA + qB - qD) + 1.0⋅la⋅lc⋅sin(qA - 
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                         2                    2                                   
    n(2⋅qA + qC) + 1.0⋅la ⋅sin(qC + 2⋅qD) - la ⋅sin(-2⋅qA + qC + 2⋅qD) - la⋅lb⋅sin
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
    qB + qC + qD) + 2.0⋅la⋅lc⋅sin(qA + qB - qC - qD) - la⋅lc⋅sin(3⋅qA + qB - qC - 
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                                                                                  
    (2⋅qA + qB - qC) - la⋅lb⋅sin(2⋅qA + qB + qC) + 1.0⋅la⋅lb⋅sin(-qB + qC + 2⋅qD) 
    ──────────────────────────────────────────────────────────────────────────────
          lb⋅(2⋅l₀⋅cos(qA + qB - qC) - 2⋅l₀⋅cos(qA + qB - qC - 2⋅qD) - la⋅sin(qB -
                                                                                  
                                                                                  
    qD) - la⋅ld⋅sin(-qA + qB + qD) + 1.0⋅la⋅ld⋅sin(qA + qB - qD) + 1.0⋅la⋅ld⋅sin(q
    ──────────────────────────────────────────────────────────────────────────────
       lc⋅ld⋅(2⋅l₀⋅cos(qA + qB - qC) - 2⋅l₀⋅cos(qA + qB - qC - 2⋅qD) - la⋅sin(qB -
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                                                                                  
    + 1.0⋅la⋅lb⋅sin(qB + qC + 2⋅qD) + 2.0⋅la⋅lb⋅sin(2⋅qA + qB - qC - 2⋅qD) - 2.0⋅l
    ──────────────────────────────────────────────────────────────────────────────
     qC) + la⋅sin(2⋅qA + qB - qC) - la⋅sin(-qB + qC + 2⋅qD) - la⋅sin(2⋅qA + qB - q
                                                                                  
                                                                                  
    A + qB + qD) - la⋅ld⋅sin(3⋅qA + qB - qD) - lb⋅lc⋅sin(-qA + qC + qD) + 1.0⋅lb⋅l
    ──────────────────────────────────────────────────────────────────────────────
     qC) + la⋅sin(2⋅qA + qB - qC) - la⋅sin(-qB + qC + 2⋅qD) - la⋅sin(2⋅qA + qB - q
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                                                                                  
    a⋅lc⋅sin(qA - qD) - la⋅lc⋅sin(-qA + 2⋅qC + 3⋅qD) + 1.0⋅la⋅lc⋅sin(qA + 2⋅qC + q
    ──────────────────────────────────────────────────────────────────────────────
    C - 2⋅qD) + lb⋅sin(qC) - lb⋅sin(qC + 2⋅qD) + lb⋅sin(2⋅qA + 2⋅qB - qC) - lb⋅sin
                                                                                  
                                                                                  
    c⋅sin(qA + qC + qD) + 1.0⋅lb⋅lc⋅sin(qA + 2⋅qB - qC - qD) - lb⋅lc⋅sin(3⋅qA + 2⋅
    ──────────────────────────────────────────────────────────────────────────────
    C - 2⋅qD) + lb⋅sin(qC) - lb⋅sin(qC + 2⋅qD) + lb⋅sin(2⋅qA + 2⋅qB - qC) - lb⋅sin
                                                                                  
                                                                         la⋅sin(qA
                                                                         ─────────
                                                                         ld⋅sin(qD
    
                                                                                  
    D) + 1.0⋅la⋅ld⋅sin(-qA + qC + qD) - la⋅ld⋅sin(-qA + qC + 3⋅qD) - la⋅ld⋅sin(qA 
    ──────────────────────────────────────────────────────────────────────────────
    (2⋅qA + 2⋅qB - qC - 2⋅qD) + lc⋅sin(qA + qB - qD) - lc⋅sin(qA + qB + qD) - lc⋅s
                                                                                  
                                                                                  
    qB - qC - qD) + 2.0⋅lb⋅ld⋅sin(qA - qD) + 1.0⋅lb⋅ld⋅sin(qA + 2⋅qB + qD) - lb⋅ld
    ──────────────────────────────────────────────────────────────────────────────
    (2⋅qA + 2⋅qB - qC - 2⋅qD) + lc⋅sin(qA + qB - qD) - lc⋅sin(qA + qB + qD) - lc⋅s
                                                                                  
    )                                                                             
    ─                                                                             
    )                                                                             
    
                                                 2                 2              
    + qC - qD) + 1.0⋅la⋅ld⋅sin(qA + qC + qD) - lb ⋅sin(qC) + 1.0⋅lb ⋅sin(qC + 2⋅qD
    ──────────────────────────────────────────────────────────────────────────────
    in(qA + qB - 2⋅qC - 3⋅qD) + lc⋅sin(qA + qB - 2⋅qC - qD) - ld⋅sin(qA + qB - qC 
                                                                                  
                               2                 2                    2           
    ⋅sin(3⋅qA + 2⋅qB - qD) - lc ⋅sin(qB) + 1.0⋅lc ⋅sin(2⋅qA + qB) - lc ⋅sin(-qB + 
    ──────────────────────────────────────────────────────────────────────────────
    in(qA + qB - 2⋅qC - 3⋅qD) + lc⋅sin(qA + qB - 2⋅qC - qD) - ld⋅sin(qA + qB - qC 
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
          2                               2                                       
    ) - lb ⋅sin(2⋅qA + 2⋅qB - qC) + 1.0⋅lb ⋅sin(2⋅qA + 2⋅qB - qC - 2⋅qD) - lb⋅lc⋅s
    ──────────────────────────────────────────────────────────────────────────────
    - 3⋅qD) + 2⋅ld⋅sin(qA + qB - qC - qD) - ld⋅sin(qA + qB - qC + qD))            
                                                                                  
                     2                                                            
    2⋅qC + 2⋅qD) - lc ⋅sin(2⋅qA + qB - 2⋅qC - 2⋅qD) + 1.0⋅lc⋅ld⋅sin(2⋅qA + qB - qC
    ──────────────────────────────────────────────────────────────────────────────
    - 3⋅qD) + 2⋅ld⋅sin(qA + qB - qC - qD) - ld⋅sin(qA + qB - qC + qD))            
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                                                                                  
    in(qA + qB - qD) + 1.0⋅lb⋅lc⋅sin(qA + qB + qD) + 1.0⋅lb⋅lc⋅sin(qA + qB - 2⋅qC 
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
    ) + 1.0⋅lc⋅ld⋅sin(2⋅qA + qB + qC) - lc⋅ld⋅sin(-qB + qC + 2⋅qD) - lc⋅ld⋅sin(qB 
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                                                                                  
    - 3⋅qD) - lb⋅lc⋅sin(qA + qB - 2⋅qC - qD) + 1.0⋅lb⋅ld⋅sin(qA + qB - qC - 3⋅qD) 
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                2                 
    + qC + 2⋅qD) - 2.0⋅lc⋅ld⋅sin(2⋅qA + qB - qC - 2⋅qD) + 1.0⋅ld ⋅sin(qB) + 1.0⋅ld
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                                                                         ⎞⎤
    - 2.0⋅lb⋅ld⋅sin(qA + qB - qC - qD) + 1.0⋅lb⋅ld⋅sin(qA + qB - qC + qD)⎠⎥
    ──────────────────────────────────────────────────────────────────────⎥
                                                                          ⎥
                                                                          ⎥
    2                    2                    2                      ⎞    ⎥
     ⋅sin(2⋅qA + qB) - ld ⋅sin(qB + 2⋅qD) - ld ⋅sin(2⋅qA + qB - 2⋅qD)⎠    ⎥
    ──────────────────────────────────────────────────────────────────    ⎥
                                                                          ⎥
                                                                          ⎥
                                                                          ⎥
                                                                          ⎥
                                                                          ⎦




```python
qd2 = J*qi
qd2
```




    ⎡         ⎛                                                                   
    ⎢1.0⋅qA_d⋅⎝-2.0⋅l₀⋅la⋅cos(qA + qC) + 2.0⋅l₀⋅la⋅cos(-qA + qC + 2⋅qD) - 2.0⋅l₀⋅l
    ⎢─────────────────────────────────────────────────────────────────────────────
    ⎢                                                                             
    ⎢                                                                             
    ⎢               ⎛                                                             
    ⎢   1.0⋅la⋅qA_d⋅⎝2.0⋅l₀⋅lc⋅cos(-qB + qC + qD) - 2.0⋅l₀⋅lc⋅cos(2⋅qA + qB - qC -
    ⎢   ──────────────────────────────────────────────────────────────────────────
    ⎢                                                                             
    ⎢                                                                             
    ⎢                                                                             
    ⎢                                                                             
    ⎣                                                                             
    
                                                                     2            
    b⋅cos(qA + qB - qC) + 2.0⋅l₀⋅lb⋅cos(qA + qB - qC - 2⋅qD) + 1.0⋅la ⋅sin(qC) - l
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
     qD) + 2.0⋅l₀⋅ld⋅cos(qB + qD) - 2.0⋅l₀⋅ld⋅cos(2⋅qA + qB - qD) + 1.0⋅la⋅lc⋅sin(
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
     2                        2                    2                              
    a ⋅sin(2⋅qA + qC) + 1.0⋅la ⋅sin(qC + 2⋅qD) - la ⋅sin(-2⋅qA + qC + 2⋅qD) - la⋅l
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
    qA - qB + qC + qD) + 2.0⋅la⋅lc⋅sin(qA + qB - qC - qD) - la⋅lc⋅sin(3⋅qA + qB - 
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                                                                                  
    b⋅sin(2⋅qA + qB - qC) - la⋅lb⋅sin(2⋅qA + qB + qC) + 1.0⋅la⋅lb⋅sin(-qB + qC + 2
    ──────────────────────────────────────────────────────────────────────────────
            lb⋅(2⋅l₀⋅cos(qA + qB - qC) - 2⋅l₀⋅cos(qA + qB - qC - 2⋅qD) - la⋅sin(qB
                                                                                  
                                                                                  
    qC - qD) - la⋅ld⋅sin(-qA + qB + qD) + 1.0⋅la⋅ld⋅sin(qA + qB - qD) + 1.0⋅la⋅ld⋅
    ──────────────────────────────────────────────────────────────────────────────
          lc⋅ld⋅(2⋅l₀⋅cos(qA + qB - qC) - 2⋅l₀⋅cos(qA + qB - qC - 2⋅qD) - la⋅sin(q
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                                                                                  
    ⋅qD) + 1.0⋅la⋅lb⋅sin(qB + qC + 2⋅qD) + 2.0⋅la⋅lb⋅sin(2⋅qA + qB - qC - 2⋅qD) - 
    ──────────────────────────────────────────────────────────────────────────────
     - qC) + la⋅sin(2⋅qA + qB - qC) - la⋅sin(-qB + qC + 2⋅qD) - la⋅sin(2⋅qA + qB -
                                                                                  
                                                                                  
    sin(qA + qB + qD) - la⋅ld⋅sin(3⋅qA + qB - qD) - lb⋅lc⋅sin(-qA + qC + qD) + 1.0
    ──────────────────────────────────────────────────────────────────────────────
    B - qC) + la⋅sin(2⋅qA + qB - qC) - la⋅sin(-qB + qC + 2⋅qD) - la⋅sin(2⋅qA + qB 
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                                                                                  
    2.0⋅la⋅lc⋅sin(qA - qD) - la⋅lc⋅sin(-qA + 2⋅qC + 3⋅qD) + 1.0⋅la⋅lc⋅sin(qA + 2⋅q
    ──────────────────────────────────────────────────────────────────────────────
     qC - 2⋅qD) + lb⋅sin(qC) - lb⋅sin(qC + 2⋅qD) + lb⋅sin(2⋅qA + 2⋅qB - qC) - lb⋅s
                                                                                  
                                                                                  
    ⋅lb⋅lc⋅sin(qA + qC + qD) + 1.0⋅lb⋅lc⋅sin(qA + 2⋅qB - qC - qD) - lb⋅lc⋅sin(3⋅qA
    ──────────────────────────────────────────────────────────────────────────────
    - qC - 2⋅qD) + lb⋅sin(qC) - lb⋅sin(qC + 2⋅qD) + lb⋅sin(2⋅qA + 2⋅qB - qC) - lb⋅
                                                                                  
                                                                         la⋅qA_d⋅s
                                                                         ─────────
                                                                            ld⋅sin
    
                                                                                  
    C + qD) + 1.0⋅la⋅ld⋅sin(-qA + qC + qD) - la⋅ld⋅sin(-qA + qC + 3⋅qD) - la⋅ld⋅si
    ──────────────────────────────────────────────────────────────────────────────
    in(2⋅qA + 2⋅qB - qC - 2⋅qD) + lc⋅sin(qA + qB - qD) - lc⋅sin(qA + qB + qD) - lc
                                                                                  
                                                                                  
     + 2⋅qB - qC - qD) + 2.0⋅lb⋅ld⋅sin(qA - qD) + 1.0⋅lb⋅ld⋅sin(qA + 2⋅qB + qD) - 
    ──────────────────────────────────────────────────────────────────────────────
    sin(2⋅qA + 2⋅qB - qC - 2⋅qD) + lc⋅sin(qA + qB - qD) - lc⋅sin(qA + qB + qD) - l
                                                                                  
    in(qA)                                                                        
    ──────                                                                        
    (qD)                                                                          
    
                                                      2                 2         
    n(qA + qC - qD) + 1.0⋅la⋅ld⋅sin(qA + qC + qD) - lb ⋅sin(qC) + 1.0⋅lb ⋅sin(qC +
    ──────────────────────────────────────────────────────────────────────────────
    ⋅sin(qA + qB - 2⋅qC - 3⋅qD) + lc⋅sin(qA + qB - 2⋅qC - qD) - ld⋅sin(qA + qB - q
                                                                                  
                                    2                 2                    2      
    lb⋅ld⋅sin(3⋅qA + 2⋅qB - qD) - lc ⋅sin(qB) + 1.0⋅lc ⋅sin(2⋅qA + qB) - lc ⋅sin(-
    ──────────────────────────────────────────────────────────────────────────────
    c⋅sin(qA + qB - 2⋅qC - 3⋅qD) + lc⋅sin(qA + qB - 2⋅qC - qD) - ld⋅sin(qA + qB - 
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
               2                               2                                  
     2⋅qD) - lb ⋅sin(2⋅qA + 2⋅qB - qC) + 1.0⋅lb ⋅sin(2⋅qA + 2⋅qB - qC - 2⋅qD) - lb
    ──────────────────────────────────────────────────────────────────────────────
    C - 3⋅qD) + 2⋅ld⋅sin(qA + qB - qC - qD) - ld⋅sin(qA + qB - qC + qD))          
                                                                                  
                          2                                                       
    qB + 2⋅qC + 2⋅qD) - lc ⋅sin(2⋅qA + qB - 2⋅qC - 2⋅qD) + 1.0⋅lc⋅ld⋅sin(2⋅qA + qB
    ──────────────────────────────────────────────────────────────────────────────
    qC - 3⋅qD) + 2⋅ld⋅sin(qA + qB - qC - qD) - ld⋅sin(qA + qB - qC + qD))         
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                                                                                  
    ⋅lc⋅sin(qA + qB - qD) + 1.0⋅lb⋅lc⋅sin(qA + qB + qD) + 1.0⋅lb⋅lc⋅sin(qA + qB - 
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
     - qC) + 1.0⋅lc⋅ld⋅sin(2⋅qA + qB + qC) - lc⋅ld⋅sin(-qB + qC + 2⋅qD) - lc⋅ld⋅si
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                                                                                  
    2⋅qC - 3⋅qD) - lb⋅lc⋅sin(qA + qB - 2⋅qC - qD) + 1.0⋅lb⋅ld⋅sin(qA + qB - qC - 3
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                     2            
    n(qB + qC + 2⋅qD) - 2.0⋅lc⋅ld⋅sin(2⋅qA + qB - qC - 2⋅qD) + 1.0⋅ld ⋅sin(qB) + 1
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
    
                                                                              ⎞⎤
    ⋅qD) - 2.0⋅lb⋅ld⋅sin(qA + qB - qC - qD) + 1.0⋅lb⋅ld⋅sin(qA + qB - qC + qD)⎠⎥
    ───────────────────────────────────────────────────────────────────────────⎥
                                                                               ⎥
                                                                               ⎥
         2                    2                    2                      ⎞    ⎥
    .0⋅ld ⋅sin(2⋅qA + qB) - ld ⋅sin(qB + 2⋅qD) - ld ⋅sin(2⋅qA + qB - 2⋅qD)⎠    ⎥
    ───────────────────────────────────────────────────────────────────────    ⎥
                                                                               ⎥
                                                                               ⎥
                                                                               ⎥
                                                                               ⎥
                                                                               ⎦




```python
subs = dict([(ii,jj) for ii,jj in zip(qd,qd2)])
subs
```




    ⎧               ⎛                                                             
    ⎪      1.0⋅qA_d⋅⎝-2.0⋅l₀⋅la⋅cos(qA + qC) + 2.0⋅l₀⋅la⋅cos(-qA + qC + 2⋅qD) - 2.
    ⎨qB_d: ───────────────────────────────────────────────────────────────────────
    ⎪                                                                             
    ⎩                                                                             
    
                                                                           2      
    0⋅l₀⋅lb⋅cos(qA + qB - qC) + 2.0⋅l₀⋅lb⋅cos(qA + qB - qC - 2⋅qD) + 1.0⋅la ⋅sin(q
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
    
           2                        2                    2                        
    C) - la ⋅sin(2⋅qA + qC) + 1.0⋅la ⋅sin(qC + 2⋅qD) - la ⋅sin(-2⋅qA + qC + 2⋅qD) 
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
    
                                                                                  
    - la⋅lb⋅sin(2⋅qA + qB - qC) - la⋅lb⋅sin(2⋅qA + qB + qC) + 1.0⋅la⋅lb⋅sin(-qB + 
    ──────────────────────────────────────────────────────────────────────────────
                  lb⋅(2⋅l₀⋅cos(qA + qB - qC) - 2⋅l₀⋅cos(qA + qB - qC - 2⋅qD) - la⋅
                                                                                  
    
                                                                                  
    qC + 2⋅qD) + 1.0⋅la⋅lb⋅sin(qB + qC + 2⋅qD) + 2.0⋅la⋅lb⋅sin(2⋅qA + qB - qC - 2⋅
    ──────────────────────────────────────────────────────────────────────────────
    sin(qB - qC) + la⋅sin(2⋅qA + qB - qC) - la⋅sin(-qB + qC + 2⋅qD) - la⋅sin(2⋅qA 
                                                                                  
    
                                                                                  
    qD) - 2.0⋅la⋅lc⋅sin(qA - qD) - la⋅lc⋅sin(-qA + 2⋅qC + 3⋅qD) + 1.0⋅la⋅lc⋅sin(qA
    ──────────────────────────────────────────────────────────────────────────────
    + qB - qC - 2⋅qD) + lb⋅sin(qC) - lb⋅sin(qC + 2⋅qD) + lb⋅sin(2⋅qA + 2⋅qB - qC) 
                                                                                  
    
                                                                                  
     + 2⋅qC + qD) + 1.0⋅la⋅ld⋅sin(-qA + qC + qD) - la⋅ld⋅sin(-qA + qC + 3⋅qD) - la
    ──────────────────────────────────────────────────────────────────────────────
    - lb⋅sin(2⋅qA + 2⋅qB - qC - 2⋅qD) + lc⋅sin(qA + qB - qD) - lc⋅sin(qA + qB + qD
                                                                                  
    
                                                            2                 2   
    ⋅ld⋅sin(qA + qC - qD) + 1.0⋅la⋅ld⋅sin(qA + qC + qD) - lb ⋅sin(qC) + 1.0⋅lb ⋅si
    ──────────────────────────────────────────────────────────────────────────────
    ) - lc⋅sin(qA + qB - 2⋅qC - 3⋅qD) + lc⋅sin(qA + qB - 2⋅qC - qD) - ld⋅sin(qA + 
                                                                                  
    
                     2                               2                            
    n(qC + 2⋅qD) - lb ⋅sin(2⋅qA + 2⋅qB - qC) + 1.0⋅lb ⋅sin(2⋅qA + 2⋅qB - qC - 2⋅qD
    ──────────────────────────────────────────────────────────────────────────────
    qB - qC - 3⋅qD) + 2⋅ld⋅sin(qA + qB - qC - qD) - ld⋅sin(qA + qB - qC + qD))    
                                                                                  
    
                                                                                  
    ) - lb⋅lc⋅sin(qA + qB - qD) + 1.0⋅lb⋅lc⋅sin(qA + qB + qD) + 1.0⋅lb⋅lc⋅sin(qA +
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
    
                                                                                  
     qB - 2⋅qC - 3⋅qD) - lb⋅lc⋅sin(qA + qB - 2⋅qC - qD) + 1.0⋅lb⋅ld⋅sin(qA + qB - 
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
    
                                                                                  
    qC - 3⋅qD) - 2.0⋅lb⋅ld⋅sin(qA + qB - qC - qD) + 1.0⋅lb⋅ld⋅sin(qA + qB - qC + q
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
    
      ⎞                    ⎛                                                      
    D)⎠        1.0⋅la⋅qA_d⋅⎝2.0⋅l₀⋅lc⋅cos(-qB + qC + qD) - 2.0⋅l₀⋅lc⋅cos(2⋅qA + qB
    ───, qC_d: ───────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
    
                                                                                  
     - qC - qD) + 2.0⋅l₀⋅ld⋅cos(qB + qD) - 2.0⋅l₀⋅ld⋅cos(2⋅qA + qB - qD) + 1.0⋅la⋅
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
    
                                                                                  
    lc⋅sin(qA - qB + qC + qD) + 2.0⋅la⋅lc⋅sin(qA + qB - qC - qD) - la⋅lc⋅sin(3⋅qA 
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
    
                                                                                  
    + qB - qC - qD) - la⋅ld⋅sin(-qA + qB + qD) + 1.0⋅la⋅ld⋅sin(qA + qB - qD) + 1.0
    ──────────────────────────────────────────────────────────────────────────────
                 lc⋅ld⋅(2⋅l₀⋅cos(qA + qB - qC) - 2⋅l₀⋅cos(qA + qB - qC - 2⋅qD) - l
                                                                                  
    
                                                                                  
    ⋅la⋅ld⋅sin(qA + qB + qD) - la⋅ld⋅sin(3⋅qA + qB - qD) - lb⋅lc⋅sin(-qA + qC + qD
    ──────────────────────────────────────────────────────────────────────────────
    a⋅sin(qB - qC) + la⋅sin(2⋅qA + qB - qC) - la⋅sin(-qB + qC + 2⋅qD) - la⋅sin(2⋅q
                                                                                  
    
                                                                                  
    ) + 1.0⋅lb⋅lc⋅sin(qA + qC + qD) + 1.0⋅lb⋅lc⋅sin(qA + 2⋅qB - qC - qD) - lb⋅lc⋅s
    ──────────────────────────────────────────────────────────────────────────────
    A + qB - qC - 2⋅qD) + lb⋅sin(qC) - lb⋅sin(qC + 2⋅qD) + lb⋅sin(2⋅qA + 2⋅qB - qC
                                                                                  
    
                                                                                  
    in(3⋅qA + 2⋅qB - qC - qD) + 2.0⋅lb⋅ld⋅sin(qA - qD) + 1.0⋅lb⋅ld⋅sin(qA + 2⋅qB +
    ──────────────────────────────────────────────────────────────────────────────
    ) - lb⋅sin(2⋅qA + 2⋅qB - qC - 2⋅qD) + lc⋅sin(qA + qB - qD) - lc⋅sin(qA + qB + 
                                                                                  
    
                                           2                 2                    
     qD) - lb⋅ld⋅sin(3⋅qA + 2⋅qB - qD) - lc ⋅sin(qB) + 1.0⋅lc ⋅sin(2⋅qA + qB) - lc
    ──────────────────────────────────────────────────────────────────────────────
    qD) - lc⋅sin(qA + qB - 2⋅qC - 3⋅qD) + lc⋅sin(qA + qB - 2⋅qC - qD) - ld⋅sin(qA 
                                                                                  
    
    2                            2                                                
     ⋅sin(-qB + 2⋅qC + 2⋅qD) - lc ⋅sin(2⋅qA + qB - 2⋅qC - 2⋅qD) + 1.0⋅lc⋅ld⋅sin(2⋅
    ──────────────────────────────────────────────────────────────────────────────
    + qB - qC - 3⋅qD) + 2⋅ld⋅sin(qA + qB - qC - qD) - ld⋅sin(qA + qB - qC + qD))  
                                                                                  
    
                                                                                  
    qA + qB - qC) + 1.0⋅lc⋅ld⋅sin(2⋅qA + qB + qC) - lc⋅ld⋅sin(-qB + qC + 2⋅qD) - l
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
    
                                                                            2     
    c⋅ld⋅sin(qB + qC + 2⋅qD) - 2.0⋅lc⋅ld⋅sin(2⋅qA + qB - qC - 2⋅qD) + 1.0⋅ld ⋅sin(
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
    
                2                    2                    2                      ⎞
    qB) + 1.0⋅ld ⋅sin(2⋅qA + qB) - ld ⋅sin(qB + 2⋅qD) - ld ⋅sin(2⋅qA + qB - 2⋅qD)⎠
    ──────────────────────────────────────────────────────────────────────────────
                                                                                  
                                                                                  
    
                           ⎫
            la⋅qA_d⋅sin(qA)⎪
    , qD_d: ───────────────⎬
               ld⋅sin(qD)  ⎪
                           ⎭




```python
pout #pBC end effector
```




    la*A.x + lb*B.x + N.y*(h + l0)




```python
vout = pout.time_derivative()
vout
```




    la*qA_d*A.y + lb*B.y*(qA_d + qB_d)




```python
vout = vout.subs(subs)
vout
```




    la*qA_d*A.y + lb*B.y*(qA_d + 1.0*qA_d*(-2.0*l0*la*cos(qA + qC) + 2.0*l0*la*cos(-qA + qC + 2*qD) - 2.0*l0*lb*cos(qA + qB - qC) + 2.0*l0*lb*cos(qA + qB - qC - 2*qD) + 1.0*la**2*sin(qC) - 1.0*la**2*sin(2*qA + qC) + 1.0*la**2*sin(qC + 2*qD) - 1.0*la**2*sin(-2*qA + qC + 2*qD) - 1.0*la*lb*sin(2*qA + qB - qC) - 1.0*la*lb*sin(2*qA + qB + qC) + 1.0*la*lb*sin(-qB + qC + 2*qD) + 1.0*la*lb*sin(qB + qC + 2*qD) + 2.0*la*lb*sin(2*qA + qB - qC - 2*qD) - 2.0*la*lc*sin(qA - qD) - 1.0*la*lc*sin(-qA + 2*qC + 3*qD) + 1.0*la*lc*sin(qA + 2*qC + qD) + 1.0*la*ld*sin(-qA + qC + qD) - 1.0*la*ld*sin(-qA + qC + 3*qD) - 1.0*la*ld*sin(qA + qC - qD) + 1.0*la*ld*sin(qA + qC + qD) - 1.0*lb**2*sin(qC) + 1.0*lb**2*sin(qC + 2*qD) - 1.0*lb**2*sin(2*qA + 2*qB - qC) + 1.0*lb**2*sin(2*qA + 2*qB - qC - 2*qD) - 1.0*lb*lc*sin(qA + qB - qD) + 1.0*lb*lc*sin(qA + qB + qD) + 1.0*lb*lc*sin(qA + qB - 2*qC - 3*qD) - 1.0*lb*lc*sin(qA + qB - 2*qC - qD) + 1.0*lb*ld*sin(qA + qB - qC - 3*qD) - 2.0*lb*ld*sin(qA + qB - qC - qD) + 1.0*lb*ld*sin(qA + qB - qC + qD))/(lb*(2*l0*cos(qA + qB - qC) - 2*l0*cos(qA + qB - qC - 2*qD) - la*sin(qB - qC) + la*sin(2*qA + qB - qC) - la*sin(-qB + qC + 2*qD) - la*sin(2*qA + qB - qC - 2*qD) + lb*sin(qC) - lb*sin(qC + 2*qD) + lb*sin(2*qA + 2*qB - qC) - lb*sin(2*qA + 2*qB - qC - 2*qD) + lc*sin(qA + qB - qD) - lc*sin(qA + qB + qD) - lc*sin(qA + qB - 2*qC - 3*qD) + lc*sin(qA + qB - 2*qC - qD) - ld*sin(qA + qB - qC - 3*qD) + 2*ld*sin(qA + qB - qC - qD) - ld*sin(qA + qB - qC + qD))))


## Middle of Typical Gait (#4)

As mentioned above, when solving the constraints and creating the mechanism, we initially made it into a position of interest. That position being the midway point between full extension and full compression. That point in the motion is where all the motion occurs as the system occilates between compression and decompression states to generate its peristaltic motion. 

## Plot (#5)



## Force Vector Estimates (#6)

>From your biomechanics-based specifications, define one or more force vector estimates (one for each end effector) that the system should be expected to experience. Consider including, based on your research
>
>1. the force of gravity exerted by the mass of a “payload” or the main body of the robot.
>2. the acceleration the system experiences during a typical gait
>3. ground reaction forces measured from biomechanics studies.

Below are some of the main forces we are considering.

- Gravity -> Friction on End-Effector and Rear/Base
 - Need to consider weight of motor and weight of materials
- Spring Force
 - Defined by distance between links and spring constant
- Tension from Motor/String
 - Enough to match or surpass spring force

Average Velocity (m/s) 

0.02 m/s

Fround Reaction Force (N)

1.07×10-1 N


# Force/Torque at Input (#7)

>Calculate the force or torque required at the input to satisfy the end-effector force requirements

Below is the calculation of the force required at the input, based on the sarrus kinematics assignment.



```python
F_ee = numpy.array([-0.5, 0,0,0,0]).T # Arbitrary force vector on end effector; need to figure out how to use the 5x1 Jacobian
F_in = J.T.dot(F_ee)
F_in
```

```python
pout
```

## Velocity of End Effector (#8)

>Estimate the velocity of the end-effector in this configuration. Using the Jacobian, calculate the speed required by the input(s) to achieve that output motion.

Below are the calculations done to find the velocity of the end effector. See discussion for more details.


```python
vout
```


```python
angA = 135*pi/180 #radians
angF = 45*pi/180 #radians
length0 = 0.0254 #meters
v_in = 0.02 # meters/sec
r = 0.0254
w = -v_in/r
v_out = length0 * w * m.cos(angA)/m.cos(angF)
v_out
```

## Power (#9)

>Finally, using the two estimates about force and speed at the input, compute the required power in this configuration

Power = Force * Velocity 


```python
power = F_in * v_out
power
```

## Discussion

1.How many degrees of freedom does your device have? How many motors? If the answer is not the same, what determines the state of the remaining degrees of freedom? How did you arrive at that number?

Our system has one degree of freedom and we will be driving our system with the strength of one motor. The links will be constrained in such a way that the output will translate in the x direction as we use a sarrus linkage mechanism. Our device will eventually have 4 linked sarrus mechanisms to produce peristaltic motion in the x direction. This would consist of offsetting each sarrus at specific times (compression or expansion) and using one driving motor, a system of offset cables can act as a new motor per sarrus. 


2.If your mechanism has more than one degree of freedom, please describe how those multiple degrees of freedom will work togehter to create a locomotory gait or useful motion. What is your plan for synchonizing, especially if passive energy storage?

As previously stated, our system has one degree of freedom, in x direction. As the x distance between links 1 and 4 decreases, the y distance between point A and Ctip/Dtip will increase, thus creating motion. The frictional force with the ground prevents back slip in the link giving one directional motion. When combined with 3 other sarrus mechanisms the device can offset the contraction and extension of each to generate wave like motion called peristaltic motion. Using one input motor, a series of cables can be offset so that they pull on the flat plate of each sarrus mechanism separately, contracting it. A spring could then help extend it. By contracting the first link, then the second, third and fourth, then extending in the same order, it generates walking. 


3.How did you estimate your expected end-effector forces

Our robot kinematics was modeled with bio-inspiration from the common earthworm. Based on previously conducted research on the biomechanics of the earthworm, we used the maximum mass of the worm as a basis of calculating the ground reaction force that our end effector will experience. Ideally, the end effector will experience a force unique to the specific segment, however using the maximum mass provides enough room for errors and overloading of the end effector.

4.How did you estimate your expected end-effector speeds

The equation for the velocity of the end effector was found symbolically through the python simulation above. For the estimated velocity values, we used values determined by the biomechanics assignment for worm speed. Geometric relations for angular speed helped us determine the output velocity given a specific input. 
