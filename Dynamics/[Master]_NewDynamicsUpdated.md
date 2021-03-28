```python
!pip install pypoly2tri idealab_tools foldable_robotics pynamics
```

    Requirement already satisfied: pypoly2tri in /usr/local/lib/python3.7/dist-packages (0.0.3)
    Requirement already satisfied: idealab_tools in /usr/local/lib/python3.7/dist-packages (0.0.22)
    Requirement already satisfied: foldable_robotics in /usr/local/lib/python3.7/dist-packages (0.0.29)
    Requirement already satisfied: pynamics in /usr/local/lib/python3.7/dist-packages (0.0.8)
    Requirement already satisfied: imageio in /usr/local/lib/python3.7/dist-packages (from idealab_tools) (2.4.1)
    Requirement already satisfied: pyyaml in /usr/local/lib/python3.7/dist-packages (from foldable_robotics) (3.13)
    Requirement already satisfied: ezdxf in /usr/local/lib/python3.7/dist-packages (from foldable_robotics) (0.15.2)
    Requirement already satisfied: shapely in /usr/local/lib/python3.7/dist-packages (from foldable_robotics) (1.7.1)
    Requirement already satisfied: matplotlib in /usr/local/lib/python3.7/dist-packages (from foldable_robotics) (3.2.2)
    Requirement already satisfied: numpy in /usr/local/lib/python3.7/dist-packages (from foldable_robotics) (1.19.5)
    Requirement already satisfied: scipy in /usr/local/lib/python3.7/dist-packages (from pynamics) (1.4.1)
    Requirement already satisfied: sympy in /usr/local/lib/python3.7/dist-packages (from pynamics) (1.7.1)
    Requirement already satisfied: pillow in /usr/local/lib/python3.7/dist-packages (from imageio->idealab_tools) (7.0.0)
    Requirement already satisfied: pyparsing>=2.0.1 in /usr/local/lib/python3.7/dist-packages (from ezdxf->foldable_robotics) (2.4.7)
    Requirement already satisfied: python-dateutil>=2.1 in /usr/local/lib/python3.7/dist-packages (from matplotlib->foldable_robotics) (2.8.1)
    Requirement already satisfied: cycler>=0.10 in /usr/local/lib/python3.7/dist-packages (from matplotlib->foldable_robotics) (0.10.0)
    Requirement already satisfied: kiwisolver>=1.0.1 in /usr/local/lib/python3.7/dist-packages (from matplotlib->foldable_robotics) (1.3.1)
    Requirement already satisfied: mpmath>=0.19 in /usr/local/lib/python3.7/dist-packages (from sympy->pynamics) (1.2.1)
    Requirement already satisfied: six>=1.5 in /usr/local/lib/python3.7/dist-packages (from python-dateutil>=2.1->matplotlib->foldable_robotics) (1.15.0)
    

![Alt Text](prototypePositions.PNG)

The above cardboard and paper prototype presents a conceptual proof of our new proposed design. It is bio inspired from starfish podia, used for locomotion and moving objects. The design uses two sarrus linkages in parallel to act as foldable linear actuators, that can be pulled by a cable at offset times to create unique locomotion through the use of a leg attachment. In the prototype, two strings can be pulled to act as the motor, and rubber bands attached to the ground represent the springs needed to keep the linkages extended. 
We are interested in the leg design on the system. The v shaped leg allows the sarrus linkages to move in 1 direction, while the leg deforms and bends. Later in the kinematic and dynamic representation, we represent the deformation in the leg as two two bar systems joined at the tip. The stiffness of the leg can store energy and push the system forward, or push the system upwards in a jumping motion. The system shall consist of a structural box (shell) support, actuators, sarrus linkages, and a leg. 
Positions A and B show the sarrus leg rotating, which can be used to pick up the leg, and push it along the ground to walk. We can also see in C the leg is fully compressed and off the ground, and in D the leg is fully touching the ground. 
Optimizing the leg length and stiffness can be used to produce motion through frictional ground forces, timed sarrus compression and extension patterns, and through the legs stiffness. 


![Alt Text](newdynamicspoints.PNG)

The image shows the necessary points and axis to create the system. Points A,B,C,D (pA,pB,pC,pD) represent the rigid frame of the system. All are relevant to axis A located on the top center of the frame. The sarrus linkages are shown starting at ps1 and ps2, attaching to pL1 and pL4 respectively. The Leg has 4 frames associated with each leg point (pL1,pL2,pL3,pL4) in order to study the stiffness and deformation in the leg. Lastly, the two tip points on the leg are constrained to attach to one another, giving us the v shaped leg initial condition we desire. 
Image 2 (differentiable, qa, s1 etc) 


![Alt Text](newdynamicsdiff.PNG)
 
Differentiable variables are important in describing the positions and rotations of the system. They are used extensively in force and dynamic expressions in the code. X and y represent the A frames distance from the Newtonian axis. If we set Y to a positive number, we can see the system fall to zero. S1 and S2 are translational distances to show the sarrus linkage extending and compressing.  The q values q1,q2,q3,q4 represent rotations from the A frame to the respective L frame. These are necessary to observe stiffness and deformation in the leg caused by moving the sarrus linkages and the interaction with the ground. 
Lengths are also shown for some lengths as the system is symmetric and uses the same length in multiple places. 


![Alt Text](FEAexample.PNG)

(example pulled for 12000 MPA, just to show one example) 
This FEA analysis of cardstock in Solidworks is how we are coming to a conclusion on the stiffness values to use.


![Alt Text](Motor_Circuit.JPG)
 
For our intended use, a simple 12V DC motor was more than sufficient as its torque at just 5V was almost meeting our power requirements. This motor and circuit is also highly desirable due to its simplicity, as it really only needs: power, the motor and a switch. Due to the small amount of components, this circuit adds minimal weight to our mechanism. Essentially, the DC motor will drive a compression spring that will lie inside of a sarrus linkage; these linkages will connect the leg to the frame. When the motor drives the compression spring on the corresponding side, that side of the mechanism will lift, causing the leg to drag on the floor, thus creating motion. There will be a motor circuit attached to each sarrus linkage, so the mechanism would be capable of motion along either direction in the x-axis.



```python
#importing packages
import sympy
sympy.init_printing(pretty_print=False)

import pynamics
from pynamics.frame import Frame
from pynamics.variable_types import Differentiable,Constant,Variable
from pynamics.system import System
from pynamics.body import Body
from pynamics.dyadic import Dyadic
from pynamics.output import Output,PointsOutput
from pynamics.particle import Particle
import pynamics.integration
import pynamics.tanh
import scipy.optimize
import sympy
from sympy import sin
import numpy
import matplotlib.pyplot as plt
plt.ion()
from math import pi
import math

#Pynamics System
system = System()
pynamics.set_system(__name__,system)

tol = 1e-4
error_tol = 1e-10
```

This cell contains the constants of the system, including the following:
lengths and masses of each link,
the gravity, damping, and spring constants, preload values (to define the resting positions of springs), and the moment of inertia values, which are used in body/motion-related calculations.


```python
#defining constants; inches, uncaled to meters

#Base Structure
l0 = Constant(0.05715, 'l0',system) # 2.25" * 0.0254 m/in = 0.05715
l1 = Constant(0.22225, 'l1',system) # 8.75" = 0.22225m
l2 = Constant(0.1016, 'l2',system) # 4" = 0.1016m
l3 = Constant(0.12065, 'l3',system) # 4.75" = 0.12065m
ls = Constant(0.009525,'ls',system) # 0.375" = 0.009525m
#Leg Sections
lL = Constant(0.01905, 'lL',system) # 0.75" = 0.01905m

#Mass Constants
mFrame = Constant(.05,'mLeft',system) 
#mRight = Constant(.005,'mRight',system)
mLeg = Constant(.005,'mLeg',system)

#Gravity/Damping/Spring Force
g = Constant(9.81,'g',system)
b = Constant(1e1,'b',system) #damping
k = Constant(1e-1,'k',system) #spring force


#spring preloads to system
preload1 = Constant(0*pi/180,'preload1',system)
preload2 = Constant(0*pi/180,'preload2',system)
preloadS = Constant(0.04572,'preloadS',system) #Sarrus at rest at ~1.8 inches 1.8" = 0.04572m


#Inertia of body A
Ixx_A = Constant(1,'Ixx_A',system)
Iyy_A = Constant(1,'Iyy_A',system)
Izz_A = Constant(1,'Izz_A',system)
```

The next few lines define the length and response rate of the animation and position calculations.


```python
#Animation stuff
tinitial = 0
tfinal = 10
fps = 20
tstep = 1/fps
t = numpy.r_[tinitial:tfinal:tstep]
```

The dynamic state variables are used wherever the system experiences change over time. Only qA, x1, and y1 are needed to define the location/orientation of the outer frame, as the rest of the frame is considered rigid and the other links are based on the position of pA.

The q1 and q4 variables represent the angles between the legs and the sarrus extensitons, whose lengths are denoted by s1 and s2. To simulate comliant elements in the system, the legs are split in the middle, and angles q2 and q3 represent the bend in the links.


```python
# Creating dynamic state variables. system argument denotes them as state variables for pynamics system from above

#Frame angle from N
qA,qA_d,qA_dd = Differentiable('qA',system) #Angle between N and A

#Frame position
x1,x1_d,x1_dd = Differentiable('x1',system) #Position of x1, y1, which are at midpoint of L1
y1,y1_d,y1_dd = Differentiable('y1',system)

#Leg angles
q1,q1_d,q1_dd = Differentiable('q1',system)
q2,q2_d,q2_dd = Differentiable('q2',system)
q3,q3_d,q3_dd = Differentiable('q3',system)
q4,q4_d,q4_dd = Differentiable('q4',system)

#Sarrus extensions
s1,s1_d,s1_dd = Differentiable('s1',system) #should move in A.y
s2,s2_d,s2_dd = Differentiable('s2',system)
```

Shown below are the length and angle values (in meters and degrees, respectively) that make up the initial guess for the system.


```python
initialvalues = {}

#Frame constants
initialvalues[qA]=0*pi/180 
initialvalues[qA_d]=0*pi/180 

#Leg angle init Values
initialvalues[q1]=0*pi/180  
initialvalues[q1_d]=0*pi/180 
initialvalues[q2]=0*pi/180   
initialvalues[q2_d]=0*pi/180
initialvalues[q3]=0*pi/180 
initialvalues[q3_d]=0*pi/180
initialvalues[q4]=0*pi/180   
initialvalues[q4_d]=0*pi/180

#Sarrus linkages init extension
initialvalues[s1] = 0.0127          #initially both compressed, then will end with right fully extended 0.5" = 0.0127m
initialvalues[s1_d] = 0  #0         #xhanged s1 and s2 velocities. Changing s2 makes it go through 0 as ptip 2 does not have ground constraint
initialvalues[s2] = 0.0127          # 0.5" = 0.0127m
initialvalues[s2_d] = 0  #0

#Frame contact points
initialvalues[x1] = 0.13
initialvalues[x1_d] = 0 
initialvalues[y1] = .15
initialvalues[y1_d] = 0
```

This creates a list that contains the values above.


```python
#Create initial guess list
statevariables = system.get_state_variables()
ini = [initialvalues[item] for item in statevariables]
```

Below are the reference frames of the system. As mentioned above, only the A reference frame is needed for the system's outer frame, due to the rigidity of the system. The L frames are used to calculate the angles of the leg sections relative to each other.


```python
# Initializing frames

#Newtonian
N = Frame('N')
system.set_newtonian(N)

A = Frame('A')

L1 = Frame('L1')
L2 = Frame('L2')
L3 = Frame('L3')
L4 = Frame('L4')
```

The A frame is rotated qA degrees from the N frame, the L1/L4 frames are rotated q1/q4 degrees from the A frame, and the L2/L3 frames are rotated q2/q3 degrees from the L1 and L4 frames.


```python
#Axis rotations

#Frame axis A
A.rotate_fixed_axis_directed(N,[0,0,1],qA,system)

#Leg
#left side of leg from sarrus 1
L1.rotate_fixed_axis_directed(A,[0,0,1],q1,system)
L2.rotate_fixed_axis_directed(L1,[0,0,1],q2,system)
#right side of leg from sarrus 2
L4.rotate_fixed_axis_directed(A,[0,0,1],q4,system)
L3.rotate_fixed_axis_directed(L4,[0,0,1],q3,system)
```

This is the definition of the vectors that make up the system's links. Each vector is made up of a staring position, and a length multiplied by a unit vector in a particular direction.


```python
#Define vectors/Frame/point Locations
pAcm = x1*N.x + y1*N.y


#Sarrus extension frames
ps1 = pAcm - ls*A.x
ps2 = pAcm + ls*A.x

pB = ps1  -l2*A.x
pC = ps2  + l2*A.x
pA = pB - l0*A.y
pD = pC - l0*A.y


pL1 = ps1 - s1*A.y
pL2 = pL1 - lL*L1.y
pL4 = ps2 - s2*A.y
pL3 = pL4 - lL*L4.y

pLtip1 = pL2 - lL*L2.y
pLtip2 = pL3 - lL*L3.y


#torque/force = A*sympy.sin(w*system.t)+b # create torque about frame, A = amplitude/max value of Torque or force desired; w = frequency of oscillation
# t*A.z
# addForce(torque*A.z, wAB) #adding torques as a function of sin(t)
# Not a position constraint (be at x position at t time)
# Use 
```

These are the positions that we have decided to place the masses in order to keep the system simplified. The locations are found in the same way as the vectors above.


```python
#Define the locations of the masses
mLocFrame = pAcm
mLocL1 = pL1 - (lL/2)*L1.y
mLocL2 = pL2 - (lL/2)*L2.y
mLocL4 = pL4 - (lL/2)*L4.y
mLocL3 = pL3 - (lL/2)*L3.y

mLocLeftFrame = pA + (l0/2)*A.y
mLocRightFrame = pD + (l0/2)*A.y
```

The particles defined below, along with the moment of inertia and body, influence the physics of the system and determine how it will move under loads/forces.


```python
#Define particle locations


#Frame locations
partL1 = Particle(mLocL1,mLeg,'PartL1',system)
partL2 = Particle(mLocL2,mLeg,'PartL2',system)
partL3 = Particle(mLocL3,mLeg,'PartL3',system)
partL4 = Particle(mLocL4,mLeg,'PartL4',system)

#partFrame = Particle(pAcm,mFrame,'PartFrame',system)

#Inertia and Bodie
IA = Dyadic.build(A,Ixx_A,Iyy_A,Izz_A)
bodyFrame = Body('bodyFrame',A,mLocFrame,mFrame,IA,system)

```

The w and v variables below represent angular velocity and linear velocity, respectively. These are used in the force calculations in the following cell.


```python
#Angular Velocity, w (omega)
wL1 = A.getw_(L1) 
wL2 = L1.getw_(L2)
wL4 = A.getw_(L4)
wL3 = L4.getw_(L3)

#Velocities
vFrame = pAcm.time_derivative() # with no parameters, defaults to (N,system)
vLeg = pLtip1.time_derivative()
vpA = pA.time_derivative()
vpD = pD.time_derivative()

vpL1 = pL1.time_derivative() # for addign a spring force and damping to the sarrus extensions
vpL4 = pL4.time_derivative() 

#Messing w/ cm locations
#vFrame = pAcm.time_derivative(N,system) #switched delocity to be relative to the N frame
#vLeg = pLtip1.time_derivative(N,system)
#vpA = mLocLeftFrame.time_derivative(N,system)
#vpD = mLocRightFrame.time_derivative(N,system)

#vpL1 = mLocL1.time_derivative() # for addign a spring force and damping to the sarrus extensions
#vpL4 = mLocL4.time_derivative() #default is by N
```

Below is where we add the system forces. Using offset sine waves, we feed in a frequency of 1 and an amplitude of .05 to lower the overall force output. Fs1 is the sarrus linkage 1 and we will see changes in s1 as it directly moves pL1. Fs2 is the offset sine wave directly moving pl4. The spring forces are all applied to the leg of the system (L1,2,3,4) to add some stiffness to it like the leg would have in the real world application. We subtract the preload and allow it to change based on its stiffness, and the force applied from fs1 and fs2. You can see the q and qd changes in the graphs later in the code.


```python
amp = .05 #max force
freq = 1 #frequency
fs1 = amp*sin(2* numpy.pi * freq * system.t)                           #Asin(wt); A=amplitude, w = 2*pi*fr
fs2 = amp*sin(2* numpy.pi * freq * system.t - (numpy.pi * freq /2))    #add (2*numpy.pi*freq/4)  ?
system.addforce(fs1*A.y,vpL1)
system.addforce(fs2*A.y,vpL4)

#Damping
#system.addforce(-b*wL1,wL1)  #seems to work fine with and without
#system.addforce(-b*wL2,wL2)
#system.addforce(-b*wL3,wL3)
#system.addforce(-b*wL4,wL4)

#Spring Force
system.add_spring_force1(k/50,(q1-preload1)*A.z,wL1)
system.add_spring_force1(k/50,(q2-preload1)*L1.z,wL2)
system.add_spring_force1(k/50,(q4-preload1)*A.z,wL4)
system.add_spring_force1(k/50,(q3-preload2)*L4.z,wL3)

#system.add_spring_force1(k, (s1-preloadS)*A.y,vpL1) #Integration step does not like this
#system.add_spring_force1(k, (s2-preloadS)*A.y,vpL4)
```




    (<pynamics.force.Force at 0x7ff66dfdb750>,
     <pynamics.spring.Spring at 0x7ff66dfdb350>)




```python
system.addforcegravity(-g*N.y)
```


Floor constraints were necessary to keep the system from free falling due to gravity. We add the floor to the ptip 1 (only need the ptip1 as the constraint later combines both ptips) and the left and right bottom points of the frame. Those would be pA and pD respectively. The code below is from the free falling example and takes the difference with the y and makes it zero. This ensures the system is bounded by the x axis. 



```python
#reference frame + state variable = need mass

#Ground forces on contact points

yLeft = pA.dot(N.y)

f_floor1 = (yLeft**2)**.5 - yLeft
f_floor1_d = system.derivative(f_floor1)
system.addforce(-k*100*f_floor1*-N.y,vpA)
system.addforce(-b*f_floor1*vpA,vpA)
system.addforce(-b*f_floor1*f_floor1_d*-N.y,vpA)

yRight = pD.dot(N.y)

f_floor2 = (yRight**2)**.5 - yRight
f_floor2_d = system.derivative(f_floor2)
system.addforce(-k*100*f_floor2*-N.y,vpD)
system.addforce(-b*f_floor2*vpD,vpD)
system.addforce(-b*f_floor2*f_floor2_d*-N.y,vpD)

yFoot = pLtip1.dot(N.y)

f_floor3 = (yFoot**2)**.5 - yFoot
f_floor3_d = system.derivative(f_floor3)
system.addforce(-k*100*f_floor3*-N.y,vLeg)
system.addforce(-b*f_floor3*vLeg,vLeg)
system.addforce(-b*f_floor3*f_floor3_d*-N.y,vLeg)

```




    <pynamics.force.Force at 0x7ff66defadd0>



Here we add only one constraint and then run the same code used in the kinematics assignment to solve a proper initial guess. The only constraint we give the system is that ptip 1 and ptip2 must have the same x and y with respect to the A frame (dot the difference in points with x and y). That way they always rotate with the whole system and always stay together as if it was one leg. 


```python
#Constraints

eq_vector = pLtip1 - pLtip2
```


```python
#Error list for constraints

eq = []
#if use_constraints:
eq.append((eq_vector).dot(A.x))
eq.append((eq_vector).dot(A.y))
#eq.append(pAcm.dot(N.x)) #Lock frame in place

eq_d=[(system.derivative(item)) for item in eq]
eq_dd=[(system.derivative(item)) for item in eq_d]
```


```python
qi = [s1,s2]
qd = [q1,q2,q3,q4,qA,x1,y1]

constants = system.constant_values.copy() # Recalls link lengths declared near beginning
defined = dict([(item,initialvalues[item]) for item in qi])
constants.update(defined)

eq = [item.subs(constants) for item in eq]

error = (numpy.array(eq)**2).sum()

f = sympy.lambdify(qd,error)

def function(args):
    return f(*args)

guess = [initialvalues[item] for item in qd]

result = scipy.optimize.minimize(function,guess)
result

#statevariables = system.get_state_variables()
#ini = [initialvalues[item] for item in statevariables]
iniSolved = []
for item in system.get_state_variables():
    if item in qd:
        iniSolved.append(result.x[qd.index(item)])
    else:
        iniSolved.append(initialvalues[item])
system.get_state_variables()
```




    [qA, x₁, y₁, q₁, q₂, q₃, q₄, s₁, s₂, qA_d, x_1_d, y_1_d, q_1_d, q_2_d, q_3_d, 
    q_4_d, s_1_d, s_2_d]



This graph shows that the system maintains a correct initial position and guess (in orange) and the previous un constrained guess (in blue). We show this to ensure the system is correct and help us troubleshoot bugs and errors that may arise when coding the vectors and points. 


```python
points = [pA,pB,ps1,pL1,pL2,pLtip1,pLtip2,pL3,pL4,ps2,pC,pD]
points_output = PointsOutput(points, constant_values=system.constant_values)

points = PointsOutput(points, constant_values=system.constant_values)
points.calc(numpy.array([ini,iniSolved]))
points.plot_time()
```

    2021-03-19 20:04:25,067 - pynamics.output - INFO - calculating outputs
    2021-03-19 20:04:25,071 - pynamics.output - INFO - done calculating outputs
    




    <matplotlib.axes._subplots.AxesSubplot at 0x7ff67a253210>




    
![png](output_39_2.png)
    



```python
f,ma = system.getdynamics()
func = system.state_space_post_invert(f,ma)
```

    2021-03-19 20:04:25,355 - pynamics.system - INFO - getting dynamic equations
    2021-03-19 20:04:26,506 - pynamics.system - INFO - solving a = f/m and creating function
    2021-03-19 20:04:26,537 - pynamics.system - INFO - substituting constrained in Ma-f.
    2021-03-19 20:04:41,108 - pynamics.system - INFO - done solving a = f/m and creating function
    

We run the system integrations here to solve the dynamics of all the inputs we described above. 
The q and qd values here reflect the sin wave input, as the motion is repeated over time. These are all the q values in the leg describing their rotations in z. 


```python
#Solve for acceleration
func1,lambda1 = system.state_space_post_invert(f,ma,eq_dd,return_lambda = True)

#Integrate
#states=pynamics.integration.integrate(func1,ini,t,rtol=tol,atol=tol, args=({'constants':system.constant_values},))
states=pynamics.integration.integrate(func1,iniSolved,t,rtol=tol,atol=tol, args=({'constants':system.constant_values},))

plt.figure()
artists = plt.plot(t,states[:,3:7])
plt.legend(artists,['q1','q2','q3','q4'])
plt.figure()
artists = plt.plot(t,states[:,12:16])
plt.legend(artists,['q1d','q2d','q3d','q4d'])
```

    2021-03-19 20:04:41,154 - pynamics.system - INFO - solving a = f/m and creating function
    2021-03-19 20:04:41,183 - pynamics.system - INFO - substituting constrained in Ma-f.
    2021-03-19 20:04:53,939 - pynamics.system - INFO - done solving a = f/m and creating function
    2021-03-19 20:04:53,942 - pynamics.system - INFO - calculating function for lambdas
    2021-03-19 20:04:53,955 - pynamics.integration - INFO - beginning integration
    2021-03-19 20:04:53,956 - pynamics.system - INFO - integration at time 0000.00
    2021-03-19 20:05:09,625 - pynamics.system - INFO - integration at time 0000.18
    2021-03-19 20:05:26,620 - pynamics.system - INFO - integration at time 0000.54
    2021-03-19 20:05:42,817 - pynamics.system - INFO - integration at time 0001.66
    2021-03-19 20:05:58,415 - pynamics.system - INFO - integration at time 0002.50
    2021-03-19 20:06:14,857 - pynamics.system - INFO - integration at time 0003.04
    2021-03-19 20:06:31,289 - pynamics.system - INFO - integration at time 0003.81
    2021-03-19 20:06:47,643 - pynamics.system - INFO - integration at time 0004.69
    2021-03-19 20:07:04,046 - pynamics.system - INFO - integration at time 0005.66
    2021-03-19 20:07:20,550 - pynamics.system - INFO - integration at time 0006.36
    2021-03-19 20:07:36,702 - pynamics.system - INFO - integration at time 0006.93
    2021-03-19 20:07:53,275 - pynamics.system - INFO - integration at time 0007.74
    2021-03-19 20:08:11,070 - pynamics.system - INFO - integration at time 0008.68
    2021-03-19 20:08:26,433 - pynamics.system - INFO - integration at time 0009.41
    2021-03-19 20:08:40,190 - pynamics.integration - INFO - finished integration
    




    <matplotlib.legend.Legend at 0x7ff66ace6b10>




    
![png](output_42_2.png)
    



    
![png](output_42_3.png)
    



```python
system.get_state_variables()
```




    [qA, x₁, y₁, q₁, q₂, q₃, q₄, s₁, s₂, qA_d, x_1_d, y_1_d, q_1_d, q_2_d, q_3_d, 
    q_4_d, s_1_d, s_2_d]



Our x value oscillates as the motion is not perfect, however it still follows a very slight upward trend, meaning over time, it is moving its frame position to the left. The Y value displays it dropping then remaining constant. 
S1 and S2 values show that they are oscillating over the time span. S1’s oscillation dominates s2. 


```python
plt.figure()
artists = plt.plot(t,states[:,1:3])
plt.legend(artists,['x1','y1'])
plt.figure()
artists = plt.plot(t,states[:,7:9])
plt.legend(artists,['s1','s2'])
```




    <matplotlib.legend.Legend at 0x7ff66ab92790>




    
![png](output_45_1.png)
    



    
![png](output_45_2.png)
    



```python
KE = system.get_KE()
PE = system.getPEGravity(pA) - system.getPESprings()
energy_output = Output([KE-PE],system)
energy_output.calc(states)
energy_output.plot_time()
```

    2021-03-19 20:08:41,484 - pynamics.output - INFO - calculating outputs
    2021-03-19 20:08:41,537 - pynamics.output - INFO - done calculating outputs
    


    
![png](output_46_1.png)
    


This position diagram helps display the very slight walking motion the system performs. We drop it from an initial height, then it inches to the left very slightly. 


```python
points = [pA,pB,ps1,pL1,pL2,pLtip1,pLtip2,pL3,pL4,ps2,pC,pD]
points_output = PointsOutput(points,system)
y = points_output.calc(states)
points_output.plot_time(20)
```

    2021-03-19 20:08:41,950 - pynamics.output - INFO - calculating outputs
    2021-03-19 20:08:41,990 - pynamics.output - INFO - done calculating outputs
    




    <matplotlib.axes._subplots.AxesSubplot at 0x7ff66e214f10>




    
![png](output_48_2.png)
    



```python
points_output.animate(fps = fps,movie_name = 'render.mp4',lw=2,marker='o',color=(1,0,0,1),linestyle='-')
```




    <matplotlib.axes._subplots.AxesSubplot at 0x7ff66aa46290>




    
![png](output_49_1.png)
    


The video here displays the leg with given sarrus input. The inputs are changing based on offset sin waves as discussed above. We can see the leg hit the ground, deform, and then push off creating a walking motion


```python
from matplotlib import animation, rc
from IPython.display import HTML
HTML(points_output.anim.to_html5_video())
```




<video width="432" height="288" controls autoplay loop>
  <source type="video/mp4" src="data:video/mp4;base64,AAAAHGZ0eXBNNFYgAAACAGlzb21pc28yYXZjMQAAAAhmcmVlAAFFF21kYXQAAAKuBgX//6rcRem9
5tlIt5Ys2CDZI+7veDI2NCAtIGNvcmUgMTUyIHIyODU0IGU5YTU5MDMgLSBILjI2NC9NUEVHLTQg
QVZDIGNvZGVjIC0gQ29weWxlZnQgMjAwMy0yMDE3IC0gaHR0cDovL3d3dy52aWRlb2xhbi5vcmcv
eDI2NC5odG1sIC0gb3B0aW9uczogY2FiYWM9MSByZWY9MyBkZWJsb2NrPTE6MDowIGFuYWx5c2U9
MHgzOjB4MTEzIG1lPWhleCBzdWJtZT03IHBzeT0xIHBzeV9yZD0xLjAwOjAuMDAgbWl4ZWRfcmVm
PTEgbWVfcmFuZ2U9MTYgY2hyb21hX21lPTEgdHJlbGxpcz0xIDh4OGRjdD0xIGNxbT0wIGRlYWR6
b25lPTIxLDExIGZhc3RfcHNraXA9MSBjaHJvbWFfcXBfb2Zmc2V0PS0yIHRocmVhZHM9MyBsb29r
YWhlYWRfdGhyZWFkcz0xIHNsaWNlZF90aHJlYWRzPTAgbnI9MCBkZWNpbWF0ZT0xIGludGVybGFj
ZWQ9MCBibHVyYXlfY29tcGF0PTAgY29uc3RyYWluZWRfaW50cmE9MCBiZnJhbWVzPTMgYl9weXJh
bWlkPTIgYl9hZGFwdD0xIGJfYmlhcz0wIGRpcmVjdD0xIHdlaWdodGI9MSBvcGVuX2dvcD0wIHdl
aWdodHA9MiBrZXlpbnQ9MjUwIGtleWludF9taW49MjAgc2NlbmVjdXQ9NDAgaW50cmFfcmVmcmVz
aD0wIHJjX2xvb2thaGVhZD00MCByYz1jcmYgbWJ0cmVlPTEgY3JmPTIzLjAgcWNvbXA9MC42MCBx
cG1pbj0wIHFwbWF4PTY5IHFwc3RlcD00IGlwX3JhdGlvPTEuNDAgYXE9MToxLjAwAIAAAA9jZYiE
ADf//vbw/gU2O5jQlxHN6J0zH78VuLo0N73OAAADAAA33OavHgZP6gkG4U38fnT0wTIgsInZu/53
iRIhcgcXmVbg+4AGovL6Fto23Jbf/6LzPJ3NP7pjIK3UBIT1ThkhePnrK4GZbumLlef6FnoRN5NT
xoPoUFAtDuEW33JTwAAASDzENrShPIhklmqxezTKZfnH/vznG7a7botp4H4uhEsaC5QQAY2e550L
FFY0N70m8qABUahA0dYJ7ny4J1l6a/DQYSXyw0fm6fxIl1RCLUqIWS/No/dD+UeI7hqgxe3iTPJT
ZP0NoCEcski7goTN/ErdA6U4GtcacbLp9CyV22wrM4zRRAvr4wahAoNv5EZzYNd6cp+CNuqCw6dp
nRpj/LlVFENa6pIOoLO7fDuhQQr1EbYBYQdAqmhqPi1fxqozozOcNSB21M/pJiHNzI27f9az/J14
x8GYqAUbZdPiNrsLMoUlx3f4xqba6xM9u1JQ5RdXlb6rtIVJUhC2FiWWTQeMUSE+FSwRHhSvmmCC
loIdAUWNAY40xKGT4CQwh7GMpEgxW/fzhYSoNk5Eb/lzllHgxuAuKpeiHhuOohe4kVTt5TUURaWo
dkcd7xsVn0bQWAcp8MtRkupSyuejrh+EPhLie0wZcR6dHixTKUbJfrMCldlOUg8h/5OQu8UYMhHa
LIw0D+FCNI2QLphi/dMQ03cd8TyNywFZA2YL+6PPB7XBovSdh/vKnwW418CkykoM4zlOGRtKkCSY
yrSusOk26N7u32cfo1NCOmjAmmU4ztWd3IHWDoWfqsQ0gKFDTh9QrAAEkfD1I8/U748Ozd9qs6Yx
SP6YrSa+J7Iy63IqeqrPprfIVezLSyYFN1kg5/fpAkGHfTVwyX5mgUhCqFu1ElGmFVeSuftzoqMT
gVgGWA8jb7LlhDUygFcYGVxjXeH+gsii0fo5cE9/7d7BDiGgiEEHTxxUUQqyRCCzF9y/iiEVZhKK
N0FRn1jQYVH98wzR6kFuvTcW2mllOJn7eQGgtrZWs4UIpz9ZmWqDxaH/ZlLbeCJR8L0Vphx04uPU
K44qTwh2jTcnjdK/JX9ssTv2Xr2NGF7EffhJfiJ5+14M/bzkElGoka7YKz+qNo21EnHRFnqkqIKW
eNhyh6PiX//6wDtimDPFXeHcvUjA38IrDdxgS33toRmu/hgJFjlgMOWhdehXF5wL2qPuIJedGsNl
E3wrcl9UDFFmcdZmRzGsADvagq7XtUCAHQTC27YeXsGwxqJDHj4aoLXH+8LFJpYbfkKv+ODYF04w
W6MJHrgepMsrELqCOX91z62/ORkhU0JmLpFoRUxBXyo+cXmQvqQdF8TX4h56q+TFs96E0XQByYnu
bQWlHBBjne9WIEfGujKD8doHfUHesq4v2AiX1OJtnMlFr93J0mtPxCqv7VsUCqumz/XdrrN0Zntj
QtmFZ/M0vR7wqjOKeICn3CI3OQ3CYy+0+g5zRYwhzU2p/aiTo42pdsB+FpYBY9Lq+Q1PQQdXPQGZ
KXpCndEISXn2dcyiP0dugycwGIebal1YbxQQtfBlZYWmeSUrAcVa0xcFUubNoaNbN73/G4eQpRK1
/YWQXi08FDE1Yo8ewyKwTh+SnSxBr2iXUswLEr5OIN6Lpp1ENT6oHMexGNDzXs7ZYYakUqFtQocB
al+j5wwzig31AtA9S4I7IIWa4gMHeaMfL8g8lNCTeInsY1bWKKYrguSIlZJm1UO0nRV26Rt4uhfQ
0Owxs+9ynniMPyqIVMw0n6eK0wvrvZSyxwYzHSJDtLwNyEOWabBVeCluycsNtxNN82wDBrBUPkH8
jlUiwTRAZLCbf+7QMcBgpsgSqfRRc8CXrPXNk8BsTQRkn1prjNrfQ0mU4z7p3Ic+a0mXNeS2B2G7
JyOWtqS1zubNCSjyGbvX/AOlYgTcICPzp/W/SNkx0ii08nhZRUvC5HpVGzyNySCuJmjV8eOBO9Se
BX1mn4+2paV4hHbJH63yaEQf+b9hDeTPQlbtDmt/6KvuIsF37Uhw4CRyYvCFdobqw5qE3R8cmUyX
z+4x63I8tHkMSvwTHaV1hKEQ2yhYIZzwRJUkzXh8+xqX6mMxZBom3Vgzd8IIh5vWFg0Bjyf548yU
Hyy5yUtJNh09lqqaZb/VKz7ODLcm313sehUP2YHMmh6XMNEv7RpPeoHfdaA0Kpjz5U34y/BfGBG+
5ruJZXTDl+lU+7Cder05xD2ABGiIXlNc6N0yrtamzzdiI6oR04bqlDqOpVsG5lTxCGWuJ3muCx/K
A9GtNfDRf4G3iG7BKZubw8SlqGaEbVPqZ/QgUPjmEspvvyvR7fFXeV+HYxLQoryB4cJRwBw5L3kH
hyBxx5sDUDZGiMCSJ3vjbc8ptjwSyksmOKXGxEkTjeYC+3JEuxk7G7bJLLZ+0FGLI+rORPb5/TD7
yhd4fNG8oyjY26jkVKEIOm9+cw4PnGUkfjGfKPr2xjvUoRe3MgJd5Bq0IBE7a5+KgpbrSUsh2kzd
Cebp8MlCORQpTIPiE3+DvE9ym4ZYgAIDJcqGouZCHfJnTwQSTSumZvCCh5Um9RAGnhpAklDe7Sbi
/cJIIkcNUaEjpEZLQfc9KfJu0ufYoNf0v/6T/MHnBpqtQjqvU+GXVyaFcrV21eG7+bdAqO/pYo29
W7EsBiFhnieTZEOIXdHqieAP4k+xrmv2P1BO1KiA8FKpa63h/CgbwauJDMAIXCdixi+nbmZRYJ14
4nYs7ao49xoAoJit+xkh7W9WR9nIc74W0R6cCYEl0PJQ0VO3f02TPKI3fsQcz/IZx0lwZCZqSBRA
n2QiVL1y0QDWIo51NBlV70+k6+AB7OPp11WXeyIfq2Vt2m2R+DHh/WmOQkeVspThq+h39FOlw9W6
BbbZNkwuruD899fR/sp93aznPfkHi0XHPOvDEoqbG7E1HV13YdQRqSG0Dql6GCvfOz/2kZr1Is0O
nFUlJEH7UW2EChUNLk5dhu4q5HDCLae5PC5Ids488v0fJDwQ8sHeqTfCbOqdjwPoOCR9zvaHXEsZ
/YM9JsXhpzmiAGLSZxqx/CrKnyuh4sklJR48OoLD7iF4+Z56LO+6+6t/L1b5Rbq/PwyTwVc8HUYU
91uwgr89OAwi9Hi4NdInwAAjoMm40sK9cFfhZZYBZAiw/zeWG22x35dhc5+l+3A4h6vf27LcfAir
TVVSCRSCeMYZtc4KlA9Dc3+2V/aGz2F8wjei+Ze/91xDJZjFKQdiktlHYTr0p5wzLL4Pns1Fhpxu
0mgI+ZzmESBqNCActhCI2s20bN0K6cDGz35KrA+BcYUlr69joIcEfcohg8qYifaqMcqKApQyGBMg
A2ACy7lVZ2QAnwyvPRoAQIrufGece4av2hqsceGYxyQSOZx2hRIvDU2TznPrYvGvJbMqWpMqXfXc
aB5+FJprRV8cj7eCrSne84IEcNg+GrgAAAUG/7J9HUwwJxgoEQYEEjZUVtar2lJNt0VhfoojoV96
ngLAujel6Gp9/haUas2WegY/M9Y4Bg5vv9UmkwtOlI8CRnB1l0fjluBrhPH12nM0e2u1wB05jupg
alFe2v7+LLe2fV62lAQAABoirR1JmUpOgyX83Qjandnir/+OR23f7/Rz3QvGsTwN8dENTCC7Wm5F
FpA/VJv/MYbltWn7HasNxW7bgsCbHJsCvCaEhKyXk4Eco4VkwkFEus8Eg6gQ8Atznn0jkPxB0XVw
ieO8CNEau3m7m+69g93jUzDAY7SvnDbprhNL5MQFkYFmV9vo7DCE5nKu/G0kDYawRCQ8soXuHZ1Q
V/4UaHHncAsRmHdRhNy3ErtYvNkRCelNvOgW6zpLeIsmO4MAAIopeSJD/YEWeQEKRw72cQ5JDdjg
CecdQsdK5DFHeqoIwqx2nVQlPreUDNU0Hl3kDUauEMMfRTL2XV/2N0aiT2aE08e/LXvoCaifDIP6
hWLGPsqZTjtzqWRuD1h9nmJXw2daNEXx5Kf3+bujLHZiG/w8803DuDbeHT/Yez+l0DkIgfNz4KJ8
Cv08qihu/aJ2sbol1ZtbnOpKU3vM7xMWCisJ9M9Ehk2GZRPD+1wFJAWecdI+n1GCaC10lCWaUeus
nAAqArv0ZdqkP5+vZzHs1KKJXb//uGvf4wYBdbMRoVmQmqGFV1fl+73NQGoRHlJPBwEPEOcvlYo4
a4yIOeBIkQfhI3tD9WOTfZ4go5pbvVVr7N2QUnE4Oo/pg01sQtmLK2+1CrMulxgbnmXWFvb1l/YH
kVvp8Fx7d+0yp1y3CRk0vNXvufwpcbjvUj3/hX/Oat+NLb95PG/RQvj1Oq7md1YXm+YWIJMGb8j7
976S1u5PU/wSUQ/BzjL4NYTbCaDNf7S7AM9DN8CNsF132c3G9y09/+jiuy/1XB3BYnJrJKi041ln
nkPJ1fj5etQsO6+MVH2YRMnu6/5bkLBTBBdy5Oy8viXNSu1alVju1WbKJWyCLmUXmCA+v5iAkKXb
oeN8WPe+RYiQ1c54TAHL4jFXDfRiXEYi2fF+89xychfVc7Yy5TZ15scGrA7/vfKsyQfja1/3Mde0
elW+ef8o8VmvLRdnP0rVHrlZ24yVnZDY3EH/LZfbTHO/lfDQBlS1nHZUdHEtpm7/ieINflulKsTc
+4ADefvLoSKj7JnB1pj21YXRsNXnS6q2AS5OpDMSMAxpTJNjlKacW9ldvFDAucs0ZPgXUjwBRZL5
q/vDPYx78GZSAjxjbgbxB1+S3iHhvBQJ8XwfMtuAnLD/vK0Ha0hUFR6Zp6rVVyF1JW3TcEvFlGDw
JDnM4CthyVQ5+u/oltslI9qjDSI/m0f+b2TqNvnIggRBXO7bFiSPhYxNfmlTZ78+/h2KM+q9Drkm
ZJdQ+qv6g+LsJq2+0wE6ThTH+8QuD/eQoT3ItCBqgpsfinzjbgmLze0t1kk5WB6k/8pj0qanV854
nHf9AGW7JRL2xH72NTb2VP/Hx41SwHUeFSOgpYecBw0TjVsk59fVfrbrj3d1a0ktDr7l1EL/3NAI
la9y4gddJ3Mc/2nFNinZtzRKlKkX/vqOuNvOHwzbR1Dr6Fvp7VNNC3bW42BiA/JCFHoGFtURopQT
HAspd7iLJMfa0YlRCjyanPhXN3J/WC11Gj2N9vNwWAwrVy9VM91+flH+LLVgJvuVoyUjOcWHbDTr
VLqG2o5egOww8B2KMnbWZa04VqwKHe9V+aMycPg8RmQ6Md10XNTeSMn6dP4n8o5NemQAAAMADJ+n
kLWBAAAEKkGaIWxDf/6nhBWcjYbF30biCGAnvtSrnQPw5g2BkhpJxDHv3rZ+GJnl7zIK/7c1lrnP
9ogo+MtDFqXyjP2W2QIwWCjjOx2kW7adlUJW8i3PtoGsOwq6sQboGV0z7ik18w1gv6CaZLZbNrkW
Cb0koV3lE1k03ArNtNTLI4wHnaH8GpbLQgjYUmzGJj75Radp7chfsvZ6L7Nq6eLpDQGv9pW0Kbre
gopMiSZyDsC1kmynyC4t7SKTkdb2KiFVBuj3OEsIk/DtGgU5ylKDygiUpxOiwzMxprJZ/i+mgrBC
JW3QeoPN63nY7rM/hpKtXPjagtmknQxw83suzNxuYtRJk43BUQzL+MLMXL6w1AfOxdYmx369dUro
z5W3OEnfOk7hdTOefTKAzIWniD0xeiC9xx8PU6h0J7JeuPUQYILZT9KwhzCclMfS0N/k7RXWy2PW
kOk7sklTebO8jpdcPyA60hZxoCOBrvpl6unEOiEMvLYYb0TiCoPswraGRKEeGHv69NTMiBe559DX
Md9an2Md8AQLYByzZmd0aoMPu7wOCOdV1CuBJR7n3wB1e+sDOLJjQQ0QHcONk7PSQgFO47VfjtYQ
rMuXSauMJOfZVCQM4wAw6c9JEzu3a+Fnj6QxjH8kTUeD+dRfspptPq4MUySl2/x3wQmMxXGMCybO
MPH64zgcnCxu3ezZLNXj/Zj64i4DNln1shK1WbV/ZVSH45pCbV6E2EDE/+tSH+x6OkknBznjxqwh
a7lvIUWbfCodyvry1Jnlshw8cassTo0SpfqsoRd871pvqcguYQPnYwFXy8twGbNSEhnP0GxTnYoM
zSK1nymxRzhNgvNxzs/PPqiDoc0F87nNlj9RbuUbwxpJgrJjD3t57LGyWzGeeyGL3LVZk7XaVtzW
Rx2jFcZ+rzPjOmga4n2M/rHZEzbmG0yiVxec+VhqG7FX1Y04eTv2pXE/+yZFQPTgy7/ZjjQnRvwM
U+y7kRmwvwY89+x8kiK3a9YCBMJ71anWAq8SBq2/oaIbXY3sXOYXRlD5nCuAUiAwJZtTdJUga0xx
6PiED9iuE+FhlAbTGPhPLCKdRQNM/fu+KYBXGcbjXVxqMzAcuYCcBP0OAo4YZs0zGdpSoQRbWaOx
ffyA6yAeahauU/Hk6C+2WyC7t2Wb7xCMcffOWKXoN7/cizMvn3OAT0PC4q/O0BOmGNuq3MfRz5pQ
uDUnUQHMPxQV5y5ZJschrE+kRw/Pn21YPuLFrQDn7Mx5na1gpkI08CH/yFv/FxG0XdUHFcTcHwJf
Ji3Rs4tH9sFg3RUqSNSnlQ00qlEoYzc71bXiJYFvUryHqMXI9pP+vqt5KeYAPC+0tyTp/M6osCqF
0YacdnnxDf+nROUTwhREAh10EbI+Y1khqBCbeVJl2FfU6vZBFjZZJchm3mOZObUAAAOzQZpCPCGT
KYQ3//6nhAHGmgH/BJYCQV70kTxXNT3N1+dc2EV7ExUMbPgIB2sphCz0CwGjqNWtdq/WRCfXS2Mh
6S+g7WIAAAMCwLS9n66BOL8JCuhePoNlIL11hXNMhBGUyBGOU4Og6ce9n0cB+vWe15kPEvTR2v8G
TbuQScMdmKiVcCtNuyTvgKFA2Dp9L5TZsB04blkzLW0phsZGe22ARC73UMgrHxDXsDyfBcCE8pVa
Epj5hdO4s2dVr426xeFVv/EINkhQnNH8eciX05s/AJTfKKwikiVKh7W//5Tqen9Hs03f92v/BYxW
KI0FUTVk/3qC1YmDYZMW81wUqIeHjA7bKLPclAd7ElWMS5Ektv0Dm77MUKp+C9uGgDQgUqrocHb/
n/lpj47le98Ttxx22T2LqkzCNOr8IvShj4Zsw/5znlz/bpkKBY+0J8U9+41KWEXa/+/SFFGO4HZB
p4nIPBmF736ogqxmGe9ss/U6m9YWm8dXptM1JoRLESMHlf+/O0V9kY9idvfrneoaIAlQXH2Rrcnj
1SrtAu8NVsH+RNheQd2ZCrk8XIqC92mxrDfalUH8SkTWCx7Gd2hF/SgPw0zvZtN6hAZwZf6kIKWX
jBEmiqU0IWtACXMXQgCmG6Nv2NBOv0NPkena2tgEQqTcUAfvFuP8uQADLdAASFNfsVSSBfB68Zs4
xcUTUFUf7Na9JLHj4vwmUCcQpAxUrbpKHkZy/wwgB4gI6Qycd/SYh72D6FvO24TjHFwhvvcF+/9Z
X/aZ2GBDHMqaZ4HszOK3eHhFkskbRS8M9ktAW1gA77bSklwbnysyH8etrRjjqiwC6x5OmN6n91bL
LShM9awczp8NPPHYopSOCeZe+p1JbSZlhIoaJjr/2B+PDitzZ3QLoUnKzsGOP9O5Sxww3r9XROHH
jcBbgGB21hpdECg7x8QjqY1WsswL7FLNrCpXT4A0t5ij4lPobhtJZWNZckuA0FZobORPq43vX0E6
tAALuaSgz2iTXDbBpzCwERJIXrsQ6QdfQPGNOBOTzAcy3FQMdSi4UWKQIKUcQe4+QArAIJVY/zGi
Tz61KHyHINSZoXlcVIrzptNArZOXNzeABR/LSRuaEvx91kGGgTaFZtx0eMBZpXg8U3fglr0EpQiy
uRqYGvlKfQhu+QfhqKIn1mnTcOEfL2xLbuucZIOQmWGsPwMntCnsVdjWKrJCpcz+ITTLX6oKEa6S
Ccf9tvMdAXgjDABNTPx1xvyp9LpoRyjqXXMClIEAAAQAQZpjSeEPJlMCHf/+qZYARH5Ha4bu62Hg
4qS7dz5a45wzkz6iOZ4ENpDwAJVGziOUCfPpvYrYHZ0ZaB4+QZXRdR1aK0SJLSeKLT62WHjpzKk3
UKMnd4qirpKts6x2ZkceDsSM0FJVwIEPufnUzdM8RKN4SNo9MbofPxSHlOD7C5komAutckVhULdi
tLwU4g9Z5V6IU+KqRDvCblBDBZ4a3QWUid8L2+UWCGQ5acrSnGZiC8w9mrLEiRENVoMcUJccSeYL
SwWSq/c1oOSss6udorGczi5X0NPjpdULRrMoyyz62p604jxud0u3N6nNsK4ndUKjp1KqGb5ORDKn
IJ9OdC8vJmiowcvGqnqUG44K6KeIlh0VoU2Cm8Z2CbzZReOyniXlRLI/l7FAbQ3ksFDfbHlk/dwa
45n1V74hQq4q0xfy23KAgXbkd3Vnlu6HmeEBUJpp8eugTq9H+bW6zdc6+3jchlfINcNMFV8h8ULM
N1jgSoqMwzg0uYiWt3JA1mc+TxP+WyuYESEpCl2WJyNRfLSYoCaWDYsZX2SIiwYtKt4l2vIW2YRS
sRwOafoZX/WeOqydNJkjpNu8xiraYKCXqt3nNkVg9UiGx2NOSoO3V7catcRr1UdJiZ+rPt8OkZTt
Fqexl+FQUdPxO1IowfGhFz8nHmUVdmTUNkLJEcnLNCUFJ6ptFUkokVtFonceV5LS02TfZZpPDmhG
Xqg9yoE34fH0ptLk00ehLW7tNl2EyBNbhYeV2uhg7WaJV+hUMml0//gYLrj9wz2GiGYa3ROWm5VZ
FIOJfB84egb2vpvzm7DkEGwSgCOrMclCycMm8Rn9I0K/JoKC5U3CNOr3x1B9cZ38nI9LEM3As3WO
sVR+RDhoaTdbGplFPtQs64VyB62gwhg2W0gD4tR4pw1VgjTyJmmGFkSUk+psbOWVQD2bgrehOnI+
8bmmNbumy6SwAOxwjYMuOVYRwIw0gFnFr1+emk/uYEzz83/SlswNm+cppyYD5wMuX44KUu+RHrI3
ovyBr/ji1oF5rpbDU4u+7CgTvXoLCd/1sI29vQ59DN0moNZMzen8jeZHEbUXuNAd+thd6CAgCmIV
aDjZSKWH9YIsO6YKZHuo7nwBmg0RF/X+wG4qviD91jTJQ2dE1T1LzJNTVfbupl6B1mfrMq8232mC
xNoEq4RdphArKmnZFvVnQ2MU90r0uHoccyglMTKxXuJjYf5wnQlx5WlG/pc2s6oGoDlOEQ6f8ZH/
0akoe9k3S/Y74mgyMBDN1XJ8Y+TQ3P6EZ8+RWvuPeyCe2nzWVVNBGAHyEcCrAdOEvigVB5RKIvED
HjKAY0RGbnxSIeciAtIB0f19HxjARS+iVarx2JEAJgAAAktBmoVJ4Q8mUwURPDf//qeEAAM4TvKQ
7poAWr8wvOULfAbXq1X9I3HZ7dtc9pSEX3ADLiFIVIC0xUIZgIYC9rUAxq3CEE+WJ5d7cxq+fY+C
FyHNiboLtw9kLcyLZQJxrxXvpXIXWF8vupntZKyVhhu6PpmdAUXSRC79nk8DBGLO0GOTN/VOkVm7
KMKuMpzoY4TTm+Ooyu6zCbDGk1fnFCg7tgBedk2piNjDP17re8iHkr98Br410nGkNLdT1scGJxF6
5cBO80/HohCjiNrB7kVHf+IBzwAPP52q5TnX+8fBIHhX15AqVxRqBPOW7F/DU8jlTsRn5wpJ1Fdb
Ay6ovlRFUytYVOzuKXMmSp4dDyV3IAr9aerCDCB8paUSjULHmniB2so+TAUqhs34nD2C7QLe5Ex1
jK25ef0B4B+Di8K9vg7hMeYF78Et5otQeFYqOXBEmUc1e7CcEeN8aWhJ+P4vGZHBsK1WnXb351xP
maJE5ZvD/Frp9Zjy3YrzQhr5hd6ZWwD4D9xxabulFtpfLizkNowveVbtAEyS9ecc+TG/H1wnT4/k
NJ+M9ev717PfT2x55wS9NlhC5FSOvV7EU8M7d7R2E3HuIV0MTsEhH5HJyr3cf+bvRN3CXutvO3eJ
1MAiiEZCma0YqBh5ymX8bMimuw34ZIzjyYXx5wh13X+J7UfuoboM1FulzjGL6NXL0L9WAUM8xzjm
e4an8v0Ro4U7ffNoQhqQ1jbjt30FcSsT4Qx4JhSHFZQvxmkmaUz2k1UTpBDKw+zQppO6oQAAAe4B
nqRqQr8AAplmOzjbZOG5TB3j9RVenBm3FmI910WP2ciFbuaAARALw58bWrbc4u6EgcWysMjkxSaK
DJ7yZwW92j9umeMgQabkAAAI5CATFdbsNV3Lx0NrOTvodVfYqmCTQofgyk78FdhLAbnwR4A4jRE3
mLYERHu+9zDwWj/NoHdMFyVZIM2StPPAQZPAUtHy9BBhJiKtsri0Q2aCYkYqTkO+A4zqA+a8y7l7
C5R2jz0vAjECDJFWXy6sFfEy4jPzQIMdL4jhCiuF/r+Rz3Nlj6wrnXTcuVUBDlaOra+T00VxULLJ
QmAr5c25yVPgZ5JTNRqp6T5B406o5M+9P5UsC9V0nbw0XPSPH4oGoCEwiBxQKX88ngZaOrF4UNMh
iIcZOd47rqDDrln1BaXT9P/zUm2QInQOKfedjMl/pQYwM9Bd0zK87ZpCUjV1k7PDVMtVMpAzDj+6
kYl0t/YkOPWibHuEruJZUQH439J8V9ZVzfeGOgCU3wzWCFMgoNSiXfHnF5Mv1lzR8BNCta8Ivmyz
OqmSM0ooYc7GWs45OoNjL35taRFq5LAQMHOcx2Y98Ibqq/BL9Ag4KdDp1sSLqM5InOjxUPAfOM3v
VnyHXi1WAeiix4eDl63LkuHNxj02m70VmIXdZDOLIMa8xHdNQQAAAnZBmqZJ4Q8mUwId//6plgAB
nrXCuJckAJZ75EKjLY3xgF0q1VTqP4div8KY7cK/Xxlud0lYmXpjpAHp6NxDVS0cIen9Jx/Y/Wca
dNqd/VLqehb7vOtPxRCbxbejmt6LHJY4l1p6kcQcG6GbgGuUNtHSCxfh+rFFOMiBuA21AAADAy+t
2krCM0gBQCi1+Hd0VenD7hXuRMJOjGXU4di4jdEM4jYrOIvGcPZmR/BrLehCPO8yQ64CIhdHGDOg
AA5C1sU+BFkyllJkdhKp/NWHzUK8FP7h8MhWIKTbKHq4gxegEC93hOTE3znhGeE51qK8mBxlu22u
fMjUzwYYvyrZT9rtZ02K2fqfTdrNEJnqgemS9V0y2Wt2Ac0IQM8ZPyJ5qfxEEW0VDHP+GVIlS9ZI
qNvPOW7zVs1Ip/uNlBvRAq1IM8TjgYR/KGb3mEB7r7+2N0VU6WNUMZUV+s1DxEOkdtEN4C/9MVzZ
GCgyTUuz6kQAHgR2SKDOWH4HXoZi9zKnIpWm04P36K1vV4gZfjHPDxwQ0Vawu0ptLUbzauRJQcFj
dGS4FwExVzmHsU5W42aM36r23AdjomI3yZ40L/5b93wl/5qJAEGzWcp8dZxvxIX5mkrzzWeHSXPP
VMG/lRKGHQHKvwCDNMmVOEtBzYwVknk74IEXQjjyUsB1eK36vndJWSydgviapTl+hJHjsTBOuxbV
RnJ/Q6IuMM2pwMSpUj6u1pzZSFfYeizbHsZEqLMW9O4FQDenimHnqO0NSFNYN2Bf8bqvsoAsWgYR
rFZzGsWNXftz8CzaJ8wWvSUznsHljRgDZTRL4euPYlr9pqCAdu7NyC5HFD8AAAK3QZrJSeEPJlMC
G//+p4QAA06ixlwBehhKlomCZMLrhB+CLogvfVeV/qoy2FqP1eyANzAUp+C/ysgbCYTB/Buwvspl
YscVtROHskFCjahgJxiILgbUr6Yn1rrfkHxU6gg/UXd3SYDKC2JogHswgX0w72GqtXGEzmiWp8v2
ob4oGzX/eHcuVx9k8L9p5ab4qjiTowuZY147O8HslPdhsvuJWZNZ2W38JpcLzZyI0Yg+y3Yn7pZr
eAeGZ/M7zwKLMSRYE7AkRLqAOwKvTQBVxwTxorOFDdKi9m5m8SX7rBRNuf4hBdkVkv/jHEQrXecU
6U1++z47g/nLSuIUEjMMZ/h5ES+n+s3JV9d2Fxrz/Ezx9Jf5WVeiPysuPSU/xfIGcm5tsOWzBL++
LbAXeocLkM9Y6o9cZSBs7F80vHTfa0UJ3Jcvu4llNCETyAWUib96DLhjxYjqcgs26hRr65zdTTAt
l/hBJBqcttr5UGQwppYrmQ7behWnaqHSSrsdQnLpqWgyBBiLIRP0PBCFvDgKQAFXUvXWQSCexmQE
MJ/9uFz3J7Bsfz8FiXIOpjD/9we2sWD8vOpj3vO3dlmu6jUvxukvotAPL4Nj2oxVVH7DFofVUAPM
VzNHSbrD1ONmnFH06BCH+8Tp7iE595wWWytQXDOxc70uA0H+QDag6q1Jb0uZYCXSpuNyn8EuDRCs
BYMNFgFNY+K8vqOHF+6ECkdQJeF/NDAJysJuakjYh5JlM15eUP6wTpUwZl/I+IarlZKlKPO4FJh8
33l+/y2Gz+IGNh4OG+XoKkmj9smeV3x5NJAIswH6Uz/pvTTzcc08od+diNlJE1D+sjDOLmf+5Ndz
W5b2WPnH6yIe4rsiAngBKzZsZkn97vw4oq1rLllL1DhB+mU3gsLLLrU713jDzDjIPpu+b2v+XIJh
LvUAAAHTQZ7nRRE8L/8AAfFLfDLR99a26oATVZB98kMyy+UP+xKll+BxTSL2nf9GmYG9VBD3+gyB
T62OejWemlxYF7PkhkXwwrZJHofZfNSeHAeI55yvcHG6imFrxnlduQGpX6lCrpnCcnMkVzrn4R9m
zdU3+2ntA9YxgfRMtVO6ELkf4uY9cOr1LvPT2jZpCbrmF/xBGRgnrDnKtjusGZvgcXRbIqwS3ego
H27MYgCFI3tU4pyqmWZSfwghFQvn5LkhuiaHuPdGkMFLEpBaQcMeORoru2mdYHlnWLkpEttn0Mug
EPYszWZ1qBcZ76ScVRQtuwU6lhKPmaPubTubVjVjnVJP1JK2PJYkUjvG/MUD/N6l8GEL0Ygiabmf
1GtIjb4ymi6+7GzX9i/JmgaAWIpLgMOkEYCRC+mlylJrYAeEVgPsiCXn9PsgsYvu3/iDOgrKSvKn
vACqCCgfh15YxTcmVhTYbsvZHrdCozCvkDsf5ubjjP+MDlgfjUaZvBa4+6XrxxJxuN/jePdWftjL
GAOZ2IOA980KvC2/WpnzEDB8+6AEukQm0RU9IN87k5A4l95m3WSL5/VG8IXHNcE5VKM7YbrBSEsA
F06U1wRHIayZ8TtuegnvxuQAAAETAZ8IakK/AAQ4M2USxMqPMtmoAEMslVuE8SqpUPJ/9oGEeZ6g
5CM85i/avpjEKy3Sg5p9PlIHT1yAH+4OAxL9mprwSZgd9+D9w3DUlECJ5o/h74pbw1tT+2+GIDgJ
60JBmSrD6nWKpV8fAt3GdKv2eW4y9U0KsJfZaFHRA/SNlRdqnuVF3My/C+CS+X32bBzalUBTj1i1
mKrqyrILXr7EyyfUOJ2hSzFlrW4s3NUjwpnLrUtVs0bcPlXrIkfakVkKJEBZ+VEK6uve0Jh5dwom
+2El/8Mo0ZpHaiwOJ0pCk81aBqVZGrTlmt7rsfd/G72HMIwe7g94xQHTD1Pgx+NtEPqu5fp34Q8/
oSPmwhxQLmfAjNAAAAKGQZsKSahBaJlMCHf//qmWAAGoT9CPA7ABXUtFuVMFE0l6MnmLhkXbm9eG
W3n5Xuv5um6UP5N5ovgY6nkBwv313vev+5tGoTqTHUwry0FMWZtUkKOcexMOFjBOC3ev/MbnpmWn
kLwaKbYE2/LzhPbex0wvEQEdUulgomMbkZIMQlrcvN37PJoEJEIZAtlk+/7/svn9+0sldbeDFrFM
Kn9ZLKKo+azqnsPZQoPp9OrFWq+7EXeOJI0fx5RWk4iVTNLMDueHtT0137wEQIpZzXvWx5rFY78u
/085NGa6ZO3pTNxe/mlGdWQVgDUAYnrtXBwJtEWjBt6hDGCsOhoqKe3/k64Va+kLnnGCE+2Mvxbg
SkmVExH0ceIHzBElqgcFiZuoGIRM3sA6U+eevRO1UmCNkhiP/TAKqnge6pOqGfyE3zX+MCFwhWDE
O5jhA6lVguRO/BAfQJAJsJBOCB4fOnQrGfUhnp4ECdjXule5wTrIgNXksh6nlOwuDw70E1KcZGA6
g9+WxoPduCHPXOohz7a9JgM+PkI0STuQWSSpZ5omJ+y+j1YFOkXlOVMpZIAudwg/WQsC72lVbA3/
1iVUdX0MRLrCBctxmFmYs4GYkHNXReV1W13jb65In7ZZeKtQyqV/wSp6rTTRo+kezvTmqnqLyoxK
Q0M40eU9Btjkd39R4744qgAbu9jAhr7WvYGIMiG7xcIiihH9eSKjIh8l8MqigtMUvP0ESYNBi6g6
KdJ31+uAhR/8ztZCoe8FoMQYO/nGoArUOrYLLeSiwJzHosBLCKwHGyCNHG7XOIaDwRpcVMChpOWo
5VviyVVFqAvCO+vhUPzCrUgDXISwKEy01Nqzo8rK+eLDrwAAAjZBmytJ4QpSZTAh3/6plgADQe2h
D+VzbQQBG8tUnc/fEzxS/Aw/xHJIMSEZE7djqj8mj07Y9mz5mZyqeSQBQ1ebG2Gb5GrxkUVutnP5
f0KPe63XoxjyE8C27Bf+DNQ/pbD2Y7EGt+kh07d8HGFmP3R2KKhS0TnVXReYpsLvkx/fLuwHhscE
JJxXAj3AyDgAXgMcrihyRM2xVw+KBW6dAqyjIlnTr+3/ZQIoetFQCAgigerhUC1qdxDMDoeU65QY
uAb2alP4wDXjhJScxrwBbxqvWeFsAmGb56wXUWJEHC6NXjziZlDg5TI9dnLE9FvOW1c+Ojj4Gj0u
0l9WEg0rJeY6S2yJzhJebQ2deoX6z0q5RIyPWH0Xnouy37f2jsVZuYQhy+g0HmSwg+3XBG1wiaP4
Rs5smy/PMygodEEWGxbk+hjetndxGFQL3uQ1ZYd5vZAMv0q2FBPuY7c6CL6+/rLBmaG086K5Uv0p
bgNJWcWQJ33S2VtWxfs6pRKjbu/pGYxlcxrGhjg+Iy/v4vFOWGzT/CK9TeCL69eDyVdmPOALBlPm
LcmdBwgOBJb2+Uvz+fi1Z+iWDK+zI1EFr+Rc/ESyV6AoX2xwjIMBgQwaMMZjQ/c4bSJqWqKVdIb0
mp6wXpp9bKOs8shAzztnsuU8IlVS9fpjzZUjaPO/aB2F669CX1dd7N/mwXzDqE1PfweEL7P93GDb
HUVwz1Vc+QdfRuYnIAF0N+tSZ8bN3dXHM1rHV9kzAeUHpAAAAmZBm09J4Q6JlMCG//6nhAADSyC8
gCMIctz6tfdxPVSMZCpCBOWeG7yNpP9tloQnv7gSnuF+qTU0DTa8t+AVzDyRsPFr920KsofNThJo
IgUiYF8312oWaQB8dMxlfSxSsyfT5v3dmH/EibanDqBUnD484GAt+sEG2hAuUeUQHjRRdPo3VNOF
8yyUIEI4KO1X1D7U8CRE3eDXfMRWXOFir1HCn+kGTYM93IZoDmhktZ2kfTuyHBT3aYqZ0pTCPWfD
URo0OuSIX8DOj94p7cpJ2iv1quMcIpiq17rvZdlTlE3YkrKLkpw1UtWuPEpD/SH7s5zpSPStwbWH
JTV/wwOzItVare4jahGJ6xqLgxM2AXI/t5izq2BjaVVPXgUmADFu++WF//P0vX/w4P/5t1L4f4kq
wcLVsyHcv0H4jOmbkRSmChjdk3eZevBDX0vZKm6VZShEVI0N/+XRFgIZ4JX4sZcHTda12jT90ieY
1m+eMTjfTtkQHWOAelKFeG/wufh1sujYkyrlwSbSpuoGnijzE4Ww75ObEfU2SplpSnZ47pKzz5+6
72By2IQK0xLIk3bu3Gsxb4X/9BJ2jLtLOIU2YRWNk3B4wwSh5VqlagjwdU2vEbRYD4n8J7d7bFSE
hX4gLPr9Lm7LCsIYoZbSXnI9W9qHhF98ORDROoGelqYcRpEMOomyZRmOdBCCdMcoJawHroGmU6bs
hKtg7NisKykevyeXEGsida2fiMp56pT/P8r1Itj3C7CKxHLzJcvde61tQaCdWMT22yAWDiE6dGX/
9ZPxqCMph/EuqvZKYI5HqiHxoANyxcbSgAAAAS1Bn21FETwz/wACz0qNLXl6oiADZ9+jEO4cdsAL
DmYE7ty/wTly0AAABNn4SEgfg/4voepzj0ROQplXnasCjDuxJAnj4W/fE3NQbR5pXVRqJ1mW3/Ia
g5rbY1TOssrs+9oGB6fHqLG9k9YcyMBo7Z8miWPseKO5h/QSRuUloRmK9mtIbn3H8ldAgK3SZXdk
hNLirw2Dk6C3KJgguZXnXC05Q82rMrAsNv+WA70enxuX7ti0BXnjC4DRTIyFgzqXv7e9GihVhBox
+82nDmLph+egbI3+LlXp3BCz0+c2cbZBjDXMA0erCkY4Djdd/w2gae9j4DU1yqbTw5RpsfXL5DBd
4oSWS6ZIUn3UlZKZmci3AOvDsLyAzT88lCQHfCVZl7aBaLFCZoHJvMZll/mRAAABTQGfjHRCvwAF
QTNsS8V+wEBxmFxqdyAEu1lx3jSSUg/Rv8oAp1Fn/3IPwT16BOcmbEY2l4PV9tEtxk82Pe7P/bwe
kzAfoD5l1O+CjLtexgWBZV+dtlonJk+IOsjaKVtfeOXHasdioCyfHVWa55lozANpTlJFsePwGX2n
AroIodETfvCLWxu8NjzdHMH+4njc29r65QvCF3SWmcFA1CD4UIU4U0+fDgCuM1ohT0ndVeMidwbg
oXjJLR47mxbTMeSLquaaQ+N5zrykqkvyLKXXkEWNEnedxPQajQkyXgW+SvtpvcqHw0FQibh47INr
0fRQsqIy+wbWVv32DEfAPFeBfEbHztaOnvHpaioUNZkXpoOy6N8Z0JpZf/E+symGTeBcYOwv1XxQ
Hu7v4z5SGwDucWRGC7MVLjtznHJXBFn1cNREXbtjOaEvpzlw0ItkYQAAASgBn45qQr8ABULCfYgI
RONcQZjVjncLTdomlozj0AH7QqgnOW2IffD+KiNUKprpD2ab2txCYOK5KC9w8HP66N+mZikTost+
OPbXRyn/g8M4n6Dt3oqaB/4DHwPfWjglKXlxC3uhU+0q0iYeHimLWcbslg0QCem4yx4nadI55XsA
yHjBPzW79DpZo/iVyhL0pYhaGoJeVZsVDFdh/QAjnTOALfPM5AJnnZ/C8R3Ojfjmv7LqInyL7rTD
0pAeTOysJkqdSS3EJw/pc/xeVZsWFQjP0vh8gKWYPzGNdMTHj+LlwQoQLmfUsauXuPy2gWKUJpFs
cF0GzlPO0QUlt4LcoO+S2Trks7lBTDls+o94vZZ0Tk90C2j7Rvuud51rHUhQ2ZM7EcBcBHX7uwAA
AblBm5FJqEFomUwU8N/+p4QABf/ZT8GD1XeHZdbNABWvTSOvbu5FL44I7cN/8wN4GtCCqM2gnEl9
Zt8kG49YoGzkP4ALeQwtifxNwrf/MKtQh0HU8CB2bF93Mm3UB6LpCJXYWF9WvqwxGfA+y3RrwmUR
Ou7F5XHSrr6ym3+054YSL++fpj3qbL8yj5iwhpbuNdWv+eHFrW/NWRQP/mx1L/DW/NVji7vLveOK
Lqyd9MzvrI1EfdKrRGlJ/aRlvKOBJXJSZNHu2PtG4/xxtbn+RiBx3QLbGEcWHCjLs3T2joanoLpc
yjVPz5ri6yZqaKPK0j9CW/BumwRSNjk6s+8Nv98eGx+wKS/ujRXm3tgjZq46uxBs0bAp6eMALZem
dZ1SHWbNaJ5zmt8EstzQ+w2EjTXgRiKqZqtQNocsE4YQudWBfalrMP1oMlMxHtOL4Cel3er8k1FK
7hOM6WxokAxzeBtvWJBBd4nX75oUeZH56rM3fuLmuamKo57ovLenFE+9krimdfw/Kq8gEzxT8P/g
oiIgptn29Sq34DOewvFLLVdYt1niwVbnTpu6io/f2+yi//KsbIoDnSl016cAAACVAZ+wakK/AAVD
k+0fLQc0ZF96NMm1iCO3wACH1sVhzoeROiaso54l3tGBWhm8KNC3Qhzesr8PKhwOUUdkAhbq5dqF
ttm36sXVY77k0D7Q05DdHYAFWsaThtOVTV++O4hseLFISkSQTy1EVgsKq6hNFHC6JJuy2in/c3HU
vgK/setghUxRrCAZ/XZiBRuvyNRBAZ1K7zgAAAEjQZuySeEKUmUwIb/+p4QABf/ZT8F6RD4vD0aA
CuoQfw3ticLSmit/gf4zMB/Qu7Bf5WruyCTQchO/ynhCOXw/oZPiOrhooDwR7yRzcnh2XeisbZb1
qJh2E49lBl+MAE5UMaqXTZWgRw6ooTaS/08otj70z14S1TnR2d1dw7Bvcd1zbnhMWpwcN0rsT15n
o/ns8UIKFg+f/xOI4ruuD0igxDwjnrdUxPmCjvYRCEPjXwMiLHJSWZBQNS+c6trWrYOSmBgUuHMY
lkMAPRuvtFIvWbdmBndWDQr/+R3zJ7mbOsD5ucwDSTXFz1ThFaF0u/DbWPL5328pusi9Dc18nYX3
uL+LySDrVy1f5ifxTZxsZP6QMAgBHlXa0ZGiyvU6hYvEFskVAAABlEGb00nhDomUwIb//qeEAAX/
2U/BdJlVRgCrV4gebGTMJO7fkjbgvEKBpBwoyVT+HIce8OVhYc+v4skarCuYcDuBBhQSeufnXZQG
PBg37PhHCKMgxk7ryzhVs03Qqe3qvNwUcqGzN8EC5xAyK5QZET1YaC2FLWY4T8evOQkQhdDE8UiY
/dyqQLirR4c1Lh7GJToAT6wJhkjzqzXhCGLV7Z6S3CdPQSZHxG9KVaROOzSjsL2G4VZvaJDA3yIU
ltgVRrk1p3kanfurA1YyAI8isf+3wa3GNdGMuo0szDFftJeIelzZVkQR43gcwlrjTGwqUeRYc8LA
+WJjjFyRqdrPataEiTPPXyHRCVMgrKVQhgpKO8tJsD0O4TOaKUInvhiulRZky4K/ZoyA0QRYiIlV
BY6e7bICzKQwFq31J1ykopWbAfpzMDoBXvfwNfNJPy9e2fxf8ZGLUiHL0auJJmeR7huhrGWaNp2S
H8t2fTbor8pyVS/wfyPZAL/HD7f7j/1QQv/dRmxXm0tOT/0dFQAwAmZndAogAAABUEGb9UnhDyZT
BRE8N//+p4QABf/ZT8F0lw7RlyAK5Up3pK37cdHT+hCvpU4nX29HF/Q7llGSQ1e5x+sMjcok5aWW
Tk3sXkECMHzOKnMKqmQueAcSCt5g0HOGQCa1+1lQx3NS0IwMZG2/9nJmxSdt1wn2yWxHwLZjTIk6
s2xV5AzVJrp7X3lzBaylBBLEST9aSGCaoVMERNHA/6wIj63K3peDQGWd+PoUZbKpHU3PMYvptMOH
vo6o4TbYkqfJMYqYuSseBrVi//fR+yhrj4hwhhzm1O4ivezdPxKqUZPzBX/RE6Yb+zUfjqgg0sp6
w5Q8GL3oUoPT6L+umYhOcTF7bt9xLHOGNnuymWnD+idxvG06ENqS4UptA0bukrrQlqujctV1fgkn
Jd6aUYmMnRctgj3zQlbivXclK1DhxifC1Y8met3nNUXVVeeYW1WxrRm4uAAAAJoBnhRqQr8ABMlU
CdodJmOPuAqfbg7/r8rBQHaxkAEsNXFjCDJZTDOLz/6Cf/o+IIH0PokrTBfoIX7sI0ZW37QpBfDE
CGnk8XHbq2O3PGXv6110HGOKpvEjd8N/5nkZH+RLQgw/7xTxuTLPdC+BxV+h4Lm3vYGOakR9bhPi
thcP4D7/COAr3nfE+S1NzQ601TWc+ENh/vL18k2hAAABakGaFknhDyZTAhv//qeEAAX/2U/BdJcZ
CIMxZAERRbVwLllBQKGJFodAx8stOInw7fkqRdpj64Fpp5gJXBjtFjufR8T0i/9uHr0rvAnOsota
mY8Zyqs5x7+8QP+8MGAZHaIocFndwBa+tQmPGa4HUGwYpZTFOzzIS0dKTAdGqIBQGkmlbDtReoj9
DLHKjUSdSuN94Z2Bxe5JCRnpAZ0Fn5V9jNnDntL+cbG6DvcAt3TIVD9qko45T5qblpBiLjgfCSib
wrCGJuyaNSGikHVMoZDl1DRHhdoLcV6oe6mR2InH4gmVvLa+MJSrT1OyZydl8VEA0Vpr1pym2xnP
jTqgEKgygANFDihzaFwjZBolrpIv0GWwqcszQpCZZBaPAqvsAD9EZm/fxMvH36tFfF6NjT5PSh8t
USBT0NwOR19mHmfc7BO2C1eJZuRVgvaKHA6s17AjojwHZWY8Hvf5gji4shDv0GaqnF0AY7SgAAAB
kUGaN0nhDyZTAh3//qmWAAMF7aEYUWC0ALMEBmnqroOjCKKGkljUihOmiBFe7Ew7k+83OFZiqxFS
jOb5fQj9HDHlmPszBoRfQuOURppemN+JO9KMOCY1V5EJ0Gao2ajmEPQzOv3udH0crnWwhnnc06vE
wh0M4iEapTtSW3kx2vY0xJb1IVk/UaF+UcufnHKuoBjUx8jPMpjIi83WdEcE+yftN23rRPzyK6vD
AYV4+irKlTnO4JMgt72KuRpEo5OrpoEjvXlXiz3D/HXDMBvXaMEPvZb8TwfxTVS5ntwKHprRikUP
wMgkFO7o4RwKG0bhi55eMBFvFhus001pUWfv00/8K09M2EngXxBzt3QBqGmiai4xp/HC2dxmnzG2
bZ2u5VvVT1um+yfRGoYpREQjAdhfaQ8V0IPUxvS0wT5aTM5IpyKugKaFPFA9lTETO1lQZRtNk9cb
scNV4c9wdv3Zo2nhPd4hO/MEcoOJ8Ad1HhPVEP8CFE2sAlr+g0L1H1VybAZ0JkDgkR9TnbfAyuO3
45AFAAACYEGaWUnhDyZTBRE8O//+qZYAA0HtoZDSF5eKO+4IAvP7KG8V2qUyN5uFDFL/mGL0N925
jilfdkb9KzzeoAeV1esVyQ6pVMHBoSMbTQ3/BvnbtuXkxm1cGXThlbs4YyETWgvh0dYDTGYkWvaz
uOn8HNceY43NUX2w33nzZ+QvFTuv35tLT8ZfIJXUccx+35ycqv73TSrvWH7pI1pu3DDD+egABPP8
VkXcflOATXeaUZkUr4/bdd8CdMzRIa6M8Q5SX+Zxq1L/IF9+QrG/8vfygzQ20dZ/eGxLApyiV5gS
lAzd0jt/jfLu8AtGhwMGsmNnQBjyp0vl0Yo0bx+8wXvZ3MaMhAQaZypSAs6BT09K3YrRZxuzOyj7
rX6OdKrHhh+vNmzVNXN3kyv2XS8Qh7d6b+z+HveEPqeyGrd5L+xl78+gPN5AF7xB//afk1zkD9ko
2W7JKbymWqeXbLGTXuURrhHQVB+8C+w2evt1OOArSHEzBUjQdfbTiXTCPmKUZ7QXYBpA9UNsqsfX
nU9PnF603Kg2oUKGj5IQl+WyJMI3aoR5oy2xxbdJ1zGdKaLi4xJbqwveyfc8blBNwrvhfeXgf2/Y
PJgc2Vq4ua2MaQDNgbaVbYIz66eFZlAwPRlanz8leV1kIAd2CNGetrCf7NVpZ4dIHN6aAY15IMnY
sfN2J0i5Jh01lv9a3LAg82Pekdbt2ufTLriTEunQAQzZlztLaMNTT0aEV9QSLk7fz7tAmMGt2zdy
ovgNMerC2E7jv9M0/1/NdMkxA5f+O6pvVH1ZRQqTlu4OqXFoV7VqdEhG0RwJAAAA9wGeeGpCvwAF
Msyn5ja/BkNFjBAA/FQsaQ90JQgGkHSrNkhZWRKDsELEGuVSxLA4e/NUHH9GU+/bNPRfoVl65mya
WpLk9bWSCiFyyKRYJGbyvC7+eCQmuVUgLAgadlNEF4/7We0wjy85p/pW+tS4QvHZig7Zaf6Drqin
JN8AMEVg85ldw+W3FoUDL/Bj696nfP/szurhBrIcF7BwyFK/TajO3a8MRDWySjvHOpReMzknACl4
JybzBmv8MF2QTgo4AYPXBwl448052W56tbVno4O+WTBZjzTeXIMr3WAZSSQ4LDnoVSoW3iCqNbRO
BNfX08s0Cewc3cAAAAHFQZp7SeEPJlMFPDf//qeEAAM25Ut2IAQ3YP/4JWEXVKDzY0lT/braw+Fv
ebOklaFuMlRVBG4kt97f5y8JkkaiYNfaeVDaxS7Lau5JOGxHy6L0aMtU4lEoWxsyJTavK4l8sK6W
D3z/M7BGRmLqo/D3+1jdKSkjRhxOSTA3QsM19ffe/E2WHAL4jx7NFePfcxCvfqLKIGYXJG3h3ZFW
nYBUw8Biehrx0/U+FRNqAjf0brQc6kt+Cgj0m7aUF5t7Mchd/QBHWEppdBiHTG5xpolDgw5f043g
HLnzx6HHJfcfgICS3bGxoFR0+V9SoR1Z3rl5yHZk1mLXDBboyhm/j61EeDS8HVOo01rrrGrIWKUb
2MQepeVr2SiZSrpAnXkAPq+J0vIaR+3EOvInKmTSBuzPFY5bf5qPQO8aRpRZiIcVSkqaeWL0cttn
5B5R4kZVQ08DMCGJyiN0gL50TXr0mqqe0P1st56/Ic9gTXVu5t4bWTjMbO2wCuBeQWhBApgZj+nI
rbTLvh9VJ4+k7BGONjcn+AT2aKZ2t2fhVQsx1FGo/vKPuhgBrzSuWuOZpQr6PjlMX/Nm2PUuQwWE
GYFD2vyLIu2hoZ9JAAABBgGemmpCvwAFMsyis3bBZVPkt2gA+htohxX2Ld+q6zwZjenbIWjbdWfu
yI+N7fHH5gUSpHXbfIYPDjAqBjj8zieVSxIKuN20oYl0OCvaHhNLqdoBITs1aaEIb+IA8dmtgxb8
H83c8YRqVL2bQb/a6oINnuxiuLlFqwfsLmYSdW2quMZMgka9QwXPfLxbqtrq/sNoHhC2APZ6/6LP
01EreRqc2hcRHzvP3tFPXtC5texuV61rbvwGONheqm1RRqjffYV3ijmxYL+QsxB1jJ4yWm2P9goa
E7K7e+sxpGXd/QD7tEq1NP4rBe12JaBWUf/20wN89NTdmDkZMig2cktjHlPox4cOhoAAAAFAQZqd
SeEPJlMFPDf//qeEAAUj3hRnvcUyxoAcH2giYqndT500Az83ucPD+kzR7FNoSOBP2n4X3zIxr9Du
U//ZLJoTejplIbwyi3UcKKo8QrNQkM//Dt8BIOZDGKDb16+1pxwl+Lw56HrfWFY4q/hzqQHjdkXH
YixI/18UwO4yW8nI7ReeQS/PtjriJQeHeZmpOrKrXqYfu60kAPPDN9VRL5x4A3s2tf2I3Ms4Vvfi
UEmsT5RX+/gm4wf/xtPqaw/IC3DZvas86Ks4qf7eyr7uRfWh8sw+q6IZQYlisf+wm0RCJDNSjzyw
Q/a9zN4cFeNVhr4lPHelnZYoFfQ5r9P8UaqqVpe6wd2UU8NGeS2NBdT1CAdJZRNdC94E8Bh0ikXp
s5oUviOIdXm951hHUvH8lxj4Wfoiy337TVGxGZVPHR0AAADFAZ68akK/AAUyzKKjNLvUM5RTwATO
xc+T01Vin9F5EYi1p2x5gkCvYaGOZ5svDw/B65bP8nv6dDngr+ScOz+sD5ZUvYF9+Jk2NbC3EwwP
gD05AOYgKHsoEcCQw4WzPEaPL3OOi06kdlzD/uueOK2f99cyjRz5CX2SuVbE1FvG0HMpXj9XJkHe
0w0bFCbFbxbZEfhLeATDds0wYxD6AZc2f0B7GQRm7pqhZxA3QcWKrq9uXM14pxObN/t0CgN754q9
eP1s4aEAAAFGQZq+SeEPJlMCG//+p4QAAzZyw0VjQA4+xeGJvFFnZbPdkL/oQwCuiIi2UCEIpt45
XivyJCAPAQYvW2/pclIVPD1TusbAUI8xF2EGieoQP8yNGg7ck5WCPoZ2+nWpaZLlClO+t6Vjqj+5
pLbhMJu7h/uW1mO6Upq26LJJLfeHqO0lIEgml2Ec4j0JFR1thUj1VWYE0zCoU4Hr4K2iycXpyqmH
+fh8OC7l8SlkapYWc8X9YtLRzsKc0nPssw88kpVuTTgrmefJRJ79+Ei4M8gyj91LCNuQSTRS7llT
+so5Alk1/Cs2cgMH33+B7nlWGdkO48Vde4NSG6qrvYOzvF45qofLnTQFG+z+Ro0dqM+EimNnDJTH
2kXMquo3AMCgv8Ewq10hgD656ncjzAMJTxtAXAdut/51nbHCTH1QggqRvaDEJhUBc8gAAAGDQZrf
SeEPJlMCHf/+qZYAAZ6F3LgBH57rlxGuP07PwVKF/wHjV3/g4l8+zsvEeSUg7o+pqP9Aod6uuGKH
RbpHnZtyT4aP7r0ZxfP8uUkEgWydwGrnszl13BZ4vz8vxgtnHEDVVGNC5fNv6pU+4/evtVf2LSZR
uuhWlGcGW895sFs9i0GV4wBOXvzX1gg7spxZmuF8eIjy6rYQpFHVzz73GN27L33IO+PmWxTN6Bvw
kr3NpS6hYkBZ6zIpWXYK1QUmGaMItpNdzNjUect2QxnZWIRgh/FH1+ifQJ+ZUm4JtkuCZ1fCInfL
OXb/KlpL6ujpTmYMxZ55VMRj+aNUz773u5lgpKWU6svTdTtV840psLOqjqHb4alNQRwd8rRY6JNy
1yT0jrThF8S0RgtpAQ37qTc5khvUiF/D1uVXznV1K8sHtz8YUxiEX/BSw81BuqgYvIeZfDBU8F+O
jB2j1KOUgfrusOq6t6q+xtElcRyr3IkSa2AIx7uMtH0x6Lp2SNBBkkoEAAACUkGa4UnhDyZTBRE8
O//+qZYAAaiXlYAq6WldOuCS0ZaNi22Emf1skfyJxIXeCf73PdHMecXTkZoLczquq3hFIVax6fbq
51UjZCn3fkjGqVqKvrS749at6l9R0GTidKGlL4Nfz+4Sgn0dOjsGtUS57tZS2K/fpa6tjRtUZhT3
hdyW31YNoVZw5ArE/EcBSVwDsiPoO7D+MM0SRsVrA1TOrJO32w9fZxsddWFheVHMUPeqLj4PjG7c
XT1ugKVWJz2ey1M73ZOEuGOyT3OPQLsgrxh/sBp001wBFveri/TOPRgGMzH4I7ZcmM+r234gcQK8
3+7uAjbF/uFMbjKivaGc/5N19egJKCY6UiFBcXarve1ST1P0qttgxe0A6520j+pjnzb5JWl8yuXY
WsnddQJuTHNXopfcJa5nUwJrW/+damrbck1WbyOP1579sxOiGpaMDYfSfPTwoyGs5ncA81Au6J3q
2NZt4+lgqmZ0+Gn7S9mmoQNiBrDT8HXYy6QyUzU/AEyZgoAma7uIoCQhOJdylN5rXONcKlTQEtUM
F6GabMREgMq7QBz12C8dbStvS2wIxi3tcTvX2jf8dYpHXpcs9rxKLy3OvKGoipIkknZ2vNTjedWv
s0kgi+Lj/SGySL4heMu7gvrKAfIz0qzmGjY2KA+oEyK6xL2YdsFmgbcaMKgNZ7lldLrhefJXhaoD
tlcX8JG3vS1e+8YXIXhe5FhJyMYB46HKhInudvzrht6l7UymsFXRQINpDjTuXL0e3pGIdIk1Iv+R
3i0zUIJbSR22wgZmkQAAAWkBnwBqQr8ABTLMqO6k/J/jVSTbXrro2ADNzcrf53UZt8zZ6h0W4LD3
ZsU0+c+K9kH4aFnv5G915NHTRRiBdE4VksHWaaGNRF43eoqndT2TmteVIVMiecg1n3gl8JuO4NFE
J4Qr2Jshay3bx7goIWn77ybuAPIx4GASMpkOepxMh4ZkpG9kR9Nb8AHErrfrWXlw6qvmmrqWsVzN
lBYgy4m+6Kk/+uQj2HUdBAXWfjimTKywqDfaOhXKOBzqwR82zXP5/d8rxeNyzjq4hKpu10uQq9C8
7HaK01uhvIdlqD8abJl9v00RdtAcnNe6C4AQYxneD3NjdGWWbMoNB+VUr0VTu94yi+sDLkoMzrfM
aNlZR4Cb4bqJqiYTPWUgTVqaU+T5lPA9dUhVLtnHKhg/FP3TRH75o9Sh7bN0lQ6uDuB9t6XmEsq/
ENn7fLjujjDn4+zn03vSxPQFSHuYjqh2VVXEKmojbkZi5g44AAACXUGbBEnhDyZTAhv//qeEAANL
H74AV1DElrnVQ1A+ZY2HK0D91dl5XmJaRj4pIwgmuGvSDxDojah+bNO20oua9ELkRL1Y/ysX6BQa
DzQjEqX5LHtWiaOwG5IaJTwzr66s9XTkMil7Vj0zDfM3ZEW+SWNVO8Q02Cv0sW2QElv19R+J/Vx8
+TWBlnDPfFp27LVbCYvR++Slzq6ot6rXqCcDprF6aH4ebrL8oGmQXTy/1bwi3QipDtZqqxqwoiqW
6XL/vZ3IVNSDQSAxc3kEU2Bv7MmBJyasdLVgJfkhIMvAojAyBDkM417jUaeidckRlQY83/4f0w4V
ivrQ/kjAbRi/0Nr2hG+2tLuWnd2SIokXHAfZjn0Sybsr6cipqvZMTHIrZfEQZXz5AxVrnbfeEJLb
4+RpkgB8f+Of83vV09LLxMV6qH2PxSvKQ6h0L6+2PDP6ndWAOO9806c67OhF5Wl+MJcs176Hof0+
rZE+7dsDXOv60vQE7Zl+8bxG+/CzuEKU+5SgFhYRuezSd5lAUe8O2UteHZ0eQpUQhyezH748P2J+
K2/KFzV/4zxe5PvhuRCC/6bfYISv50kvuTAlYGUbjP7fSSzMYpaQ9u5MbEQO4aU31C7M7BsfV879
3vlMYn2+RcMaM8Jyj+e2pYv6JcXcnGPgH+BUeO8FgPblvCsKZpXpQtfuJd6JhoveU0p3cQ+6xGov
DCIG/A8mPw1EHYgK3Srjhd3x8UVZeppeW9fPrV4+77be7igsI7lVfeLNk3gW3lJWWW1oEaq+8rx4
qWrScZbYcSA9VllSb2C5cAZVAAABBEGfIkURPC//AAPC15Xk73bwIytItmmkihLC3HfFr700oH+i
1WCZ+58ACH5Vqh0Oe967BXdZevmQSeFLGPV6Zi3VlaKXGVhF7ebg0We7QlSuVQd44AvVK9YO8NFn
mHRbQjeKqMR3pz3iGVbJFrdw5fYUVrkKXhS0NzZHTV/UEgfdoO5oy7fx1r/d+b28yKCHOHxfmZjP
EznNCSaSfxVJIkLYVvKHwNYBI3UUknQhh2PMkwZVgRi/H0jrs8EHBwqJfHBeYoz5XuknrYXBqa+e
IBtO3GRIf4BFhGiCJP2tyCw3UbMV9R6BuNWyTL5YcLasO+1+1voUnVdg15YxGmV+IXRvnq+AAAAB
ZQGfQ2pCvwAFQr3MqVeT1u01fmkAAEER7Fq1nJXBL1pvSWfZdxT+mrSJvB+hLxYKZCWYcZLT/YCz
OCrTRXqWScbvyxAFkyl9hl4TmYY9nUPY6SlICtOyPdAj+r0GNyi92/eXHvJpCmyjh2d4GTgfJ7jH
tKInEtjyHyuCzJh1igT0fR2V8eR2wZcdWTmEvt/qiaGfIt3P9mVvCcIUVINFzPDCQeB8sdxTU7iW
5yJkok04Slb8ceIxPvaH22xO0yWoyTbwgRzxrNCxDRKEIoQGT6nUvKOL2gBvWVb2rA8pZDxuUZms
9URlY791qPtGvVPc9W6sfQbgS2XkuVVXrxsaG/tycN6EiqXFNTdVnGSV+8ivuQps8L9KJuAU9pdh
wxOHtRJWpnJnjz+7DpxkWyYBhAdsNDNwIsvn0to/jq9OD/3TmvToiMAvYc0O0GbWSD1ttn9xH0gu
eTsKGbKs8XyCqclMPYczgQAAAk5Bm0VJqEFomUwId//+qZYAAy3toRIb9NunQQA3+0pl5CISTjV7
BWI+1rvig9G6MsPvM42lCa1ym+0nnxOm0tL5hMNG8sEhY6pivFadWdF/iSwgxP6xBFMMq1cbCSFC
k7T0jlKDLtz/yciX+6XCgse9C+QVOdwrLZrfiOW7lLj2MegA4jwsbLIQcKUyk1iFpiRFqjzFoXI+
wGAopoaGpab5Q1ifJCIHwzQ6td2ZrELKuBo3PDgcCLD1xu0VMtBQBo8IA8Q6cwjJitLxa9RP6YCp
n2YazlhRi4CPv0tjjIT+cq01IH6OXEN/PRUTm7NFm9fDrjsuOMFlkF6XtdPbkYT/bDdDUvmdyzGk
vCZ6xsYZZzG7aHDGIEt+vY2kxS3KgDPBEO7yzf3Vm2FQhvjiueedwJDrSo8CRDFsT7Yav444+0B/
yyvuj2h8MYHNe0VHDiTNDLN7RF8lzr2EIpiwD7osGQRk8RfVNXfllAI4UN6nIznBvhMrwumr1IbU
G8LrbaZLjxHx877MIJEpYHJirCsls751mcJI0x+cyKEy5ezyPPoKOPihHMOqLApT+9jOvqF0pHci
jeoMDWubxA9bvNLSE5bCzytS6CjJCeCCmRBeFEGlhQLwcdUv1zA7tav2IEdUmOE/u5a/DM63EoYM
A299cC3WJMWBZJGsan53Ay1eJwN1rJBYCb9qn1qZ/3Gk6FScJF2rB1rjdYdC2rXPnDjm3FJNuSd8
dynAW6I9/7lMmonooGF6uwCM4GYrNxINtIHJ47yRwCoaOg8U+yQqewAAAZJBm2hJ4QpSZTAhv/6n
hAAGbwM6N2ya2oBJAArkPHSPU83fXvjGx+nDcuAt/TStBy4EcbW0QdW3n9riEM1NzExQXFY68TXC
Fck4yg/X1KiHNCJBd2vnb7cJnCoKo1cPlupTW/HfWEI73uavxkJZKF460/gwtr6hhXmThBnjDudu
jYEAfXdQvUyqqNI5rW1Om615KAvmko3yqqlI62cJSjqnr2Z1DxAs4QVO+mhD3NqJJb7yb43ucdpO
MXNZOuntNT+6sLaONrh/HRLeUDFWLtuYOQ2XMlt/5ibHLVXfLm2vtA8b2Ct7ZwURd2I1DMsCgZLc
Q3yogKsZBX17iapkXxEdG7lPIOHtHYnODLLzFX+xIFyUjt03Ug8fFxTyKW/AfE0u2JjjBotTg9VK
JwNTHON1jshvzFT1wxxuwRG6+VxFbBW8RSvkEyKrjT/Wj3xiqaow/U5dPsfHorRm1dQh+GE6F5Vq
a9rJwf3H1peaFERSy1OWuuOq4nY72pJ6k8IcHXJAVqWsiBjgw0MdHDakiEa4OzEAAADkQZ+GRTRM
L/8AA8zWIIishfKEOBXRTjvhy69ACQgfug5O0D/6mh55+OMYuReipF4NuecJiwiYNCwIfn2JYwQN
aD5Az2Q9QnnSf4E51iYv2lHSmFT/P7Nm3AmkuurTG452DWMq29phTnrQBjVU9e7WsTjTe982ZFjq
99vubToPTuFvdBNjpCbZV6FL0/FOYV5V9xNjhnGh/QiVAhrsFZw3X0Calr+1xo/DKE16A5qmJOkw
+IXTSQvxiSkCnnP+xdibFybbOYbKqxb2LOGHc61lBp51Yr3U8c7rCFk8DvN7GbbJmqphAAAAbwGf
p2pCvwAFQsDuj33txbNU7UOVJAB41G8uJb3UFWjLctmi/o1WLCTODDey78lzwPAbn7tzTHKzFCpX
MInNgwEf467zMCy+jMvz9taKZu9bysCaAt1Dlq8XJVQiKbKwy3J41HrPmsk0P1/fRXoFwAAAAU1B
m6lJqEFomUwIb//+p4QAA0pv9ntEAIs1NEyfWtGMFQf/cluIBUJFw97DATgch9/8EkC2rdwHROK8
5+Lt4OD/oAUXwl+g9/Q02FU20a2QL1RiTOHruaQATodGMCP1WZp0vmFu5inE6KRMiAN7Az78knaQ
4Mn+ELXj3ErYPRfzYzraBjXeUWvXdg3nFwgmYVNjrNBt0FNt/T94fis6Y0jXMzoroSRx2FUQ2ReD
e/yAgkvk38joDeM9qEpnnhcCK+dU7wsd6mzx84Elx93q9WLYzfEFJx+kGlyqEQvaTt6JQgVoTcSD
PaxGdTx1USBo6euFGIPCwm3jDBLzcQZH+9zsBFDJTQ/sTS1rst0ELEE88Ufjg+1+dCT7CNHUFatd
JjQZmoQsBit0InllxFOhslqRwhM1XdhhhSSr8YGvrxB5vmCKYk3zoY8D00qf2C4AAAHxQZvLSeEK
UmUwURLDf/6nhAAGJ9lRgfy54gB/sYrBwqu2xvDvvy1T2rcpciy7kERsJru9UkhE9byMORpc1WS8
l7ah5Yfj2+Zj8QVNZAYWUkHZyV1ZV9eKvWmhVsZ+QQKmaIw91IuttpfwC3zAV4UuT/z8YVwxWJUW
T53/9xfr0KXr9o8tbHd/P6nmhM6Ln3BMm01gwCnYuqlMRYbkug/b4PMeZGJekrd3WAz9KrOA28ym
Bt+vdyulBprErmchVYvFwVlY67dqgFvoDYA1EAIvP62cFsgTiRvjvRRYvr85yi9plDpp0BGf4MJ9
HfNu3bUwKqzSaoiN888JLlvbYhn5rbueViARh1ikOJ9nvtZGW6p85QUxBBhqg2WW97eQP5ueAP/2
7IpOKz04CFauVDHPBOGd1RiMkjb2J02SpKUlUmtgiOUkwTTVLYG1kamvTe52o9slkOT+NH5Tsxh0
WXEk0OZObBSe+Q0Hj7LjqpBKUT/Pxg5knvIIW8h7cgIJXE01uZupUKBHEvYA/Ewid6XLGOcVTJFa
/zqnbfaIsqKeq1idaiHHBGwt+1RO9XAc0IRG9rbofMksg5oloNPTwrIxl946/acPi5/M2Dg61VfS
P0U5BN/hXECqiOXDBpbmVKbpNfxVaJ5IxhVJNx68i5R/R9UAAAFIAZ/qakK/AAUyzLY6aO5CEpGe
AAH4FfX1uKNs9lRfAvKL+KqDFBCXsApGFhGXY49kgELk3kft+NXSIdCgV0uo84XfiTBmU3cbPQlr
SWp8ydHMUM7TXGgHVMoJIaHNcn8bjhBu746faYfaOtbs5CgvwjTVOYj5L1JpKseNzy0DQpKUM1uo
Tq1p27PbTocnmBq3phRIDSRCGZz0ofiAHf4EFMMdAmJbs3ki35gbCiJVZ7T8JIo0XyEFxME5EfKL
KiMAKZVcYoBZOHk0lpepFXFzKVl4yxnSEyiESSjDmaoU3/CHRQG/n/coljCw8U+fGDy9xKth8s2y
9QAjz54/pGN6O9MwzF4cUlMWFGg2lzGWKmE12zTBseB7d4xHx+yJARRkEGkBGM839r7ZE5azZ9uT
paq6Ynu9sA2K+BHql9shTxxBX5cfIA9w3AAAAjxBm+xJ4Q6JlMCG//6nhAAGJ9lSYbuvClp/+4Av
Qy7q/I5JEud9BgAWBfDYOwTDQEkAhPykw/Ac/T7wxxnengCBeDUANpDFvTJ/u1uMMGY8ZTSw0D6w
VmJY9nk5chpsdisfm9/c8/cxTabpg11aaD4E1gM5U90oFe8bsRBF2M0rZbSlQI45c2zWEO3WDpTr
b/3q2soqqJbEt3HFG0wcuKLeeyRFz3rs09BtFlZOWkpuNxlwnto/hi9kZ3WdXw//Uy3GldZGjd3/
RavXfH2lTZ17D0AMGMos4zchlv2hGqbN10rFi+T5oQb+aLuhvexL79CiRsOhFfmgvJsQe349j+nW
2dyZHILswrOPqcBbxak9gx3NycE5oenNu0f3aAH7UoGq5Zno6fGvyeeB+d5A5TMa2yO1p9/Aj3Pc
geSOALTN1uxbdrk54y/bOrPSI1VDp2PXKJ5ILV2nPMatALWzBLRSFJZo+xsKkcx8AiA+zZM2YrRK
1MuQb8uUnZvetZVJSVc3ikBShlsUfx+H2FpmrZUywhOKiVmedtj+JeuzvtCfFtMbNh02IOyKQwyG
COcQ28S4sJQR/wZzhipW2syoKzAThHYAOkTDWY/CplpdVPN2HJLgZMd/bLsTO8ENydA7KG67E4wc
F/tN3tTWdwr94Kyg007iWDCDRzAHXeBa54JYhNr5iUPOFJOq9SsyNfmHujLcXp9EEZZ68Oi+Mskv
9LHhgJ+aI972BqfOyukbAPR9fO1Ejwer+aR9tVOh4QAAAhlBmg1J4Q8mUwId//6plgADQe2h3buk
LdYVLVADmhd2+Mnf912ty7JIyw7pywTqJHBjfJOtapceOpsWcJq6S2L9sQCUf5NPqnLrtBQ0dUps
wBT11oks2haRtasfagm/jrFtrr7N76QtMTyyQe7yxG7orXlCK59Db96WkWp5HYDRNvweW/4auJmP
vNtwz6XFf+RFOSSpADuQFRdFPZJ6frSUasfNMetJS4XI+IP1+FPWOwf2HxWi8YngzpqL31Io5EAp
cEdpQT4Yi1Y3VPn6WOVpH0B8khXkddiQAuBtb6Y+1hiLXDqYRYbJ8bxwNQmkVUe+lo0uvp0wmZP4
dpcCgBVGP/v+iNHCg3RDeMZIu/bYCw+FDLLEWu7bkTJ4j5Iw1PrS7qcCNdHVzH2YhO4n9/basWhz
5O8UUAgcQ6b3omIjI/esN1WCMwdD2+n05iD81HDRxfW4C62u9zBpndmKRaDoQe//iDywgLhqe/ue
TftZJzmJ60d+/LZy1I7+vMx2TSWi0d3fgZ5bh0yoQY3lWW5yNe3CXveuVWtsIKvzmx8EIIkgoQz/
VaTeYes4ohsiwD6lnjrjqnlSdvjzMJcyO2OzJtw2izvn5xQafECQVWbGpv9crZsiemCp/X0kZCdy
1JcEWny9DxfABYJcZg2F+JeOazEzluc5AiIp5cV+bX0yDYWvPwAKh+tmcTcjeGim6i+mYJ+3/jm1
BH0AAAIMQZovSeEPJlMFETw3//6nhAAF/9lRgvFGIRekAP1X3Fy//7/gPkB3QatIkys8AVXcwTlY
Dn8BzUp0XF47W8lh7biS0UPQg0hW5N+7V1OCz8uFl7g7BUxnMIplEVtyhVSum2rL3pjqz3aGgV/E
B7z1nqIBGr1zzsj6dL5n1P4O35gtimnaJ9z/0bJu/zXV562M5XcDdpurJQNOK/nYoKfuysxFPAFO
c2L0OcmfnbX01+FK3figBvx6QDdzZcM5NJAAqRXdZmvK7i3wJmzq1M+feC0iZOFmLdo3NoCFoPd+
IvhiEb0BkLXPfcnTWeuupoDCcEtY34g+K+7birYmIBmmYBY8oXPwpNfNwO6C7LJgFnA1ELAyz+Av
2bKPHZsIE7VmwpWMv+ZAEyeEQAIxseRS+7CJVT2mOVH6ql5LAmXQPnCFeFgp7JFLgjqZvAhNeuBb
g29ywbsvKuqcG2FRZ63Um6Z77Y68tKiTxzVhLTjAAnzAMl0MjYDlBfdEZ96P2mVjxemjQdOTNjQG
BSUJHuQDT9MbKtgeALFhUhHMqOFo8zl1aBGSLY4CD/vouvSGmQKWbSVp5JPkgNm6opKysWCa54dP
gwaZSSbar5apP2XDEIpdcyfwOcLfm96WNPWWh43CUN/9UD3dRfxO2de7fjKyxVgJTFqFLtCtcClc
NxxpBY2R8ym1/zYCmVu8m4EAAAEKAZ5OakK/AAUyzLI4HhVC6EICw4AP7pN24ehg7LLryGj9y2Lj
5FFWhOTHuVU7VzCX9NgVps1hRO3uy1zWNzq8LoRbmimxUfLXoGUpO7sLGX6KwDVXO2vXOAGKR1ur
wYxE02dcyVo1GhpaRw8OyqQgOPgLdpwAR/ecdGVyNSk8AJ8Ir0SfMXdsO9qoPWfwojcubnvMZD71
o81kuBePMfc+eQ1kHbwbFHjibbTFE6XsN9dS3k8wA9ewo2ciRPsBF/oPRyc7MhgqyjTqEPL6wVjs
/qnV/yx8IZtQf0/m+oTYHNvHoPAd5s8ypPxHTrMithaDFaVTfNPud7QKbyrLpIgUePTUhw7194Jm
wZ8AAAFQQZpRSeEPJlMFPDf//qeEAAZ32VJIu7kHWXbsmAG6m+eUW40h+IMtvxZNDjJY2Gov8Da5
Kzg1IBuaPcWZclxdHyGmmIT4obhoulJTO1zWzFOESoqNOBIS+DW1jvSxJNU+HMiJs/dQh9QbZVgw
l2ebV05QDPYAhYI8wNYQ3/v5JQNMbquxtHl1CTHRIvMexVQ5ZAcxvz3w6RUtJvAaiz9AE/QNENXs
INdFZZsl3Ggfk12QtwZctll3JlUfWXYZM3ZJs7gNHGU3Psbfh5CIk10kfO8XgwLMzRpRGw0ir4Ez
ewBoFsLaMRgP9HImXC48WfsJb/gsuqcBSbyEs5RhI2nb0Oudar66VdxE8nA/pMx2ZmQ3HOrPu08m
tkIPCosayDKDdbnJ4B9D4qAkg8pyBifmtfk75k+hrTuTEJdtiibi39VLbQS1WWQRmEE0AF8K0lZg
AAAA1gGecGpCvwAFMsyg7vC3WrTjVmqwCX5HhACY5WyK+St7qclGmlRu3Kkzxgqi6vVUlGKGuSOu
6Nmz85FqwvNNHMk5OLWR1sFCRFyBHKNQPQ3s55vdNaJnrhDgusqxzA3qBd6eIXoTKxw4bQxvdQh9
OPWa9L/2A7A1GyE1RaKRHKs8peOGslF9zALsvSx0K86uojY0Dy9faMhxDdX6w2eEqa/FhMkebCoW
ARgN4joc2io6CzWZRcAEL+biGtl50YUDbK6do3PHoAq73YKLvsk0XLf6f8e7GOYAAAEjQZpySeEP
JlMCG//+p4QABP/eFHAhFtSCvFgBxH2vPn7SIVUUMEN+rvD8PLRiwxmBUU+cCZf5fCyXEJvyFSt/
4/Q+BUtwCqvGA4c1jntJz3tdLODsvtIQQh4XsZrmv9msc3RM4sm18sofjhtLTuM3LeDoU9LJHP8q
KNxotrCt1+R0eY2xCViNDyuTChyitNAw6cX/3I1pN9LTji15f/XaUAcignUp4dwVYYtm7nafeNF4
i4Fwidgu0PQzSXsHip8QLXp591AotAwiijzY+rhXsVXdITifkTxTutaT0AcjgG3e0jPVqnSYZiip
/L9fVJnHdMaeEoXOjKBPBVG58tRpXCCiZ/+vc6igYkASI8fvFe1YUOrRrXEqRgOoFbciHI7Nkr3N
AAABw0Gak0nhDyZTAh3//qmWAANB7aD7IVONyQAlb6Z/iVUrl43NtEeRDO2GSYfCdNNksGK06NV8
w37QRoJN3xK3KXX/NqT7RWG21YOrpRU87WGj22qM+QzpP4i/Kb90faV/4DwfYNKRM2/SDn6jb139
ee0wa7q4DqHZg/CNbvS2Jf6T0wK5XElwZpdofGFCEWP6IevjVQ5Izolyxz11RkOXskk2FHbeIEx/
U6ShH2T38iriprJ+stFgIt/mTvIoUJZ3eOXgwn3iI22h8uAXsP/F9klh3+y+OFKMgMJmokt12Djf
csmNDz1bVCcqda1PwB3dCzKaTYyIltNtd+/VjxzRosvMb0Hp130X+j2oN35QgaBlBiKKa0VUiUJG
80Lu1qmMzpUA6/pFXbJr2zDfuopNVP1NE1iMbZuhk+FuxJQ3SWo3uJPl/UOBiQJINsEZLgOxn94S
Ee0pRENo3yetFQeeVXJ+LcFFjIK3DclFqcIhnCrMqum6UXdELctScJusiZhOgkS+DMETgq8uC3pW
exwTbeqjl85lwd/IsA5vH9R5U8Rs+BaiWTZEGDsxpGQlsf5CukAYh742/Es/uGaA+1oNUSXeTAgA
AAJnQZq1SeEPJlMFETw7//6plgACQ/I7dV92IWACupaLz/OnomVqzNe+5YU4M70tiES3D9GBQB0I
xnnJ7atUv6HxkeySwEGfTRBqrNc401LjOZkqdmJSG7QXzUo5KZBLaCLnKC0lHiOQT1jopPYlE2x7
gprMAXroob9W5mLypbj/W+T3Y06qvabvD9oi5uQ7tIxCSJK/vSCR50jLAMAN9OuAe4T4NpLHuQ3g
FQbIWltksk/JZQZSjpWc5t/OL2lES13WGo2/IM9HvYDEFSKGtusESNtnOTRnysuozYwjl2Iefu9B
SieUL7ci+UQYteDCsfceXTnAO2QZhtWieTz/pct9eqk6tQf95bhqUoOGUUiPYpb+t/ASFnnFY34D
W5bT0cYAcIdUs+MZjBV4tPA33JohPvu5iD/jUX8O2Ju0zcvw1de3rH2OBqc+HBd014LOg13qZ/mv
4kngkyHf8x30E4spZSSsXM7lxbIiOAvITtyMogFB9MTCBV8rh88GOLcVphKuysgnZxJJdg/LonDX
4LRAbwca5qBhaXM+j4vRB5ojEbn804N31IGVFYK1gccmcBkV3T/tNsk+IyTm9Lw6PDHZctpXjbbe
WqSFuPsmotrYsXpOeNIIiB5HiiQvcKM3CJmZxoR6trEfLRa18H1NjjyZKd+dQEmkmM++mzFtSbIU
dDTJLYkrG0bne3u+HkC/3ZM1qqqjj8fbgynBwx+zHgoupQLb2rFwCEkpMgmT84c+yDUc4GdVlyrt
cIKs4l9kYMaG4duQAEm6to8Vs4n1XLAaqDNsLq8ynfSkjbu9gIDqnxpr6Sxhhv8BAAABawGe1GpC
vwAEd2I6PsaPK2TaaiaSQAbUe/70weQdlSqk5DoX6PS22VBzCdiy1Ybf6rdNfOW2uvw0HxY9kMCd
ZUcjB4A23slfLHcejhINcnEcfVmoDR6AIHVGMGaMXDxegf/Cs/NLlVx3A23JwSZLV3souydp0NBP
wARZ1Jeb0LUviJ8rKJkt7CrAVTG7aXbSCZYqqdpca7RYEwRaHPSe7TawdMjPi7nMBXiYbiiS7uTv
RQ6XEdC3S+1vkJAS/ZEgtVzG5m0fruEcwankUpakbzWzUucPFVmn0L9e/eKEpbxO/psyHx84ZFMh
BZ+L18XWHcGs8oBax03ik3U1nbpOK+TJyMnAYnUyGCTQECUDOFDL6pV3QrXQplnGZTTZXFwdufFy
J9zOVw9bTCwYEyrMSnbpEkqatG3EhkFt54LKP7ZpQmxgPqFpzhZgCXTB+nRzJjS6TicMxhbSnlnX
5gA5gdYSo7Q1aTc64Y8wXwAAApBBmthJ4Q8mUwId//6plgABqJQtACLAvEMvrTyb1wzAtymHMbsn
5L54FgGZXzUPYyra9/PKCCcoBuwbbD/nvCfrwPtI3j54DQBsG9p/IgTROfcKx5WdeUEWjeupNfs4
J0XCe/z1OMjCL3yqbo9RDHqJAouqs0K5yHS/iAacIw51irh/EY8/LD5o+r2a5rS2B/QJJXPpWpZ/
gtKSDmRD46S+5Jvd93AzjrQFJvJb48BvFZsKkM7n/wX2VlEDnCULw7/pOHzM+ODAvEmMdt1+OEmS
21QMnXu/QDHbN5sXBoku8Cg4UMZGN5R5uf/Wy3VC0JIZW3eewhCFsxPfhk08GGu4RF5aMgz0RoFT
n9pXLmpHtvbV6B2qvbARdfVgNzpAX6HW/TywfK2JrqupUPHtfzCR3OXGgMEE0l6NXH6DmlHs4Mu2
uUxt1mYkzwdEUFVAxEvm5yQGtnrT/8ZzjYNym7N1+RVFt51EnWyVt11piHtPtpcrNo+USwv1O4a6
cS7t4hZiXtyhg8nqIKX6k/eKliqN+iZyb6M4K0wqGKSsV8j5qLl9fYAVB0m51TNxe3pLVjsEdZ/H
kBxcbCYaSW4ZdTCr5y6jbGfCDsj9z1BTvzz78w99jDV6c6VUKJ2lNPA9WYGYeVw3wNCF8zzx0BB1
B4a3bV2X2eIHp2R7TkpqMwMUW1bAiSK5N3mJ60hpw+9V4kpjOJFlkPWEgqh3uyYlw8mNvHnZduHy
TsJyCuP0jzEx49sVdjN5quXUY6k1cIt69bW+IG9i8yk38u93G5ySjIKGMbLwOgY4yMKFfyMBQzEW
eJUUO5wlAqe7mVtx72tFEEP2UnoDQyv9Po9vi3lXyUPBk7q0wefSa2/3HHWh8rh9IAAAATxBnvZF
ETwv/wADTRHGd/o8T/07gA5fJs4IfeaiFm2jagR+R5VeUg9s3QFox9Oyy4i0b/unprZsrWFlnZtk
oNq2/PW4ZNFpFVfRJtYPtZavDEBk6PoI0VFh5W+F6xrjHo8wOb0Ik4uQmsNCF4ZaEmYvlAvJL7Ij
Q8IPY3iyJ6Q6hZqjFQ6Fiu9RwFaNl2P/fV/sXRk4NW+fop+f4HBSl/pWMG6VL8niy+tLakmBFSuC
SsdGlr3CCAy33mJdfhxS6BkfKgWXSoYcg6C+cdkjK4vwJfp1Ho6c8E0BO0qVHlH9dfnX8ig4osON
7J+piVyAv9YD9l098z2QfkJCe94BjSRoRM8yWrLRzKf0O+smDPJ/s559fFzfy3kZJSOTwIDRNAgP
fH/bGLmH/3eoslWxbgxPaYIPNYiN9uo21l8JAAABLwGfF2pCvwAE+sKY/sJ04xl75q1jUAE41peW
DFsaiguncH/CgPt/0u+XPV3M9aP+TltZO6Pz+oXV9hm0f+VY2KkeWqVWoqqxoaDhfn+Gkco5/Rlf
BVukVkF4qGDTD07D/2PdOuNHjNNeg/JLW7cgT34w/IwATHP71weWShHm9KY8PRxXR/Otak40hItj
CJ1Oher7wWKbQqijvg/IrnEFOJObrNX/uLc6HTROG8MU9pBLHQgAAr+BxI6DhaqaJCmAK/UayzWy
3SkAoNubL4xlOhOXRET1qqv/mnLR5BXX3fTNTYNty0HsUkQQit3q8zHQ2/CTLbBhzSW6kmMqC9JI
BymaNI1qYauENhuHmvP1S+Z8X/5tk9qYPYl61XXtPmbbEnlBslvP5XPJradknKYDhwAAAjlBmxlJ
qEFomUwId//+qZYAAxntoRTxtYAItCP4ZiSMcV9sKHg6X/RvIJSs7uD9iSuJ4rTo4Ws05v7PWvt4
qrOQfOSmUp7gBMWYQ09r4D3qZq48AqQ4r00SL9sfNqgEOaGyySdQ+iaUExKiVC2pLsdBV8EVjp91
MXjsL0AG9T+uM/9A02eF7KQD6aOUeQVgZKXtoHGmq2M3lTcV2BNGiH2G/p6x6asKVjnhKyWVY/wQ
M8LfC80Leqwm/n6I+0pFKetf+Uqb0TzgyDssc5TQ9Okau2QITGSlYPxfYOcKWr0nSqqoGRAonJVi
jpR2h/lWd15lpLegHb/F2kbmn8v7oL/PRhqwMUSOd9/3v/wR/zf73eEimOq70GXzmKqX03QMoMXH
LodOZQypbH2QZSMR/HncjUIRSJ8GDUwZ+95sCLCGVpqWwpniP7c25k0NLj0H/ME+FQfxH04HgbrW
z8MzLte4cSjD64ExsMUg8OrCHDbsEqZ8Y07uLfbuqR8m92Rn2t88Cof6sYjhVM/BQ+EFuyvbfbCz
nmSKVQ/MvtngnsGVonDobnWaCxc618/+l6sK4oJMGsJI63dzMG8QWoqm3Bq7eivB49WlRe3OzJHT
5c0ezpQD7/tHPqWJy/DoMGF5rREnSXXRvtu8mThji/Y5iZif/AZFR6fzkeDz3zDbnJxsfI8vpBMv
R3W9hIL8GaQKZ5iJTNbz6My4ZmZreIUvbj0V/SszGqP3V4nzF9zC6zWfUwVcJQgpJjDWOQAAAb1B
mz1J4QpSZTAh3/6plgACze4lVTClYAWqgAVZnfZuX9wzTEeMZG2GD645JXrJwrA4xcyoe+PwrzFS
LSza3svcNJTWH4Q5Ksmc7gCDiw8XPnh3WjDdxEO5sVZQlIvy6I/NsAjGH5SrKTy+9EwLJC29zVd8
wOriJ5+sXd+zjr0M+zRA+SfHPAZCW1cDczrUEKPQWmM9uk4vA5220IyC+ilccplXvxXzsMv/ROIu
u/yT4o3WtxkQBoPAzljqDlORlaj4W/RnxJcQgPxQ8Fz1KJ0Ua6/m9w8vbITDxT17xBzpMX1x5b2N
rvU6cRH8i5dwAoet7+EMo9zRSiBsmSY6JzjVkXfx6VWra6DE16ayGteIhjMvIRo4n3+LPktyW1V/
KsuwH/Y4fzr6IzxcvpD/ZGyh1tH65TnRmWyeXYfq2hu1jpfq5jlcdRWrMvxkBdZAiceIPfo2DSZT
r43BlRYfchnJ7c7/0NjtJ7dO/mvjkEC4tZpd/jdcrf8/tg19UPjNAPVjDws6iTHBNhRkj1ZmzHS7
p1py14PyJS3K4ur5Dl2Mf9crMrD0dk145+2kNunXqWK3Rr7QflNyYsMifSNnAAAAx0GfW0U0TDP/
AALWy+F4x1cT6vJvDYlRJ2tpq1QqGfqwAnLmluEMOaEivH64E2knbR4gWzTCAVr9f+98of1+sd7q
FKXcPN0C0b2V4ffhCQ+mv0BcXLHCPDK37QGj5OQIXnsjUc0iKIq7VyVIQXdGth887t5jewvwU8Cv
uxbqYRPWdlRaOB+TKgdyZEoHPIpxzfIZHFcyHxHpMi/hdJxh4/4FEFH62HDFHlQPIMeEl32MG5s1
VSBr/E631ZSUgrHhjZq+XMHq0YAAAACmAZ96dEK/AAS106mMIEWL1a0GsjkepCVBCAD7lOtfRJ2I
jL1iU07XQvJyqCCTfUdGvp2HmTSRnczc15kn1mRtErhEYxn6/kaN7FL8V8SuCNOnaPGBFGyCac39
hHC5K2utROk9tStDz23jN8o9HzTai5TRlhW/0DbtbHtM4obwzjeftEF9d8kd+vfv13xv2wKJwB5v
B5q92HHSnQbG+HTMJjSWW7jx4QAAANIBn3xqQr8ABLdiOjYClCvEAHxkpuEacU2xjn3hmri+rqRo
4Djm0qMWFPCzVJv7WO1ufU8nYErdIFctmrqErxoljExIiWOn2kVX+BeH3/UoFoAsx+Dv0dP1+QL+
0qZGVZxpv3rgQagDHZJ/F+O/GY6LndEmfRFLCoTOtqAYA9dWjZPShDTLcZF3C+QqzPW0w+jcwwYD
AgC593QGre+UZDTnGWMSOPpFcCV5WBX3+zgEmNjkAvcD+IbyLYZ6HLqUWInsU2DCoLcrRMXS1rhV
hmHskCMAAAJcQZt/SahBaJlMFPDv/qmWAALx8A48nMPhADhDvMg6YWunC+Wz/+/0xxC67pWZBFMt
tX2/wmAO0xDlZSvdnZS7DJCBlr2RLShmCNJ0UxKNgS3IyPCXdds0OUi0emXWCtwDvU+zLEwd1MRz
bx9RjqhR0qfjRGX2ObhaaRw/kLBrcyViTCDWK6HXcUG6FEz9MAZ+UsZuw7TCYKH348EbVTDQ1h3f
ik+sjdcnBNE95wAKA+WkwBIYgtsIWInpSB5AxvwOWQWhCM7r37045jig3RJ0MHwkM38H9Ejd3NLO
RKFQPzx8ijJyIZlU00JTaOfQlRXywu4fsGBjYwKED14Ot7tTgBhmwpPmG+ECRkS9N0BJC+UTDmDA
4SYr3LkjZMhrO8LnyPeAR29j5TLSXUIH43HgowxAA45/PhSHfqjw1sCS8V99QsuccbVm0XuCqWdw
W5XsjbQn4AxRnKdszFO5JMnmeh/TN03XFDeemeNu9hOy1hAh9MjXksbrJWYFopdsTA1CTdO77L0c
Th6xJC8gk/as7XCiwVHtzzA4ncDCVRb3jKRmSMeoeAwXPgChbPw11Wc1kMw9HzF/gPKyUf4cT0uJ
DbFzHjQbRI6uIjpV9m4nuSjN4ioKOlsYS+g9BzlhwzpUwxPwoRONuqbUT6Yeb8FPWbm5pUkrMz9W
PrjQiFWg+Hqh7S7AXWTwiN44LzJuThWq6NqBh1TtqJiAK7mShhKzOPJIuYMs0DeStT4xMG8zVwt5
2r00bDfOfzJIc3HnoeRalNeYYbBrpPaXQTdf3aZYXGjPZpyPPp1jaF3p2wAAAS8Bn55qQr8ABLZN
g5w7LScXr2aCYC7G4gaiAD+D9yzYDlWnziehjL/PDZemsfx2mRQUlqYqAn3tI10Q2kPckErxbmcu
DxEge96FFOk+rgxE5dWtfJo6MYTzafmr1ovsxAdemBPr9EtlOpWFYpAsPxT2zcORKj87XaV3d6/P
DsRWSLp3eKrDsU79xq1IOY7Cfo7i+T7COxGlIEyXsM1XXSHwQbhefKWS5AJmHDO3hE+um7GuwK+i
jlanvnP7RXf3iFni3PfhxFM+C5f+hq64Hv3ihHx17OTuMT83dJdespKJzqNGFyL5egnpU63T0o+P
X/WMfppBgqX38c5XL70ifcxMkAzKOxFcHf3vWH1lZ26dzEhegVA/vmhBEnBVR/KjX9hYTPfJZayl
J66SrtwZz4AAAALFQZuDSeEKUmUwIb/+p4QABHvkbK0WrMUMAmKJgDnqlkUF16iap3b/LkXomufd
6RFp27e7BoR63oN8jMeskLgqYLz2r/Y8O8ErfaVGGvjireOaH4AOANRkeXKRuXfv5OtL0kKnKeVF
J8T1XpvSi96QWcl7OxcvhPmCgdJu9TAKo8JKQl0lhQ1hzMB+LVHN+g+euRuFvic1IXYAKmzjXLK7
VF+mc1YTMDbkr2opFiD+Pq3DawG1sySTBoWSAKtmJbjgmArDBHGGUCmQ3JpXfyIL8VkhrR/+n7fA
j/+gvWed1jvvA2vOourIxMmkd+3ucniqlx9lW7Kt6oxfT2+sCUOSDPRM7cq5wWRXaI31l3GzNHKF
0UC2IULbJjefSJEekxOBBI5mk5oT7jsk6SIll8vPk1nwQXrniciQCb8dBntD7FZAoC0AP1JlcPb8
Vki7JHaYPhWddHcZsWxnjw216pW6IjBjbKC2EXloKRhTR+Hl0q/9GhR+EaED/HuwJxuTEWJ2kLnR
ZRypsE1gBefumu9/3nfC3xM7Zqn9M1wg6ASWYmasfsM3MhY+k9XpzCgmdZPUeLqCcRE7A3x0eSXH
uD9hAVKxt4J5nRJRRGrZm1nW/3oz/5KGSajiXtRdk7k/4tavqwx5xZx9Y3tPgVi3Sc7lc/ryRBSu
s7lviGLw4eCsghjXlFWfdcWfCaddQQM4gZn82kA3pm16ETjqoI+fmGDnfTdkkccrFKDDa79Fk7wd
dTQiNlEX1iEDOMOY5rMz4TJa8ktScv1cg+nngy3ClUxzmqVnqgABZukaNrnr+srxuPookeIS1nR2
5FJ5GoPdTUItylOMgq9WfH0PZw3t3XuvfOOeFfbz3pUUaWN7DM5Qv/IDMoGgldmDSmmTswFhUjI+
9oS7BJRaG7NiSIr1pC6lP+AAYlrf9RYQX9mOOr7xP0c3hYthmQAAAYJBn6FFNEwz/wAC1yuHSJ+B
kIAf1gbZ7ECpwZFUbU4rotAARATXrfrqRs3xoLmf46f7OwF/h9WXKeotbJKrcuZm/Z+qL/oOGmH7
Aq/aQO8KXyX5D/g+nrj8HuN2e76sqBh4Tj2hjS6EiEFDqG3fz9SlWWjYzpOqEzfQv2d34OdulZg2
LHCZh/HJV69vWCvTzITbq3WADq4fD4x6H/j8A7L6OEuwIQZ7wXUhQ3L9Afyq9bSzNsYTnBm3J+Mm
+h43J3BPp7eypldipiM5wkijO1l+m5HPVyt4tIcCF1Api2eI9u+tCR9/MogoVwspnYvRLgTj1Fvn
fA0Hmx5vsQCJ+8CudnHsS5KAcqsUObOYSXuWJDpko3D12gdplty68xhslmZa/Ui5bju1QbHTD+SH
wGjR2MSKCoo8QxCgsaOK6jFgkHbWDOr1I3ZOG2z4ZBJq+IPMFudJjiZ92tGDp0BxPkBNZeOQYge6
+zc9VRAokhSMBp+stdfB+SFqLIez0rBFGLA1PAAAARwBn8B0Qr8ABGlG9OAcADR+sCS8ULQ4ATt5
0Zkkzs+EOZrojWknprYiis8HwsHD1K9XGddAnFhz5OQq/YkH75g2qT2kU6etPqFoFDGanZltXhM2
rEfXOyls3qxeMFyioJB9WemEp28+uQHpM89dqsK0hNaU/Qn36wmZxJjdBrfgxGU7n1rH4QD9zon7
03NYqWTsUlhf9Q7lM2LL0jw7uaK0yUXBRfSKm/SvB8ylUYDv7Nv+Bpey2732vmJOFpfjN8x7hc4s
9BbvJdICsSsIvTx41Q+P2XhA4CO1FDcOEzGwk4kzc8FsTvSwQ4S+CBTRi0xwypdaYeH8fiRG+mqb
Z78GqrIIoha7AV8shjDjT4a/xoBW2tAifV4Lr/plgQAAAPsBn8JqQr8ABJdbnZHD8GAIJ3DD5g/d
Lh7ACH8gv5ltwihNdsxIvyg1dg1LnmzEaTcozIFK0Ermdd4oT3xYNgbUNPUbWTC3Eo9Po6BrzXn3
kcLm/vk/uhgcG+B3ytEnPbxfZBwxF29iSDvjii1NhD9lpKwFv84JFbMjh+/cCECFaVeLhip5trso
OK/i+OfRGi7OH+Rf/oEOpowZY7a7I6gtddWI57DLhu+H3y0gwkfP4bUcTbp2BrK3coW9YsksusIX
SOAdvAA8JFmGRU+b+5ZMKayFEZ1+zcUuXJ41fiUdHmk03S5V7cwBIPn/N1mN/9pqwdqnu3lwYBsG
pAAAAXlBm8VJqEFomUwU8N/+p4QAAzam0lkHwADgxYU0VdIfGRrfBuufvMviqrs8UcHaPFE0ku8C
zhHtYKa/NZM1ypZgFeqPqUAhh04UNKCqmgwd2j4ao/E/U5JnSr0mcGzzZVLtzmeg8Eq5VjJAWlG5
//4w/zPzWA5KTrxHqTFKPs8AsOpdfXqQGqDqV4A3h8kB7XDUtSTDJwrjQ+FTZxGtmLZZ2OPfTW3s
dorB8B9BBg0RfY16+fWpLg+HItXqEGjEud79VI2JVFHEnusPnOHEK/mLW9l2YoAMDqwaQ1wYS5ss
kPYDlgHJuOuzN7tyoM08YvkRpRydgxhtP5xeFilFCfl8lSAQ+pTcl70JBwrpLdbG58IqYyoVE4i5
C1gV8pSeJmt8jFZgQpbCQaupSQ7mxmI/8lmWCVIL7uqdWEjHFXmNU51In2Gl7tI6SPhsVDix6/jO
YBNF2DJ19ibRdtRXYcMmjcoj3/pQDhi3ba+/po1enuKmXGYzVD1BKQAAAMkBn+RqQr8ABUOT1d00
aRXGJekYsbelwAmmvbk14zwBOdnwy4h1Tyd3HFX/9Hb4CrnfrQQPg5dkAqSOl42iaF2PlRZfAC7o
wCYUoIUNs5d8HSPEL9kNKg8AD/TO4vQCux5dsdY/xyTAys43TSWGmfD+QuayFydSkOGJy5TwMTZw
EoSC5+jmhSMf6ZcznOZ+F1fggs3cO5xiXWfUqeBfQ/y4ovXyC5FyyxiuMjypivmcmmX3PBziWHT6
qGCWVbxNIAw11w3eLz5BM88AAAFkQZvmSeEKUmUwId/+qZYAAaAuZjGAC5fCqb2bWrJXuuR2EQZk
x79wp8QTrbIMKOqez8WMTnPD5yavH1MfhlvSj0T3rQFGVzDRQTn85o0BGMPa7PNhMIoCvkarzb1N
h7/o7tnGv05hU/Ky9CAZZk3uAo6BbVBwN3alxxF8jLWc104V7/df5ETcrznX3AHRoezzA1mLVW06
14dNKU954e+YEc3SMXivaxlijJ5fdq/ls5j+n5nDA38UVa4SbQNNRSl33NAZAIQeHRVjj5+R/qOM
ABaE7I47Vh3wfnT9eYKykv+b5CHT4kS5+/rZRnP4uxRmS1s2bHMRH/rpXtpwyUI7N2WFpBoDv4uU
/2q9zHXdG922YpNUZpRFgpShZJLCDiZ7HRtdH+XPLkzFxLZPXhc3anzKWjBd7B5lbODd/0LauRys
IASWQyuU9qaCTaE4nKcY4decvWSXfyYcdsGt6PGg6Gz/H8EAAAHEQZoHSeEOiZTAh3/+qZYAAZ5Q
IVs5rK3oAJMHlaubMETowAEoc1vyfo8nOm5xUGUK2MJTPsNo6Iqd3hkkex66Zft2lnqchr9Z0fkJ
iPF3D3Gm4f3CLzerrcAp3LlvHKi/4Vv+a6L/8FRQG76pmqHINSjqkLox1BERgfmazVc+vihP3FiM
LiLLW9tTpfA8YwLsJ4qzynSqgRYhtfq9CQWuhmvROypBaSrAOGm/zp7YbYDqGFFaZ0y88PxfPiAG
Sp8Y6epbl/4yfzA/4+ZpnyW+QlPPdc1ErKtT1A1SehRMKIzvcnhiZYw2HPzotCgqZsSzirp8uwLg
msSq8+INGStbMqQ8V08f7lzZu+y+1Rspv0pA+LtxD8MwNKsA6n5p0UdTyxnK5apZnWRsXBQ6hKaw
Zmhdw9iAldFbtnm3qfaXze1Ok7FPEJPCOhScBw2jocRkskj0vxTjChHD9MfnHOhY8C/v0CjpLJMV
lRWReRvhG9ooZYmIiSvs+FMfsX/DhtuMo/zaN1mA/bE4PS/FW7xnSQXwzg6PG0u/LTX8FRnQfHDx
ZjCYCMfNyHbVJNe5qNW/OrFBeyLh76F8AS0Ay3Ih3L91EXcAAALzQZoqSeEPJlMCHf/+qZYAAaiX
lYAq6WlUAaOkvCX+50Axyq0x0jkanhE3VaTaVoW6dxcps3RPbcCIyuVUEV7zY+B8cc/5W2XvlY8E
DcNLBxC+BbWtatVBPjp3gqflgElKgrDj2NOQWViGFcaxMcG4l2hipTkqU4O+s6dx21kdt8Mj6+t2
H6geGvXgoDqtW5+fTEg1OYmp2jrsf9ygEbvK5bEMmKg1R9B37va4SpbCX1lctBY0p5EIlp4XcRJc
AzF9mBy9GUHiW4gnGvm3aTPvJ4HCdkXwSQDszVrQ3OGZzeW80Igx0R9mM+j4n546QHkHgpy9c3RO
6LXOt+UDMMR5xuNnTsP/95aC7adtkBvXP//q/E48VWmlHUwNUR/MRNugo3WllcJOuXY/R3MSI+pA
upPlq2kEeifJFgqiR+UTjXitqqD6Mar4KBHPkMieFImoV0KeSc2Ac3HcCuv/fmWw7zvVdvvCoei8
ElbO8RiTdoMdpbkYsWbl6OQxphWkvmkqqPy0hYZ7IxI2pz4rL80lOXSjX8+LiveFvqyVhpu7x1he
sNzvsBwXKUDB1cMOXvwscAlrlO0fzPSN0QnCxGVRNMSJ7KygCuL+CnhiLh08PczrcK4XmnNXSm5A
8Tfz4UcM7BsV7VMgt22/wRWTLQDgYKIrd55ZgLHSmSSpEZKcAJ7uQ9M6D0Lau7Aaox8khc2XfZOf
YxEkoGSsO4K3AcRE+gWXNQto2ceQI+Ds/G2ndKrlM72d2+rr7g720P0jd23AEz85HFDuA7y13DNE
4Feq7+W7ok2OjcJcc94obwn75Mvf2be69WpQtyIUeVv1xlQ4pxZF/hIFeNZxIcpKiFGFNjh//TyM
uMbjRTKfLXqxsysraW+ck+A7Kutk8DaDoSVSzP2prwOK+KVgY0iu7Z3kPmUgOqckuJZDo5oC2Cmx
zhKEQxTjPKtcMKGmBSdjyUzAFfomaeOO3lRc3/qdknBmJwDNpU0kpC8/j68q0t68RDJlg8AAAAGX
QZ5IRRE8L/8AAwe56hSb898VE9vSCjN06IACdWaIRjyl4g22kvmQJDhsEekd+B5rtWek5NYyt12G
QfpgZ0yn8Yx41+MrN8FpsvloPj9MasWXkgq/oh8HB4TRNSZ0yT+PlIfdbLWnikVQvNrD4eeEn9Ny
OgzHPABxVM10Oa9Lr0VkZ8zvsqtnWVdLMrFrULIcktw1TpJF6SmsMM/P4xVLOEjDpWY6cdE+50x8
MmSkPzY14dDc9fuKeJ1NiX5BAHilBDuG7b0zW8K5TfaNWc39DyTxBiqv16MDEzrmcVWhoSclZkih
z8T6WwB0vrWt97swjcEqpELGb3FhMu6GKztghrjvhGsyUJT/D/XmIUD+cRGJcD1bpMPy/+fUy6u4
o8f9oiHI8BtVwVZlta2oAnTzAi4dmfQ42rI9s+6qcg9AwEFZ1IgUv+9Ct3+TtnDxr4bruYe9NBvn
0i/D0ifXXf4pVKnBwdYkvAfUS3CsgFmLlLkf0d5SCpGUPhtEPUJG1bBZnv+hDyFLLAHd+yltkhcc
DzzaumXb9EAAAAGRAZ5pakK/AAQXYJj9VBAA/nbCSLe6fCJe1LiJSDSHd8XkbqcT0w+I90aeJ58L
RXM0h8Md1FjRByrMeoOmEgg4L80nxfAAbr1IS1dogSlH/nUPRiASfB4mgGf5vnwKBcRemJ4cKd+U
mh/TIVlm2IF5OJXiLgonh5nK1uybNGM/Rp7/jFDpCLuALYXSz8wbhTxg/+TSbEkAsZ4itA9f1Fll
q+EfEOPPym1UPKKh0hPhMFbThAcSUfWlRStoMcVky4ciQXWvxn0P47/g3az81dN+UyWBX+DQprsg
8IpgPHt8l9eLZDRHF6T154Mj7Luictx07lnkhs3N7TiY9xPyWqnkv9WbVVl7m6O0sT9/kR9lT4l/
+Hj317QG3ytaY9sgePyi0W3BVbrnBwlpUTEHnZ4m1xfJ88lgPfT0awdpht/yzXsXQZwyp9aR8wIP
A/O8Eb940JuVZgzu6WbkqhIiDx64a80Q8G4vJ1BQX6JSKpotHNt5qsucnrXAJgJYE1mllP3ZCH3D
wUi/Z/ePAeEr8hk4TFEAAAIFQZpsSahBaJlMFPDv/qmWAAGemsckALZOBjBNyZ6tGTfny24i1nU6
xhWZiv/rU27AZRy6LfjPEbME5Nnw0dtAt/j9oKmSb09KrAMgDkuTJO8hgLytikpN2dPE1vBuvTAs
vS8CsaBKTuR1DjeAq89mynb+QV8kooVD5LSd1xDK4BXbHaFyLfv4z79xKUah1AxEODRsyk/OUCkp
J8AAMrxioPF0Hq+usbPLrCp+OrBE3AZ6yYYEXIkmjnXA+qKKTH9qnGevo7Izi11IsEXmNeyay2CF
ejwWdm5u33VsDyfXn8mu57E7sZbcCjY1nZGj7/tEjKzoxRAmYTpKI4UeaX8m9b7HclzXvYOjW5jr
qgcljQKrsh012lkKaOKaquaJSbk1BaFOUgFy6erYUhGJd5mYscBGyVFMslqiozOdmznMwmq8J97L
EEmXlKBr0lGRujAbgZic7n4CqerRyDXz/Ha6AVvpsNrC215My3fTjvlEkkC5SzBfGynoMDB0T2EW
q46IlpSbp2v0Isnd5dirS+6c9JUKq+pnv2qVM5Xufa9wxxz3evi3p2he8T20SLD6k98vU/GgLQA1
81RO8vDWQzCRfiFeb0tfRrNtgYMqlw26O9GU9PAFTv5jnu6jEr3ZUchw0uesbE908E/6ikuxdtME
46cadxgq63GjHnbPF6Da8AWBgyvpFgAAAVcBnotqQr8ABBg0Dt12vfV08X3DdABO3bCQDgzeZL1o
9gmQV9Eh3QWs+qCEX4+S7I1H1IRcnNhfx+s2xp9OnzJjNT/9iX8PXpWblY9e1LY7g00yje2MCtvZ
JjUWl9dNkBPl2v0An61L75Kb7DVPKPysAFHgCfXFbJN2en8TI+gIhFcVM8VpPC7LdnQA13724Mo2
PMBYSiB4O2C+MQz9+tSTqtDF3tvpQjEzrIzutmRwNbQDBswNo/bOAOhLZZpXE2NWoZiEM5l7mZme
rXYwhSgOKJSxYl7dYA/fqZFy0+l5jBBjyEWX3z9019NvB6jOrwBrESOW8nPMwHAC7GVobffM+/lr
YXODv9QXadLOWFMpjIHKyKKhMLudPhO6t0A3Edg5a+XWR9vlBshucTEGpnhmaX0AIQUq1NiVUrix
VdTHvn6Mhr3qJ2H1DKHecYO2mcNXexbj2X+AAAACcUGajUnhClJlMCHf/qmWAAGqFLEksAEWhSVu
isG2oR3+bRSqrFkWSS6wXW2fVF0QRGVy0G4TUhGO6kUPjkBW+Eb1ki/hfNAFrrz/FP4AcEwTXpFX
uyEjba16fwlYqCFnJ7WsSopL1BvofWfbTs17Mc7RMudR+WoafXh9KmfX+IcAtoJPcl91nFI1RGXq
v9sBFvTp8+YHaIjgH7DKZ/BwxJJIFLUZYJLrVtqy0XWRMBD4GF8o3WmQrfAV7iF85q/oJqgtX6YT
dqsDDCwMH3qYjYMqKdV/e2XbuMZZPLkYAsV1e1ZWrvze7MhdeJCAuDy3lDX1RaVpBhURmCwqOP0m
IBVmRrkS5A5waad0KhUBGjh46li0XaqkQq2b1Tpx/oIfdj0ecDF812QBrmXsxcYYZQlkFOizo+TG
WoSCr7b8NmbB4rJU1wKOFvLclgU/suW6+V9X7yCiFgWBMAMZ4uFo+WnRB1vLlrc0CK8LZkDdlfd6
V5TEIwWiNS6DHKLaTFK5NgOzx+3PoUPabDhPuPnALqXha6upqrbaQfa9tk3inTam4In9if34yI98
/BfI+Esg2BM4+RJxcdVBS8v7wha7t5wzruvyT82AkrE/e2H1vwvV7qi9h6IBhpMVsMjD1gE+R1OY
NHUoQStK28O1O3ZXvbLth6oWhpdcgAMN8Ojspc75j/QK9j17lEjntokmu/MLAddE2Qt9HvfHPo5p
LeyPE6bA2+sf/8hHflKMTDTicA1feeC1kc9wiuztGuUrfuw4P4EvykxPCg2w5yLEjzdjP4gFJe7S
gO4wrWqlGlviEjxVWF+7fMU/DuPN/S0XN2r3HNEAAAJtQZqxSeEOiZTAh3/+qZYAAaiTBIAcH6/r
SaAaEkLNZ5YW8PNc7k/2QVF9UQuNcdJOVFFQYZv30AHqh5fIy9fWrCmOW879uiNRX7l6/Ykw/9Yn
ZkKvwgLPoawhOKcL1xY1SPpzz+Mip26VQzL5dZQ2+6ueqjzBqGPI+cvcm0fZFPuq6wB5cikB+uQ7
xvi3XCIJbBMaAYz831a4wrn9MLLllTYKzAvctcSxfd1AJvzzazEyfJk7TDHWiRyTg1dCn6WydLvy
OgniCdJ/uFM5nmJe19O7hdqds84h4qgXcQz9HYaVi/DbzckfaqfIrPjhVX/3dvnT9f2HwJOypXR4
aEyNGea6cjRnHyVnvtfWGGRRlDIcMqDn/gwUrokMYoxg77EZ4du9Z39E0cQdxCjFzapzEZtgzgdS
iq+1/MyuLRYib1joQ34AqJlrYiHJKmTGJQi229wHafWD9aX0OSmBA55M2sw6cSM24mEbqa9r28s7
AfC+MI6VIh3DN2CI1Mx9DjbYORZn+WhTzalBh+BuoLk7T7cssD0c99Wb03ZUYuUF15DMDU3nsbbI
h+yUVOclUD2KrF6LolNcCYov9LhK31alZLqNzZgZD6l3QBz8pdkiYos/kSYlASFJ2s2mwJ7fUvZa
WjQh39ZThnDEL3rZGIKGQSPRsEVKtNHGlzYsEGtAT/QbL/bNQ0OKpzC4RKDK9sLVycDHm2/GPE0p
54yCded6GWgESsThmhNAawGQUV1hv7ECrg7s6hPLVq73KYl0d5ROy/Z+nNVb8PCSt8Lpv7xUiB5E
uhn2S0aDQPofqGjus1M/xl5m26zKVXibFVYlAAAAyUGez0URPDP/AAI7kIEMc28fUr/f+8AAukFP
94YXjoF6WVYEfXJUSS+X1/mqJLTMHC4dUUvA4cqxPf53vOVykVZIUHUVuLESfg0oNdAMkoLbnxNn
1iaaTD2f8STE9/XwPqsHWqSjj9GoOG3FvSGRk3oi8zNeyGDL9CHV9gpMtbZpZGWTiFVjdbnUt9OV
ZIOp/a7x1bYEHlDF2MbQhV1506hQGjwQeJn95i/QOKe4KEPnm6UP9VPxSSzWVnvR8IL+skL0kOYG
Cs0GgQAAAL0Bnu50Qr8ABClUaE+sWpGmm4zJdyKQgBCa2LlWBm+MP/jG+41r6Vbuzgrv+02qv1UJ
eUTWbBJjxrSuQeaNIzqIYJSuxdkp2Mkt6Vpr909vNtOHOzbi+483jDuMMgdHc5sZgnx7M8nmZgPa
tjRR6t35UeZyh7TYbte/VEfKasGHE8Iqva58Cy6JRAW1mylLUkLlpMhZiuamkA4KPGasgC87eTp5
uYsSaYyeVjRUEtmq2gfA9ujGGB29DzPmYQcAAADZAZ7wakK/AAQpVHIni20zKLVOv81gBDLBDIBk
ap21rAsOE395krCq9kSHZySAtG2C7gD2pfYaTgqqqonSyrgPkg2ySRdjwSI81BiGW3ISv0DbKp/i
SY/iNuH+GxknArSzbv4o75P1HBug72RMNr7dGWDi1BuUfn8HHl5x62dqN/dhqdCIzj0BA/ygMKvZ
ZOqZXQAxpIJx2on+aRqtilEy7BBjQ3rXOtNnftO5jpglGSZYQTJuT8Vca/ZKi/pmIBjnL9HFfWbh
o5jYyR40gnYkjq7uUyU5xUOCoQAAArVBmvNJqEFomUwU8O/+qZYAAkPyO3Z+GUs0ARaFJgeeTbqP
tbo4K3NifxmCvxjARMP9Z0ynA9qTHAWF1wUmosrnqaguAVbOw9YO1skjWb38owUDEi1f392pvgI/
4/XE0BkkG3bI6SO/Ao0IGxfEVq3QtG8x7310RUBpGK73SXsTIjuEfVEG94EqdaKGj8FhfCp23Axu
7Gx/xkSjIaxe/vxUpxZJOOhrtQAorQSucOb34QgPewz9w9xwPqFF9Oq8FqVyJsUOmYJZawbaaaMA
aZRzJ/TbfSTcpsa0ZB6ueg5/PxWoub3p11J21MKdqTLgx1ZInDBuZrpWLbxywnBFmQGzumDKrZgx
O4Wbfz5j01Vb4ZsTNfVwzfwPX4ilDwVr7N//qyQZqzAnieU69/8g4+KUlOyLME2cn5HYSSLqYSSa
4vz+B77fui7oJ41q0FYjP1W52n2PgQHuTMkS0f0a5ycZDkTlAwT5BAXKdKY3QMzrIfXN2GTU8RF+
h36S4pkkdg3REr8f8zTYDqg1kQxJxLaRTl3jYf0ccujcsPS5yujzzdfKjha4XeITNpy0pv8Xx0hM
zh0mdsc7i/9DIpLIv3IWQGBHEbV4viHikNwNLiXOmOShGxPVX/OBjOnnxqiEfceHu4du6JO8/3sS
DHti1zecB/sJ+IDTJm7lq/VbAU5m1oyp0rsh6nsBhfloLYZmLGW3zn6HRZXSyirrWz5hxqL+9JiD
yIfLpo2wHrxqdTs+IiNrbSturidcZ8aH/IZbqsGBoJPD+DvEQn7kuSPLBH/XY1xBLtems0EFo73O
K3o/Jr0rpUH7JFnppo5o5LB0oWAmDGXEC9i19zLcnLBx+di5b7HAOvWCMopYwL48Kr1HLBCUiY9v
Zbc7LNs/jncxzgDF+KokuCyS2urPGgZv00cDzZJ5G3EAAAEVAZ8SakK/AAOgCAaNm8AHdQh43+Fp
xFF1TxIh/Z3vV5D5j9UMVe4fQJ+CbPKmw5W6yJkky00r4QNRlJJObUw6YS52r/0H9VAU64Qfio3w
yYwjA3e0OHcR9f1I9MpT520rnk5qXgb3Ka8Tuubhjr1l0Q6jzEHcFvXDy9gOxyFX/oEqAauTVlmr
HUhgaXw0W4OwS9vhLjU8yvEca0h2bbSUbgat/wW1r6zDdCu8mNAJMuLMMKr5x0oyKeHcXA0obkIO
edBdJPGm/vDf/10m6OAQiA1txhg2s5q3bJ4UPEMxWvxZtGWNLGHs6YFTGtOOIWMpaQkTpKXHfJfF
s5oni1F85HKDSKc7JvDZJi+PIfjHqchtv4BlxQAAAoVBmxZJ4QpSZTAh3/6plgABqh0d3kgBeiAL
5iu4h3PQGGf6+DgMIzTeiaARlaBHMhfTKuGhv0TyH9qMtxyfLMB8OToUdD0CX8xf4SgOzkWJsr19
v2E3bE0tS/lYF9wXDq3gLCEurFuvRt0Rl8iaFLuQvHGowG4Pje18hk9j5O8DB383/WN0qUeTFq3u
sXISe/raTBh9hwWvZDEv3Ab3/Rp7siZzz6PJal10AAcRKocIJiQl5NUV9oGJgyEOixTN1btM+upQ
4/h9YsRCEf+T8IVkLOCFBFjzviRs1mtyCc10XbjEx0bx7jECX7lLsHUWMoN7jqLsjrfWKih+h52E
l/X9DCZltiL5j2zN2Yir6L3fZTlXdQ8aqv31hNTQTIQVWgkpZDr0c9KWIc9lJAdNH9JlyLOkWTSv
Dn0+MRFpcRSlIBzjyFZju1hMfZxv+a02A3MUuu45U6zHpiJaVQ0Ih2R6goan+R5evncAohrZv/TO
1wbIXBkQJXuVAPsNnHTGSxT8yFxf5iN6aBVOKYiaVf7wlw4UHePblG0Hu2JdKo3g5owJwLg5UQYD
PuZJkIkUJgpyJ363FztsRvJTmIGWgfWJstyuDvJ704A2rQZQAyLD27MfWzeqiuYa7qEqEDiPBtbv
NhzI2Otxt1d1xtB6Vj1gxAoYHEew+U+TexbVzPYYtb75/1NN4WkrCFdLvCfiNMMl9ar2KqnkRXd6
LT3+muiLtEXce82HIjX9KzptIG+aY3d4fkwKd1XugQD6sPhaqhq3Wp4zawLpI6pGujgBCCW/MBJl
Yg+5UEVhHNddA8OecT/h9CpzCHSwQBC3qjhfrcVzgmaQbl4ZnqBf6Rp4FtCsHbAtnYgAAAFuQZ80
RTRML/8AAxDjDKvwW6ePnAAOSNHw0J5yOZZOmWUqG8HngsHiE0c/eox4bo+EjxUYw2hqar44XMq0
t4QQUwfDQOtvBAE+ku+VWWHCBPTdIPJRmvg/GgSBwieVKq61HDyey+V8laA2DA2XyK9gsuWjsIiq
49zNYn3zEnkcYeBKPRFedQ+BVR1Y69N/M9CuHBdo4yqLUHX6uyP7hpfCIM3zboCzNgyiEsJfakee
ioVW47XDhYC/hPvi3SgEjSHlo2aGjN8apu+UVGbkEEuOmujfOU0cOe0syM4lKEcaoF7O3k5XOTf1
xRt1R6Dd5ZOWLDJp5jc82jhhuCnrmggUG/y1aOizB9zsT8LfA3dXoVPI/DBm6wSODjQYAtc7mpXa
4vymxkfIC3PnCWr76vBsW3a8pqEiLTgfBAsH3fOW1x05nyG/PoSqAekXLICMhUm8TG/bgxr0cLrT
RFoPJ9AESdXp3Bon0UEVISZP2YKDAAABVgGfVWpCvwAFMsyis1rJU96GIgj0AH8bUREbQoncXwH5
qtK6OJsLZPe1qM6f62QEowvgbDSQ/UVdMLWrUASXnkKb6oNnEuDFcc0asMy1bkv6EAwGSOBVSRDk
ixKrrrrxVlNbeEtKAvP4KOihWpMlP/DElMoOctYGZ1yJiNjrD5hIs+o4oVjQv//h8bSDXcv3hcaO
2ADeJNWitX1+Fn6gQ9TJR/PepyocYCa8YqoGsvqGPPW0TL7OjdcUBZC51c8InZ1M6uP+la/KgZID
gM4jT2mSma2iDpt7kd3MlxxyL7sSGtADu81iYRMueJlBhEsT0ggBYm/hAftAmhq6n4/XZleOKfz3
rQLOCE5wtkX0KKItYX74ufPle0iW/+Zq+x4Vxxpj9D3773knO5md1mO7NM1X4lNH2dfYgJCeQxGC
s/w+TfNM4R2j4FnYNvrELapbxPl4EUZ14AAAAdlBm1pJqEFomUwIb//+p4QAAzZ3Yd77D1zABbJv
nxJgD7B/j6M2sjWSJf8hxnfjbs228H+NvO5bj2nBJoZw6qFXJ/Py5mC5ylPptUI/myU0QkKuEZyo
F4qPnQEG7tadUL/KMx9bLbqr0W43E5FQ8hr10nt8Bvzgja+dDDWz4/2IVCTuG5gQYiwVPHUAVLby
2e8VUoY0EEUPtcu3swmbA2iBC+YZ3fbk+1wnWfpiURQS7YUSfnltpTu8X4LjHqC9e9NB4RweTBU6
rFzwxI4oriAJJdpj229262sTSfXB8TfKMAk5P/cgOyj+IUPV8v151+291mCPpUk6zAhj66/+IkIC
3cUcJ2/mj/ncvKum9SO3TSmvBY9n/04alR30zBH+gvR207Um02fGI+yeqqAAHri6VSnlWXMrUR3J
medimGxeEGVVqNORCDYOcuHoch4slakPEvzF+O8gh00tUgFVL3T8M/gBcalXAuSxgx8rr5E0Xj/5
79dq1WxxsdaD6VoO5q/3c0w6WCV6eNXWo9SS8wzB+BlwAHf1bK2paDWu3qz49FyGSvrSaGB+eVQH
NfyT15mIFad23zMYyHV7FsSdj/FfcqRIaIWky5ud2nH8z96VENDztKT40OtiQQAAAUxBn3hFESwz
/wACO68S5eOJ0+OXpysAEinBfEEnXhB8QLWH9H/CmVXwMSKo9ioy9H+UU+QWGHSeAMQ8iPBSMMkb
FRMCUsQ0pvkXcD7GCXhSERWEa1V+gVJ7C+VX2bGJQQ7lsS+ahhQl8fbsFadg9wg3VFbZDeouKIqK
h46SA2hONWwAlS+uqNHiRuyPOX5uLfnW+q7+4vFLmuVsLyWXZRvU6ec3+Z3oDhGJbHZFBEea1ArR
lINrF9b40sD6rVF+ePsjNoR9uWgC9f70UwpZZynSMTB2GmOMn+AtmAXnvioD+xGTdJI0Nu6bgcKc
dE9rv81BIHahewnnt4ZD8XAIVDcJWcXQDbqrr13kOu9cUYP5UZ5j8MjKDkKJMvyrrRc8Z2d571fA
c1b/n238vnNxgknXGGz07rM1nvALmiyQQOKnErWKMDBLnovziiCswQAAANcBn5d0Qr8ABBXTDfOa
F6e7MYATSp0c2Juav0ZswvGcU6BC+C/PX3gsj4z+HMXtt4F7hzNgh4bD0ocXeIiG5VzUdNSUXx2s
cqEViQp5ohilkMGECIGks17BBh2ydwfL9F03yLY9lM62rDZ9rPR4ODABVCzfsK9clwML6uoqtVoJ
0ILrPByTFrUUJcDUqGmeZIUF/7ZgNOa+xYA9lzBPENca2IvRJQVwUCKUHNKQH8ddBtV5EB1OfLvZ
R15fcSwM2p7il87FxiklSH+sBWHVI24XJICR+sOC8AAAAKUBn5lqQr8ABBdiF4hm85wdYAEoY0xK
Fsd9a5gdmxvWml1VU62RSGFUScsG+SwZ6Jdn1BPVGdiqr1iMl4A2mDNyltyQfnHVTsfIX4E6YoKz
KkZedWA0A8WqEj//YUWMPAbEcXhzcsRMExKPLCKlOz1DkXfQotDF2NcY6qS8gseeuRf122O/LAO2
Ce67f2pBpe/7WtOS5D1UZX0iHehmqCEG1/hqHQsAAAGDQZubSahBbJlMCHf//qmWAAGehfePoAWz
LkczY+QIzHN43CJeDlKEZFNMa26r1DPa7w8iOicVjabO9MTpPOZlKbB6IbPqqrQBAf/gSoC0N7rU
SLmVE2Ng+5+SvfuLvGFqlOeg7osf3Fs8bPsKGyqzzV8ECdaZITbn7rZIUj5lw/0P6bni5Lh+04nj
sFaGiWLXbaLmA5ywsGQS/DILlUlafGst1q87Emh9PzzH1Jn75MwjXaFpBMGJSahHD7tCzdDn0pBJ
ltIWp3ckydcHJq86rMtjAzCMrOPFIpVKbZldjfNKc+GmUbeJA+5fMVcrsgtmtuYIvNkOyXxibBbQ
Unmu8p3+kGBTq81d6Q5BFDOCxfKX+BY4mN7WjVTMjSMwxWkY6TfPv+Ta7WZHtZYp7ThV9TFXzWgd
1N7EN5pBoazpx5pKBbqqjlU2bMYj+ml8qyctHiPiX6EjSDFSX+Gd/frXnkjgHlzLio1hRFnjiEf8
kTE7uRUCMzT5OVocPukpmwYy7dxAAAAC70GbvUnhClJlMFFSw7/+qZYAAaiXlYArfwzyacLyoypI
uQMao+wVrSqr98RFn1LEZm1+vqNnE9azx1di7bUVbTx3WJ1OmXG/5AQzIaLj7rV6tu7LSVbqIef7
WBUj+Xrqv5CE5v0pxBXw4MlSfGaraYLFDG77jBsKwi8zXasgX8OiC4aAg9ncLjK5Ts29g3jsvz9C
r9DyGJ9PkrRxBFt/ZrC8jKrE1JTBJo2GAIaSP49LGqfpeCamSiMfFfup41tgg9ahnFniI/cMqy+p
b4r13JeWww9vyXvkEEJlkgr5+olDatpZZBNlHZy6d5zrRWagYNAD3A8t8vpPQBNd4G8RS7pNk4DR
bJg/6EAF0x21wNNX0mu1ccWB0JyVtyr3CYlRnlyuDLnVF56tsfQxUgsCBK7KwRIi+iFmYEvyDuuO
+5kPL5ypT7xlwm4IrEsO88KbT8dAcZF++LqHbNVan3mWs8PmQ8zSIeYblpzi2GKsvt8QpBncKtIQ
bAOF3yxiJ5Y1+OTYkg/FV8C00ZTCE4YTRml8D1U1vMb+8oJP7WrCNWuoVGv3t/w/PcJFx7Rs7muV
blK1fh2Xt9F7+KKVheQBKCCvPFumFB7uL65My7edggzPVFvJ6px835FSDoWVEC6nKk/QstMk7OMZ
cJkY9HswEEz6MD0EJq5im8+t11JkR94A7mE1m45DwyCcaXuPFjJzn3kAiIyPN63kMU8oQYbWru30
w9nruC7CS99FKS+ZduyTnW5EkBypDajHwTeSNBofAcilDOnQCas7ds4BDu0qP+98xl1nHz3Z2lGS
goa0F2rdRCCKF2MEFI5bVgbpfQTAJaECXbGeTsZO3WGBOUeKR0MmSlRyGtxLt3LO6huRhgEsHvvn
F8kl3JOqAVdYE42RV8wYdfLC16QT1FWZuWGvltj5+KDpS9B5MBO1nWDfwAQh0eZ3wjgrGc2QQE//
wlZM0JDGc9Y3WaFOS3qFBlFtXpFIDvFYskcQoE5iEmJc02EAAAGoAZ/cakK/AARnidr27LzjRREQ
KM/iWYtA2dAB/GbvT8MxtDLyYoQbWPHUwR5wYMYOftfrxznIPon6PZvBqsDDZHnQx2/NBdzWXEy9
LVbtEcqp9fyI96e0zB5ePgyzxYAN489NqV8F+dY2ePTJ6FFypzlE8OdbfuUnorDJozJLgdwilKZq
efbfipCMraIRnRGiCTymEPwIYpxCnxlgxvrWhq39g6oAxAy98yY5C6Ng5bxtqjhX9cwsFO4wDMCv
rcWX99O4vfYDANNj8uWVnwH4GNLTDNQuVzudVS3rlXC9SCncb0LfPcLFOsIgbULzBzlCoFsPEp1q
T0XNKVfHQpnf+HUnVvUjDiKyovsn0KNk53pBP5+XmpBDEQbTpKuVD2g8A/T4yMCLoO2dffVWeO1A
6sUzTQq6AVxft485Gz4O4Csp1pxK0SVTt82lSMcqrn48j4ROryKsJRIQwBsQit6FRiqfg0RUVdfu
pe7g5LWYW5Qa9fQOsQfFZnFYp4sPNS0FGZlEtNz+B1yY1JeXkhmJBDGc2TDTPMks+rs6XSeqDKtp
Coz7PhfiIQAAAohBm8BJ4Q6JlMCHf/6plgABoDgPKItABokYKffbzc+blmcOSa/abGycP2fnzrnQ
YncYT9/e3crA2FTzOnaGYdCC1dN5/+4AVzguxE4IpQQh1cbc/f7X6A0ikIruBd6zPFJc3k8w5PJ0
r77Oyt7DBT/4VquZjHOpwjqKeVjBIB4VPG8/S2LEZ5WQXQu9vVEMqs9sZctpWyl1yJgzXpCeK+hR
/AdNS1P9aij9163U/kRS/Uz8BFWc3GVUgcDbj4zI8klpTpexUIr0UDYPkbII8j9ilg3SV2RswvM9
Xa56oAlfeqvXUvJmWVcQio2VFKinSN+g3qavAasmJVGy268l3BtJpY5kTW5CsjTf8nmZXcD4meAh
F97bredH39qQWl5v55byxOMapkp6D+mRaNAYelq2ogMPJZPp0VBoxknFC8YtLz1orK415ywipuKQ
P8WwusKwMdOoTHYVKLhCx+Wf02NAD/dmFCJvuvbq1u0VooXvOYp/Ng6VltXvQLLQ+Lyv8O/nkH2p
K2MX31JR/0stlT0eXVdv9mTyxPelt9vgBMZnMVPbWrVrxBrZ2HGQXtSCij8vRUR9HEsSQsDByVYR
dMuwunZzTphcZRK8VtH91TNlySYqJamROp4oSG/pccxSrqqsq5Pfu60ibF9Sx5KTcAkOPhCMc9Ug
RiCEDkWktfOYJI8/rHFYUV/kaz8XS97htW7Dz5FQ4pffgR4TsuR2tnnXbrOeYpT4/bGw/We6o10q
q+i6dE2xWRcY6ogUtrJq18GJ/WuYqIzSCJKEdfBLgsebB3osi/QyZlGEMKZgt8LUUkGeKW/4RCqJ
k6Pu3q6cz7f2dxj0PvU1fjKg6XfWCtaTLVXK3CRMKXAAAAEdQZ/+RRU8L/8AAteRmKM13+80j8Pb
jpBv1FFcFP+9w5Kq7y1S0dGa2MddggACG+yO3zCxCZenymsihdesBT4eHOlIZoc6oEbiPCEG3SOe
Wh+1hHqh+TAevejnDblJA+yz2ymNNgdEPiSubUMmpOuoojMGFw5c+La/OrDp0u1Xh0flkGO0dG9n
cNsiJ+HV9WWe8l9B5rV/4FJkr23+QJBF+cXLrcEDs7uS1BBo/khwYrMxJCObm3fYs9VDECbqtvka
WJ1sErAbhpA6pxsF7DFhzOipYancFucKY1/nfYo6PawBZ0tImPPmg0SeA6CAyO0P0fYeic7+aP4n
xRmGALZfYvae9lcMFB/P1w1FNF9V5zlOcfy3CfOwvJ+/CWYwAAABOAGeH2pCvwADzMwFHzzKaxxo
ATt50Z4QeyVHJ8R5nGLJGfUIztwd3+u9Ix+nQX1tnFFXIUVWP0nt/QNLEZwbaIBf16nheDSjXC6L
TYlX6O5F9giwZeJWtcSn64IGQDfAVFC9ntykZXR4M3/+ax4JI1+yNAnsqhXlrHvAPDZguJtfig/V
taBd25TSE8Fzf/uwZXXOa8xjZrNUus2UU5JmTphm0ZdVL2zLvp6tSvRK6V9T5FOYX3t6OuHqdpEW
EcUZNhOpPE9YHLsccM8DkCNRYy72Eh2XQgEu0ZBFyJFBA1DGk6d12iMr7pWQaevrxfUk3fcN2SJJ
IY/u0TN208fhiZjJBzTq3mfigK7FDK11Zkc16ObAmVLhHM4YNIWxAk8rRO3B98yRJjqoZ6rmSxFv
ZDfAdkZkmT5xgQAAAjJBmgFJqEFomUwId//+qZYAAahNs5BpDgBYffBhhdGn1+hC4bHbLOz3SLSd
/c3RPtYLCPdVdeym2y6sX3SFOxlX8cDUoco7hJ0cj3Oe1GJA/yvizotS1bX2FTLOsU6Qdz7W72eK
Oa0Lx1VhU8S1o2zVuTYmVCnmM740p2g+n1orC+ewsJs3Bpkc/R33JIfbgEcc/g9m25IMrmh12g+K
Pq7BbYZ6WYBm/1KBTWNHVqSa0ZsX9LCn7o0CpBGS8CJ7tk5dJNXFj5CYoYXmhnD5vM8A5NV0W1HX
mmWXB9u4CiVMZ76P3YPl3T/nEuEhxLY06cXOggIDsc+tTRsw14GCrMQLt16GRBadXX0XPlmHHmyR
+FgJn1CUNwlll1UZFGOBEzUTr0M2ByRAvZhYfDvR7bJ5u8oCPRLm7CmPQ/UhcJr5VunBAReMvT6M
2hopeUVa9oyi/F9GHOVjG0P2XetJ1lZhQH175gRp69+vX/ONPjn/tsgSKAbaxlhls5HtlO9TqnDM
BXU4WN57/5PnlgAETCGNMmvWBbPgmokSvTx2wT5b+uC8gl4FCdT71yX/rXFJ2br+KMUR7yfuDJFq
NbDn9cjnSjgUZFTFp/b9OJr85P2Fw01xwk+dkkGaHCkCvn44LOxQxzWGNopVz2b44jNOTzwauc5e
Fa2CHDdvYzdL1jQfukBWbR0ABDY3IRZkhOvj7LZ8ZxzmREyUg1+4z+wF2uJ4qvsKLCmLkeUCZc17
yBTPDipHAAACSUGaJUnhClJlMCG//qeEAANKcAmEWXfgBVg1oJBzmQQ4SuTo/r1gsdWvSE9CCxpD
iHRMNd78cBfxrTv47SvQ9lATCSbsazbfUp+5wdj/ai/R75tl5J0zd/v4b3c5RJV4XHYe9wrov4X1
jeXM/bjiUxyI2O+9uur1Ef8dMpMc60zNx1a9HcAZKxhPAgoly4SmOB5KnpEpPlVL7Ouv6TRIYzI5
EENSbmbtNWCq0hqRM1+PNazoI2CTKWv7tTbfbGurukfYKNC+0DpItbmA4ohTHge+ibAao0sr+bfO
vpDknr6p/8phN3L+lWE7PAOnGxo/StA883a6vCkwJKKZO6iYXQRfuUhF/mFCHCoMRzgpnmdUrh3V
5ZffX/2tDvbbKHzQtcpzd1Mi0B2Tt+aCixx7ayZslMww1mbTSciBtEjJ+D2p5yrgupn2yJ6zeKu0
9ENJDGFzk2SCeZMAZD8ZnnSi+qGTLL0zdMNj470z1B68xproq836yRTjyJ2smkj0xTLc3PsciQMA
U/Ru9zLyhWpZn2PpsOOYg9mluhOrONX3rkVJUoBa1UIdMUBRlWd+HqKzUAf79Nim+2LxpuR0H8j4
Ck0zF4PUuJJms4QYjF/9wMJFrgSKqS4muWvYUjKldMiD0oiqKKaQprTDBlaid6swK3RsSb1rCC9S
lYspbOAl4RF92UgYjnKBhTY7tDqP05K89N3evHUQJy2cII1bY6liRMzFK5KHtw/seXoDWQUVaDip
qfXoTB6qX/0IAF/Eb/mytS2nmPdM8HMDswAAAQNBnkNFNEwz/wACOxdq6FjmjAAFfk855Isp0bT9
uwLC7qQcls1Ar5e08U7JoEw7aR3A6vhIyLdWm4RpXAr4Wn7PzwF+UKOpgW8bvLk25T9IdrkS4PkH
tr3pnsp9ZMbXY/JPfNa0QBQNlqvtJgnIQqYF/0G52AOS3radUKjIQ7WY8XO/0qmOGP4Owq4U+JQc
BfAMksOBIOD5/MgPYMpjogqIUlO6ld26uSUb/Fxo0BtiKyNt5SqF3S0KAV8OeX7MkkZLgGjuDpvS
QJwi6B3Q1b9rwj1Jdr42leEIeFqai7M+pgmNRwkk7gxEUX02JkJWwSNHOrfNMbI2TZzit98BFjQr
kI5zAAAAmQGeYnRCvwAFMsyjtjIriszAmlklgIA4eGkcDMsLtvdotdosAJRzuVH7QEQLJ/4urg/+
fnWfiDmON/hQREuwuCMrDCwdw8+cgBTxGZJRC2JLJVZ03fqqI39L+dvtwghlcohqfPvMfyhJjvIj
9kvoMaFh6I0ZdWd4F39Ioi9YTwoqQmdV1XLOctSZZ/Ghy6CbV4Gk704BTaqdcQAAAPkBnmRqQr8A
BClG8CUcnNcVGj8U7nIv1KmADN64ipKUarQ507PkmV3SKjt77imd9ckQ0c1R2Se0OS6NEwfMZHzz
3R5Ea6oVRicV4WZ4szAcrBu2yo4QfqFsJ9gDg40LGfxpBbg/RAqITpo5vDovs6ga4yw/KHA8XEaw
mbsfOe57WxTn0f3zcSnDszLPkXrWTXrjhK04Ojd7ab9VGYOHGue18xRh6DmQBKgHIPM3Zrda0Z8l
bQrwjPv0gKtclPKBzO3GeuClrbVwz8EVoZYqoXKClJyeoQ1Yi2MxZ4QGOYIe3y49lzZkKkMQD+hE
lzhjkIsd7wZXlC/z+mEAAAGkQZpmSahBaJlMCHf//qmWAAGox0PoAVyWGf7dDNKSdEAkNQwssZrF
2nmiYBrXZoRFtWGp3U6Lpy/i14LYCLKykzIcR8sJyJvZG9y/QCv7Gj31GlhgjhPyYujSL9zqG9vN
P4FYAKWREyU2k3X/ff1+1Lkpf9YCOqIK73JSrjhSOo7aNF59ZAxXS7ji9LthztF9RFWG1j/T+vTj
reLfd0j2F/4bxeCdQovYjNSWKVfNUfT/WOqY94WRDi6hqLzONeeN2BR/shtXWEQXjnq450Pn6hsN
1rKNHZVY0FG+Xy/etNcZXKpAixPN3IfLwcfB9hBAQR6jXZVe70ljnPlxmwsdsz+p8MAOqVdAdJ/t
1srXBSlU1+WhjAki6GS8h6oFSZGGOusoo5uPcVlvRbhS11wC51xMPTcybtB95R7ZciB5kVPGzQV4
XOOdQIqBm4JzZ9sKIEyOacciVhfOZ3dM9JYheBDE3Ta/Ck97lhaTurMRs5b29H38Kt/OLwgE7qSw
3J5+KXoFW5LzXxx7Jg4Z5FbaiiAX759CpBzHKeF1493nKlyAuJpBAAABmkGah0nhClJlMCHf/qmW
AANB7aEP55RkeAOEj3wuoj4rY2zUsoPrieE0WfYa8c4xR02vuspHfjrv39fNay43ZggzbOSNhCmo
3XIR9kHJ1VQWhxkQ0Oh7SmTAS9hC4OfEAjtSLuWu5Td59oXuPpim1IJSxnSL0JFW1xaqrIzRcgrc
1UyEYJAbz+bP7Wfp5MtfCPGV00jlfiw61b8KM0w7xY4THa3zIatywcPLemvKD8Uf01JUJlr7sfen
JDquvlo8KJjSyLKHCuR/P+T0rh18PTfBP/nUWX4hRzE4hHlGhMIkVe4b2Vn8U47NPV8dAf60Odpe
YKhuU5uDeckrBDO4nXG9bAffwhaRGTHJRQk/EE9YcjAYPE/fttZa4W3c+/zANUYXRpv62O7JpeaP
V+XaAdWU7XNaqNmfjEQBOxLmos5RDHrBvKRV7KTr450LpJTiYcGRkl1Lq9631HdiNutVpxKEec/K
2kco8z7UIkzD4YBo6zUrsYDIdHu1+wxH2PV4YYFQvkgCzW2M7ShjDA2oulTq8W33DutirW6ZAAAC
j0Gaq0nhDomUwId//qmWANxZf+AinmUNPJV+Zkuu+Vvy2BklaIhmQhq/TpwmrFLi8xN4DXWK2+Wq
KeGREAJu76L5hOXBH/rxnv8J7hktDwJzItEq0mH9FIYIgSCWfGn/dgSVsVw2wHAc4nAY73tnrilr
2UXe6ZUmf825OrzuWSz6RH63EarK5OrKXWHvN6zTbzINC/ISLc5WiKOBbXjjyLQrykb1eVmHpTGI
Xgr6lB0QgjKQfRNLbdh1PvsbUiahztPgRZml58ZkEoez2iNda6of+x3ISa0s4r8S+0oNoQtnTUBs
d1MWRyM97zzKeZG/MEC4c9wP0jLMDF4i5hQaT5Ms26j8nAR/LB9Z7fhK6WPUtM0rfdVMKkoIqlWb
Htvu9o7MbB3hVXCM1uze+TnzGEU1sHw3uqOaO0X3Oiq/6WMXO8xMgJGk9qP9fzfDWlzyjTKXY7q4
od/8WAQ3d1bmyXDJPGkbxd9aARNNOl0/Yp0LwvWEJtcquaLXeN3MZqe5oLhAn7aZY5z4Z3gyI994
b75mDgutDshwGOVVCJ4Q6+qKrnKhej7MoJF3RjAIffidSp4f0tnpLTzf2RKrjUmANFy7pQeFKnbM
vGRWwK33Jzq9xrfbOsfrDInIrzLJwVdI3rEahWFoRmfzF5cYekFDkraeZnnaZAlCK1Z9B31YojAJ
XDF3+bwjaDYp+vQEYLvvs4jYTawvgonHycMu49XkMbazJVLL6ktqkjKsr3PWw3FrV4NPq8u5mxO3
JsbStyuTYYi+FjwTKJIOXuaiT1FxAWoLfLqk7Wk+qL9eVWaiV/Y9wN3YxwKCJ95B0Q2hAJRAmJ8Y
qPnUMFVXRXObjg4jBYAQaR3Yro56Zma7sX2cWqAAAAG6QZ7JRRE8M/8AAs9Ka4mz739pXkoJuKUA
Ahd6bHitFp8bgObfRSfrQ1TWOc/ShBwg7P530u3ZSbr19plfP/kbZwpJfU6xo4f/v/6OnRi9K5r+
uBfBrgogc8cDOO32UNXJ7AvvA6bnuZxhGqQlvfB3uM9zttv77vwaBHB6f4BDSA6FZ71k5D0h+4TM
aV2dbPrs5xXJmwd0oqbvOnIvuwm9LX4zi9bIoFt8m9eUfKW/xx3X0kc6VRzz64yzdY+zQdsIpWUG
kqEWuEfyZivVWs8n2EHd7EvVdOBQcP2pyGMJAJJVGITjsBFVbv7myfv4m2LUZ4YQgWSM0Cp8vn9d
J1gEfpRwB13PgUb/pa0Yc3jbYdZ35Yzg1fymiqBUqWgn36eu19FFqryoldeoHnpZGURJLpcJGNbk
SFtthKenp/jTpO8BM+ndOIQxvYTLBcra4jYL001Zz/kMGpaUQaBdTvapK6Yes2IFWloj3RlvarfP
37ZtvxpUqVd0VZunwudmI4Gn4iL2/AcT4FAzeZ+yGAKLBtlOR8MT9xYLuQO95JS8oZDGQxDYdzrM
B3+euGLwRXEMDQ+PZzxVk8mZ7QAAAWUBnuh0Qr8ABTLMbt7KiIWweUQKXkA2OhzOleKwXwAJOoxf
zhlK5E57Kjw/LznybKcjU5zP6Z279siAoCXMD/zIV+haYNOTcEZXD5/sjrWGLh6maJCoi83MpW/T
wtUjF22OqvPkvGl90l/FuSP3PemTjFw/7DQPR1LviMBOI1NCFGt6f9E71MsscRa9BVdPBaFLfu1j
h/NNRCnmqi8xn4lmOFfZE5sz6bBPy3bXxTKkXcK3wlAwxxlCvXJAE5dJQ62NtgyFqjp2KQiKQMzB
MmQXiUYSWUABNFRkJ8AETBGWPxfKdKXDRVoWxSHe5LKXavt1BDpeCnHIHB6Qj/VonaboywT6Sbii
59s1L+8JKLOo3WNMVMYRtPEjWuagYooeJM6kxWf3sRJpSMf/MO6p+WLuWncgxIgOxZ1VS3cj2O73
3VkM8ztI9cfwGvM3dJScyZXtVeHyMrNknNqeipJXPVaSYJPtusEAAAEwAZ7qakK/AAUyzGtyF+Q7
m5GAEytbpuCjjo4GtzEOWwYyVQ7qxUYT16Sv+x2FudjO48+L88AK6f/V7u2k3TXfVQsJ/JrNOkk1
i/IHntQMA9uq2hAQoL6fRnk8VFiYsMtq5EaJINvUhXXfMHuiYokPmWdDQuazGUme7EEM8WRH5jWh
AEEJTJjSa3Ca1+FQHFENu2DqMQV+9HRQFcPuvHRXPPTsBqWF5yQFr1niJB6+hUhYx+VDk6wggfbP
gOgqjSZhe2PqxC0KCokCtGOsaXhBSmh3CshipzL0+Tpnj9RskTibgyRlODht4STJoSIeNJXoKBom
1tuF43CHI0THtSjTLD708t1pOWuRZ0mX1reIA98fSfTIdyyKma1GVp3ZXCEYctEBDZt0nxXtWrl2
5st80gAAAW5Bmu5JqEFomUwIb//+p4QABHvkbJ8lvvAC1vxmXJ0bdnq7nel0yz1lf+aALH7lal25
ayY4hW8728sgRwcBOM7+cWYpbHoQf1d3AnDPiBWhFfq64BkZi46V8fzFbkSEIyJGciXlhcV9pbHu
2hl1aczvuj52JZ4ayQb8NUj04phKtNVx9Nb3ghxtFzYBv5UOJhIqcjfyywKNOQZHoq0IpTPDd7au
XUzeBTKlzyNasE4ODNtWpXIlkChYsyCCwRXW9/s67SG1ZrkRljfBnmpti2Q7IlCfZIfxGWkfWM+N
3mJC50BLRV9BBvxu6pOHZeoir6jTm+OuxHApy3vUH88sEqF4LzAs63wrp24qdQCmK3rpu3qIhN6D
4wcefrU5N4mloaGUKs2OyXjbDA9o2A1J9UOo23jTEtu0SOKV1SkHP0YCBuSBTQE60mKWV95xgoxG
f17Ef7WpsmsKcO4GzdNFG2HAVCgZ1t8NySl9z3Ae75gAAAFCQZ8MRREsL/8AA8LXninrgRy99TaC
UHBRyDdAA/nuFhsaZUwpBKvdrDr4C80vzZXy5ZxN/yo621jTtLOBoQpQu2VU1zZHKJNa53li0zTp
uLbkRHJqx6KZM/ojxoDOL16JNyjREse3Y3xPQk1KUdqn9hhy2QVUJT3nIudzQ33fxhZfSyHb+ztS
EB70TtBbbOZGEAaRhXV9UZOys3UnfvX5qg12n3IMU0E6PiuwEfcJjtFhh4RelhNZQuwqzTrqhkqF
IXhIa5KCHMRccCOb7A0CatX9AKMovkNfwJE5O/J2YmQ4ECtl+Tzaq4sRU25Eu4qLfFy3lX/bUChi
4QMjugQMuCV4ep7ySM/BUxwIJfBRy+DFiZBHDi1TqBnqnI38VLB8ShFuqHw//pUy83U2Cajlbznv
KBljHsy9pWjZe7sluIQJgQAAAK8Bny1qQr8ABTLMhElGFqoODEIWcwACb2CIb2vVWB3bkPfhoSoH
ojjqBhp4ZJ/6dN/GRawRZD9osvsLaqveGsj7YfOONzDT56bmfaZ1cgX1Gm3gSseMBFUQ3ZYxHCjO
fvc6mJeh4R6nfZhd5uc0GBYe5SfUvXB0lqs4XJ7bmduEDdcPUD/Zn8hMeXWAkMwh8hYlUQy/m6XW
OxFw9eGmM/zVctafEDmMKuNOmx83KBVvAAABqEGbL0moQWyZTAh3//6plgDZ4pbSeNDkDRn5u5mJ
sHUxLw6AEUg4+m8p7Oy/0D5Ga8L1qjDGo6JyKliuEFg+K8pmgRdZOAkWv+Yeg2D98iQrWjYE/kQb
CeIVMvB8HjNv5eUBv/8YQ1TcPOUd/PCuUec6vkgsNEmr6xFgH5yUOrqa9vJOpB2znMf0oALREEhD
VH1YIJpoF5lOm1kZ2POf7eUFit864laYHxPQ7iJVoGo/Pywy7j41wL01yAxk+Z6Yc6EkbGwMvFNR
66XLdutCaqqySIyOg3MC1eqX3Fk/ozDD4CnH9LkERknvFO77HqWf6hzFSOibRaFf7oiCeWs29LOX
FpoI/cimyMFopmr3VyHOG8wVP1pXECeb/jzN5HhxtQBmyFTwvUSeg3K7j4h30kIwJ9NCf+tmz5/e
e+ge9O+c5TCXQ8mhWVQf4qkLvZ8yZM05rlZ5rZ5mbiqqml4gsM/LATYo8vgWV7vMqCurEdKe1C3P
U1Y5qzJmuLMyFoWKIIs2I3Avg8nOn8rQduG5mGQQmMmNdq9EfH83laB4dYmMfX8y7Vta7TcAAAK4
QZtRSeEKUmUwUVLDv/6plgDjnT8sr18XkucZ7oAA1TAE2aCw6WmvVZ9eCF7N9HBKfq6lv1pO40ye
fwyNF8c5KN73YX/xldqsPZzrLWOUKVf4C1sF5r/9/a9qWpk+HhJv+jDYTc/fz59kPT4+cgzCgITc
pLIW+m55BAVMoehxFDucrfXBLNkuTKrCD6nkr8TNeAl4fzWP4kaz7AljcHrNGhvpJcbfr1yNQY2A
qhJYDromWugXRriHXwhMd7k6cnlbNNxsj4Yxa8Z+Z+UVgIToIsFQZkm+m7vn9apg+Q6QtNmGkKda
uj8gMpZFA5CIDPFZioYF/BJ8/l+iUILTvezeni8YP0ZCt6n+LWrVY7eHnS4DZ1palOogMe6gf5iR
p4cpTUl/EkOLZqBgtoMhLWpzXnKM825OOkufIBsL8Gwbj74vw0g6hZ1MPfoVpJaE9PG18HQGUAu/
HVelenoGawyofUPpR4qb5yj1eDGBdnm7imPLe7qqmw7ofbWQx99if2pdKopTOKg9fh92IOCWmndw
5/viZ7Uq4LMNInVjG2qJoFITqTF1aSosWsQ6gwoZAes+M0KBryyIrG2P4B74h6GjMwJagQfEFCpM
BQ1WcHRHL5K6CGfmWUkRPUYG6/h6qu5tprlN0rlwq1npKUFiBg/v6dmktTS3qiIsOEuecR6Clu3i
8pGY7U9NQfjqWAtlivq9bXl9xAnpO6ToSL8vgFWpTGAoSQlcFqwGuekLrGboN6V0K2mobilXb65m
3n8DK/m/aEc1wBiovH9fQhXGKcmglwohOiXmYQvxgJ6T5EvOkO+XDaTHtQuinqj7Gn6IkyEvxLt2
EyhrbrSbTilQyLOh+VOufTGO5EWN1FHmxZmGflodLVvPMsktWX2kp08hIPVuzLRbBGWFZ5epgBr/
QDAraW9FZHJo+9eWAAABXAGfcGpCvwAFMsyeqS9WNxBdlACFzu+UATT2n6OGC12O/RN40HqYb2p6
Ih9fN9drJ/JoeK5aIN8j5CENXTsEGBbdPcwgUbcsB/K9R+xOr6jWBR1ne388cTvGKuxbvkLa4ijn
EmWdJK/mne9lHPg1t8ROabtORmaqtkOrazoVDkIb5ZmekgOxr/2jIFvnzZTBfTNS06y09aL7IIfw
apljqJM1b9iz2syEu/OUwIG0g9GxkkgqhJ4KfO8kLadmsHI1wdbYebqasBu0Eow6BSDQhuuIwYAb
Cn71qMfa32yELB1FbELx4AZumtw5E1+FWvNJRIX8TPts+ysGhHafagPPZdogAj8u/LNDLf6tCqNt
wrT5K/KQpARflhUHUuJHpeDOjM5xYJe6Fwso8IUdLVbmmg8S5dlZlDEMVBVuFNGKLvtoYkV+Hjf4
r7PlaCp/hlbD5VysrD58n2+k2JKWRwAAAmZBm3RJ4Q6JlMCHf/6plgADQe2hD4nloAsPw1DnFvMQ
yo+khjxUedD2hPjuKriBuifqOxGIwqmABKdnM5fYjdYaBuoYPc8+jltxHtCnhP5YHKC2S+keYihN
HpHmd/dUZ8YJjAamMy1/mga0dqM87hoQ0ZnaW3S1T/2n1lr22j3Vf5unLnS4GaCJQz2tD3XVAt1W
bq1B/O74wVy6MT9hpZ5eUl0AKD+PU129Bxq2ZRvwJvySDX9wdWOXFvJfUxCMCAvD1Obl30NwGuM8
C/wKh+WpFgHDNRKv3rwU4zyvLRX+4ZHHzhP1bfDH2+rIwL/VJPwUvehQMSpCOAB13IhqppsNCcTN
HpjadajuRtu8kqG58gxIMKAVt2EbEosdEduwd/BvuQSiJ9QsBSBevesAuQ8Am6oFFPnyduJu+It0
CGF6LMH2WaJdvr/woVVPG32b3F7Bx3MUCQAyxI30Bms+LHoo47KDNrRPoipsQLX/W++0hYGvddfo
OBu+PVHJ4EqEObOU57lgM6AZVwExD29ZZQaXhKbYl8Fn3CP1e2WbenF94ELPepZzQECdcJSYIZZ6
QmydfeF2g76LO0FUAWzM37R+r634LwlkmRvYiSaO7eUW+53WGrLgOL2yg5E7lEcsiADnMncWKBNP
/FIc7P1R/qb6YktgCTbMv9XBgi6yA+H/totLNjD0rsqXnYsfac16ck96p3vugaafmB3jAJZTdBsO
NGi6xPYRYxKwdoub7j9MUtRKuys0YKHu07dmQuIcySuGHy/yFmNB+oIzRFV0UBmj9d1bWeWRH7dw
iyjTNBlqSasZkrLIgQAAAT5Bn5JFFTwv/wADy/LBSrnyYqgSQV3qR95EAGuK/+oNqAuwnzoiKMe6
izHaBylK4DdrAWkQyDeCaZJaPsThV7d4J0mPjKfmw9TGwRYIYAx/BnUtWizBn0+mds4LGmO9t4nX
Bz60hq1Ld1mYractbdVO4EASFiuCro7YEVkbHE5kIxYD9044KYrQDSVH2mKxpM1M8TRx9LUWufrc
adqQbf4XemiIgF6QyGcUtKDQQafOXoFfAlTxEv5s3kozNvZD99fqY+gQEx3yvC4v4Y2I9xxaBTYt
QowsiabGOahPs2QtRAPVM/MYOhySYfWQOx/88OcLSzySkUrNIOeP/cAwyuupq4v9X+RA9iFp62/m
rHJLYdF5vi4t79GS/d0dz8sjxIoUyBv3KSsnO/ftDvUoZmv6VydXUZ7vedz29lyiNQEAAAEaAZ+z
akK/AAKzyJKvpOAA/u320MkHY3V+QJToVQBOhIPn8wJHa9IuSM7lewE5lnLQ8Jm0iHnEujK0TpP3
YseRrahb0W/95rJtOKC7XljUBZ8wzw0Uo0ABtfqH+0uzuXwM+4q7noQ7qorD2o0RlRj+bIOrHw78
FHuIOOj2xzyXje8z48PZjMoxCScPkqGRI08KoYplqRzJQLBDg0vPaWCicW4RxjLsra96R+HKkrZg
zp9R4xzOBaag0nd5/tXKCzpm8eENkOh/Gkl9YvtVj2jcp6r8a/48y7yco9ZLI7rt9TRPM6/bD8GZ
6zKC6QdxZbXFYVq80iyz1CDiCxzXmzED19/3o35gwYd04EPU33nkB4MsPqPnjFVPIAPgAAAB5UGb
tUmoQWiZTAh3//6plgABqE2zkNapACLE3bW1AN1f/riPqOfPw/kt5jnef/JBBZNRGdQRTQyMt/7S
40KCCjI3EZ0/oLdTVIuzUyujFj1B14XjpOqQ6SmPyIvn6iU6Yj5vParOwBnMxeMx1LJ9MAjWCvvC
OEyPvIwogNxVrG32BKTfyvjappmfCgtrlK1xgKLmnPZJpf9IL9jxDCVUCmZ2oFIne7Uf3IRCkoLf
iptgjHwNP4X6QQONQOvMDVm5928UMOEsursd5qb3rgzgEligEt2YppKQIbcggYNg2h+Lm74tkiIy
T0SLwnP5sPcozT5Zr3xUj2rZf6zBlXGWF+EsQYHrkLKAH9xSrdCkzQCUOS8xis9vhlK5kW6DgjQ+
ReLK7rCoL+xrs1lRixMNVLAcwBHQGYsEdCSTUaG6HKe51qMABBivzLHoMLSD5xTISjxmKQvslKD6
n6ENvcKiPmjVnf1Qv8VH/va7o1do7pfybKER2kKwfgRvaThMHFsAZpRzxQH5kHyEjDvOGOB1XehC
yKs6KPJQGcDdrcVlId5qAi+Y5o+jtvEXz0miQ3CBHlUs/tUeRnhqYgCUT7uEKALMk1AEvZLlyTxf
9EIxMJ3lAVUZPDfH4CSzF7xfLPZT3cEImGVBAAACGEGb2UnhClJlMCHf/qmWAAJz8jtnqI6RgCpF
oMuUc8Zr2YtTDFUTuM7dg8Km143vpqyydqn3T5cSjoMoTg1YWRhVwpeK7gOP6bU0tTWqzntBU9th
dH4A9vLd3go1h2k0GHTLUYvHlpIxQXNcOHKd8FvrjiRv174cd2pWxHAfGIkwbJlkr5/Tk7jk2n83
hOBoqaOjmg2spCnASH/LSAjGaqV+KgkUfgn4/ln6JNDKA/AmpU3JctYnOvasQgWqqQby4NRpnwR0
+76v8YytX8UsIJUbgR2uK9U8tBKzuPs0zggHsJJ4s+qS1cQkObzS7VNJdEMz1zVE+c349kGdwnJf
Y4kfeK9sbKGx2u1Anq+zq/tKzSB5FWj8RddQ6QP9Gd8brf565PiphmR80Z6jz1bsEi8aVnxoO259
qH1MIiLd+8Q+rVvw3XlOcEL8xWC7R3K6I0ipUpS13i/MrE2kUkit/rggj2GxnD4w7ORyAPXB5j8y
j4WmTvAjqXo4co63f2rUcgCtFScXeclJSmAp8Bc1iKJ0vD7fzcVoPSCEkLVM3P3UgVnUUKXuIWLL
+tekLCHcSnoyW1xucR0YbViYNgNkpu9p+aQzN6d3YasP3n+K45E825jBTZDOiidkfyAg1b9R+wz4
/gSywqCbBAjufp0HtpgpmQV8+xJV2ZOIKX02G6lHUPD6g+mmodseR+Rkgb830CR3pebjdSOQAAAA
8kGf90U0TDP/AALXTVXwmUadAC3hhme08hTF9pwN8ZuPibrk9r9grTtNvs4lQu3RRfzNLFv5e7Ss
3oC8aaPAjXACJUIc+HclvF2WxK53+M46I2pmqug7ENo+jIiY3g0kvO7tVO/jmL/63fMrCNA8Hl3T
cU9DBN7avZefcv+WYvSA54KYVlBagveOrQUbW1qF6ANVnkECF90wiKABpdBwrwf4tKqLapBSwtDX
CrI5SphMRNKZJ2z3SGlLWdVhWdHqiWaED5PWt24On6XPIBiamG2Mj+SxhRuqc7InxbaqPsg9d47y
8yIT7AFY9i6VQ6q2zgovAAAAgwGeFnRCvwAFMsyMO/hafoZMaK8OcK+0AJd1AcTcs+GkipTlkwNK
VJVyEWsn090smdTwNmQcE1a82xnQ63jWB0SHyXWbJe5afxwsAttGyneIdGN4TBMdfKC886Tchh+W
bJnECED8QQjxuK76l0mrj7bcwZfjnhkECOhszTtRNm5cKHHBAAAA6AGeGGpCvwAFMsyTDx/SmzTO
sYZ9skUwAaFKyXQI/mPay/fqwxO8f6Gy0U1f61Z2iwINny33/FwQd0bs2LoyepZ1jU1hLszTu2wC
jscpP5WA0WMJ41pC0toxBoAm6WaUfyhHN8DKewLAnIktzWSdtUQcSvHrxEOfqhe4Dql5lcmxzpUN
iiosMk0cGszNE5AHm9SPJUT3vbnHGSASEELXf+Wp66lbxnP/jFkOhiF3fFCP4dzFc0QsR+nDOpCT
fyDiiPfauqN6GyaXTw0WE/TQYG88CfFWwHUGYyGjxxvwjt18glxsUk71YTMAAAGfQZoaSahBaJlM
CHf//qmWAAGoxx7f0HABW4IsqZgilFh5kGEfxx+VTSH+av649Zd1G9Ri2T6OPrHBpLpFpVBj0nXU
9c+vv6YEhro4H9x4/JQhukfu1Jp/Vp3vyQNprxLkldRHG4zeB6E8iNdRujipuwbXVED+/p2nKqhY
5HlYtmiMc3r6qoZgk8tBZp/WhuiSM1pYW0F8DhJkfM1j39rPx3jI0Yj/aj6dfLclGA9aTNnAh2Qe
rXQpjJTCoXDs5g9DNxWbEmZEv7Jy177473JJNzZfGDnq4eKYS43lLp0SVPd/YqgPamEKx3/vQs9+
t0ucaT1TEHmaH24+z/liYcO7vPpp5ybAbkUlMI27GmMZFEIFT+zP46yTl1sP0XSraimDsmRSRIOi
Nyvkd+g1FaWaRonIrSeNMBE19sPk+QEqMzMj5ujyLta2In4D0NxM+/LAXqXiFlxKuUFNtvQrRF0Y
eVGhr6sP2BYX1aQF2sfrx0gikORW8nptp43MIFrauPYPzuGhB6L4qgtdjfO5F8pNZBNQlV/U9eOh
jka6juTpgQAAAohBmj5J4QpSZTAh3/6plgABqh0I7EgCCJaVdabWH7EcRc9xgXtCQdHz8x9Ly+sV
ylOqzWWhbxB0umO4ys5qSSwz6BmD6qifEIr1l28xEHiC1pf84aQ0rp99UUjaevHm2gRqqbplhYpg
5t06Z4XWcaAq1V51zJ2ONOMCDDhtVVzIu+aeW5KRDxsaWdVr2GTlNQDWrfDRkb2RHtUhmGHPzEVf
WymDhHS1SUdYW8YYmWrQJ18tboaaj+g/r02+cK54/v3xTPDBInfeuRqamT50E2OWbxsL3RXO+8bl
k2S8csH2QR2NJ0pPchFAlFX5QqwqOuwGLwp9h5xyCVp8CdxZQAMCplsZr5D39di87L3LcIagdtzO
pKNshog1NtdkKt5Pt9gCd0LyruQNOyM7qepTPXjVbmeW3fFaGU135gIGEQZlwdYqO76FJfbTHCJS
/8LV5CeNMQKRER8lW8i5DMaUy6FcOu4mqkxeyfJfzKPaXtUPaoqq4vZaAzbZ1VzrKxft6lwH7wYb
vodJQYSablgpfFX1GCO940KbhwswhM/tJUpZCHZqFGXeYA08ApwK/ZZujNDNIrxooA0M5Ice9lTw
Lh4KanApFlHVxd072f+ew+wm3Q8539qiAFo7198RV3hggCjOe764ca/hhFbdeZXvFcgJZIwvSMfo
1SQn/vi4cK4YCv8z2niFCJaf9pH8IO0VbU33Zbp6nZ7NiSeGOuVTOWaydeLhwitcTGRChSjtkmab
W+nNOo4oIdWSrlH3Ezj+XRjbJYQjdwnf8cR6/7cCwQezWglU4tSXoAJXBCPv+qUCKNt9ky/WvSHf
LOaaEUob3uSqWddlGsX3yqmYMSx6fHZiXod6FHiYrEAAAAGbQZ5cRTRMM/8AAs9KfnArwgEWQ/pg
GAEQdVI4GVZUiP7IoY2UFDLvZ1XeIIW59/tBhL50UiIgc9W4tptpVzNv8urzh+v6vE7/wWb7nh97
lGDHsp4k71vk5MJKp0B6CsWOwbzQHJgZ+belYI3m3krcKx7eFue9pZXZs72GZdC2snmRE4EeZ28P
g//e0jkMCUSz32cRvtkE/f4nM0ZqDUurc5MVkI6x9IYI0xz+KF8YRhxaZPtbp0vXul8oiM1bQjq1
a62Ujkh9ROO9fSDuAeMiJ2u3rwUFHJqYlYVPHEMVd9g6XVf3ODQXJolICgZcQwaGWMNLp/kSYcVR
Y0ak+OEO/8zbsnodxEEyrINqxLjLRloLU1aOAT65/6Gjrq1H7YXjN7DhG6ftysVC/eVKQAhhdewN
PEOjVf7WD1FTxZlwyVDYjaT+BmSOaSD4dfrbQd34u39MjgaEXugtXKyGqv6e/luaszUo9tQVxrZY
Y+7t6ePCD6/GS8GIC3NF5QTSdqigGZV0eEzjmlO/+yPct9L93/7bgTYwp/Gms8t5AAABFAGee3RC
vwACspa2jlsIATLHZXYVeKMkxM/1D05pXrf2OwmdsTTR+6E/FJOzH2EdkUgA0+4zJuCq6BpQ3zHp
IlKurjcZ6KrwLg/FHEkTvKNzlPk570lWHsrR0Wp9CmFsO0EdVC+7dO083eHygr0pHuSj0LwPg0li
xW4+fJISZcmmLyh3tWcg7139xmgV5bwSSgpsC4zlTvcrT3oWzARa97AldJcqqHiwiouyuyoi5MO2
fc4Vms14lnzXgavgAH4uGmIAug39oOChPqLh0FOZ2TReX2FNvgOSAz9/2SCadQofZmNQmevjETNY
F9Rp82+sdtVO2bSN8dKQISoolA2Eg1yNZvhRs0+ddpCLaanrEPIcG9qdgQAAATIBnn1qQr8ABTLM
ivfkGi0AJtOqvzFzucoRizeMA8sOM+qqjrBcE2iyETkrsmM/p5f/nD3bSbprGGOFJaMOo/reJOXw
v6O78tZdmCvR3p9PyFUqqJ88rrV1j7rd65dS/QR3EDPr3bWOglAVsbvRK9nj+Jhz2JA/y19+Nh6b
a2rhLy4puRD7pVtQMa1kCJTT6JbFLIWha8eUxAlWw5iUCROBAXdyDchKFVLRUnXCvpNnnsleEKhl
com7L0XfukeTF3XAWw6G3bHnd6aW/WyAHCQu/WbGveZ3HTs/g8rkkdD6LBUV28KNfQYJ8pC43KBw
rSHqC+RrHOjxqnQzTIJqbObe6zNmM/kC9NlN2ln1HcE4KxfybBu8YZpWvktkUkhvoAdueou2netD
VP+08dnYFtg1M7YAAAHTQZp/SahBaJlMCHf//qmWAAGeT5IiOaSmlPoAcb4ahshG/jq7pZOKqluE
uKk6HgFcmNM/BBzvX/F94f4Ya4EI/2zFf9Wwt9eRTV0w7FR+9OETwgFpIDa4fyqPrin22V19f9J0
OWfRY1ewjUGQYh89jxbBXHwmpiAkliN2kNiCoju/TTBWaj16NkTTO4Tfc/oPe+SAgHTcFjT7P/Kb
acr3b55FHKpXWiAqGMQ9LVY7gObhE4uUoI422GlBMde4jzawa4oNJXMLq4M7qNb9M58TOPpzKA56
nVVnXO5UH7ZT653XtYGNqCn+Em2KNIvkU6sbvOawA/n2AWk+M69RGOvdD8/cutWP21QkmrL8dkC6
5HN+fJa86uv9r3f/OQs7bMmX7eDNdY5hJ3ZwCpeyPgbRoR2bfwdtZLbq6YJJmJPkB6ty0fo/xpDG
1hdvHLEBJNPEg5aXwg8pp0snQDiZdUa5DU8zNaTKk7cgJf2Zj39QYDwKo6c9ASPwigeSZQjS8GbU
P1yjALUmLyT/SRi03qbItIFgPwqgE6/ziMQhvyHezC7lnTnKdJXgqgvof1f2Iq+C5p74F4ZkTTqJ
+z09JkYPgdjWihKfHfucu9H5pa4Vr4uBltAAAAFlQZqCSeEKUmUwId/+qZYAAZ5M4/ZHhllgBbvf
SAM9RRupW4EZM/uXthaXbPdyAnp7AUsPvRzNf7sGuYMkwkiSn15XKozmTSW5PfVyRn/x/MjCnpyu
z/pVyJfsq+CRZLUCP2WrSKrbJm/qUFQu1OpG7NuMcMB9HALFzXmOELN6a3NQFGiK75FATGrxiiD2
1yOeabjVGAmQnaGGt09xYDJsn59vtlAW1V3rmyEs6/mcP9vcxd3tVP1Tl+/hXJrQ+JLlfpN/SQEX
yuV/kXZqMAUfMJ7BBgS9bhntbpl5bBToanajFkZ7hKl8jZzYQ04QZvFhcg3cY/Ub0gaM38y2WlFJ
SrXHyh775/UuFZ7Y8g5uEUQkntB+bsOzsjMZKZFJIH102FEpVvbT2IAvDcYDQPFFvQZYMZHHV87P
M25lV52nU0d/EEp0C6eYyx+Bn9/TbRGq6tmnPBY/MTdGzbjm5bsgA0lbKXVhAAABH0GeoEU0TC//
AANMHDK+WXJMg+BfPwAR0GsA5K4MMQ2Br/vxrvrFBj0Xs7MLb8nNZ0kcmhQ2UZ+d0FurWylaYCZq
3n9DscBzQFyBwjc+XtBg1RXR6DP5aEWSJAMAbRMvl5yIcC/XHiuHDOhPJBeSEDGArzgxiVHUw3Vg
kgwnpq++GjkfHgt+6gQbuGrWLs6gnY0+ryfwlky2Uc9qCCIkrpwsvcL8qDTCdXehdnW5nT7PiHN6
OK+0hsgSoaIh3BkeflQZJVJL084Ew/HTXQzz0hZHVGGwf5sgcUTcVDK2YDLiXLTHXLxse4jSwDHU
6AtuWCpDlsSGICwd9M5ZGQ7cEBuvI44kmwunHXEjqKtOlufVsrpIaZH24moGavvXW89+AAAAzgGe
wWpCvwADqwx2E3bDAAJlbNbGMuXVULBiyjM/rRQWciy6ql0RLauVAfd6weLM8/awosxTiTdu4ieS
i2x6/iADVlIQ/Hc5gf28uaPRbGRWFYKhpQdH+eZ6uZoDt4/3MRFBxK7l3Y1FSvipHAjmE3LwWAd1
imG5P3j6EtE9nrfAX+xYV+YaZx2TkDqfRILvb7wQUbrtdt5TAnWGO/K8efcEaV14dSn+YFX2Fwtv
aC5GrswJsSOvWXCtymI5ejToBDTNl0zxgpcnhLzmOtTBAAACUUGaxEmoQWiZTBTw//6plgABoCu/
HtWeOmG+wfCKLvsqCqN/UCV5k7Rit2wa1C/bxu/ozwpvfRj3iRvk20/D1D9/tirHxennOgxpA5tN
4n96GvSfGQm7xX+HaevWah/44K67mXc9ILYFUisFEz83mXzt3axE0DOfFJgNEc/eiKWUME5m2SKz
QMaU2aaPEwTL/ZwZ0h/nnmkFoF3n1+O3tDl61miRsl7Cmv6IKitZZCkIk6QKkID2b6gAZmVHfscT
mtK9YNwfE60GYX8K22sM5qbSulZjMUtTv/mvdXijzhh8x/QGWB0CHwcGUykSRtlkFwN48lkyn9+l
WH5KwzzXwcmEpA1lBPEpqLho7UhGMsPHTLaoq5/gG0oot7RRkVZyxQx8GKCNt1F8KZFR+OENd69b
sRT9M2X4F39rrTyqXDMDvjO5UyKQdRRF/PjySDlKi2Pjg4ww/lr1R5JhAh2dpJqnHSQJCqsNfPwJ
1/LPMz/mULxu9dHTcmmxZF8rtBY7NmiuIs7IyK+7hqlriwoFkYWlIsIjrnnQxiA4WKakJkK3vEb+
B9Ubr24E60EDNHHy9mnUs3px0sUXBgUkCixMLyh+kD5wO5dc9li8kesbDzO8/OfsZwpuQgyRxYTX
tjs7eEFkIsVpN9X89ahYEVpMGArmkT4PSozxAzikyKlNhSPH44E1ss3Tefp6mCtfvhf6vtHLdADx
D1QfY84YhFB68BFnXh0skAjbbdRgjyOEy4LsmdzuPQ/xp1+qa/UUptve173FQa8OK5De1c2V7jYw
mQ4EAAABQAGe42pCvwACoU7ygAEy9/w4W125P1RiHTlfpsEZeqF88+V0+pZTozmIiHQa+tET+5M1
XNQGp/5FG5l/i/er8A8itYUajbzCHhnol26XmeYja147Yehn9yk+sVCRH4Si7S/dtH7rUGJ5ZhAL
EZCXqhPIBZpgVrrBFl4hQe7N8DoiyzYM4WCr2/we27rh7maLNUrTIZUXYb1rIuaJXEM65e5z9cG/
xTwjnYH3QIpR529qfwhFtqG58B4T53DrT97/R0is1NlUI0Aec8S9kJ3Px9+0N6Nwn2b5YW9TJdEQ
AExPH4SQpcdfXFNNBGD2DJ3sOTLhlwfk3U/G9RQ33KufyECRpo1Wk1+2ty4MIufZHl0KVvE9djBt
+YVzvihr5nPOi9luLp1yB0Vj3j4QhhChnFpuJe5w8CbqxONy4r1ABvSBAAAC90Ga6EnhClJlMCHf
/qmWAAGfBkZS3PHKgA2+LxDbt+6VPxoW0aXCpHgRmEOCey17uXL7YufjrsFA/APcAT8p12gX+/p6
kVeFX2JcXkqCwo+XWCZ2QVBpvBewvry9T4EvBv1/0wDyfRqD7e+925IZYhBFREoP7vbUzXoVu04U
NLPxkYH4iBUXYDkZxki6Ub1U//8gCZ7+91nniSLRhLGBTDQfTxsdbx9G02rhuqA26vaDnyHzAglR
x6TG0sxzENi6oTTUhRezbSwgTtPGLMHpccOSDFxI/I/FRHM5Vad8DearMYFDvFbZKfd0Z8ZbrUPA
BvKADuFdN5ibCk50M/gIh8WGG5mCM4SLY+9olAFiNJX1ZxogQ1rWu5QVvu/v3EOIRz6pJqd95cZa
BEDJppo+fvwH49PJ1gsEHNcbdYCxttm0p5czZisetIsr9orJ+rrWRG2li8WmWxmfCuK2Xm6OVlHM
PORqCpp0ceca+VBI+GktSjQvubkXsMY17Qwm2k835gpERxqieKq45FQ0as4/e0tXLkpJdqnEAUHR
E5aJIKItLUJqcM4C7+j/7r19nmBi2dNDGEZGTRc64+XSQ5rxJFm3oWrQmD/Z19BnOoQJsuj3xlBW
3UxhKf4m4cA1QsSr9Y81MRYpefZ42/ML9pHKkB57QBcqOvba5VjyxqvhNfoYO8YlmxpdzcV6QokK
wPBJdXy/waulXLbepOLx3LW2q9CeWT32cFivfaaaED678U9IMfMcP1WM5/JxawL8etkea7ZbdygB
PjxYg21xekRuvZdPxFzcKGc0D+5iY767q7TSzATzyoAGqfe6YYo7gUumZ/0n4wWNkm1pY9Ea1P4r
/UPabtjZNpB9YNmAXMrQfxwUfs5YpM7eu76zuYb2Fz1JHTQj7si260dH+wtgYHgdJNa7ByGVM16b
RwfPYsvbepCAOIQiTXUnNQHfjWz6Kk2y6sA8GJAcxlayO+Jmu0h0zunqBGoo1nULn2/NoXlAdXS5
9R88/beo4QAAARBBnwZFNEwz/wACK5B6PSCh9RTPwBh/sOQBJuEhwUfQRd8AyyAATKhU8ZClL1Oq
qQxe1hkTKefKQK6xRgoeOQcYBmxx32QjKJOnlQAfocNvZ/aDFpiCIbMWC/mh6VSA/Yi5DOqLMXkj
ubC7ciaq4jfYcEarlNjVroyiH5zvlSvpIXx/gUBcWNI0bSD5uuFVnl3w1nAu3YLFv+qJS2YIdMPb
spJLqt01zIjnzlwRZkYV2GtOOsR+OVRgeMefa/TQQRbhwBBiTesFLgrxZswAzwVMbWWCCWpJg8Oo
fAhw7Cp37KU8gpn3XVau1KPK4bTjM2/H4SeWaK4/nkL43AWz+9GUvNWgY+5wMZt9775Pj/SakQAA
AYQBnyV0Qr8AA5/A/BXT3tVsdeAEyLrCFjq9LiE/sANTk0vdbFBtWad+mxAdnlEB7RS8O6HAL4xt
GftHz686jX2IugQCOO5BaIAVcoq2zDBhzejE0B1yvlme0AvaZZ2I+lKwkHVN72xBFzFgnsdhccE3
Yuz+nl3s+4HOQ0RCM9hMHBx3J0P3ifuXV6+i6VvGkUrmariXhRf1fvejlTK62uZP1UTCegg2huon
Lk7CWrBzpof0BOSkPmvCBrmQBUUevI56QIvMo2VZTEce0jRrkatHnkJnNfz6QSG2ReEpl+K9Gh5g
hKT2vVeZ6NZxqvhdzpDgP83ob3lXliW0Wf6EKnQYcflrau7qa0r79NyK0zG9m3FjG4lNWyjkCH+G
mW/g8tzmtOR4OMEFRW33YF3QqPFxNCMVku6wNu/Mb6//qWlERWNoAL3/TUZx6wRAHwdJei5my2AH
vmVG+gIvYACEI+7iQ9SDDvcCkUxbkEz8etzcgnxxe/eWK/L7DS8M126S3W65mYBBAAABVgGfJ2pC
vwADoMsCRHnf5YHs/AB3zfXlJSYTH6ce9vu68gSnti+b9GSPEmX/QNs4qaTzCXXw3Qz7UcS9df+7
2GcNYybN2GKxjCoKRsURlUkigZGemyRQU180+KGQRaWN6A2XMdzaeqOI1Z45HDJqIEukkGDisvGw
XeVLQJ5u88TnZMtFb/KbNhibuVVby/r5NuIbR2Oox1kqQ5XV3YNh18Q6UVGejxWelwEK2OrZbkuy
UaTTAuHP6CYpJ/cZL7rehM4MYXsY2ZXvKRQdm5Q+2W6DABrcqCVrH/EFWyfy7La5ZJm7Uv/GsuYW
T8s1MET+z44UbLYC6Wmhbz/ZWs8J6bEnw2yuNAzPIEsqDlEPzglhjB5GHSgnLpjq6w40nWrcOIRa
9bgGxP7JNpqVvD+3JJdU3ByszMB06SdUhtPx3IwtF/Nw4Edac2uOdFkvGeFUrnFtg1KkgAAAAnxB
myxJqEFomUwId//+qZYAAahNSCFgA5oXdt+IWtHCt8UoN5KfmwFOCHkYt9x11BrUqMyH7jbtOIdV
vycYx6nKKQXXnuAPRZqLK6fgYwWHSZoCa8A0hxPYQhnXr6Vt3+892XWyX89gxco8DfQNujnst8N2
1Vijx54g06VXyJDNWFGrFhLHHMiYIIXWHQcyFObdJTomJAgyKQ7b15nDk+ltPaFWulpG9aqCagul
un+B39RFZIhuCAxKBXo2QlWGUrmQermmxE8Rwh99RE/GlpcQLoTD3Iqp87ZoGo0TxEqXi56R3rLL
BculwEXCQiFOe0+NxZ0TMTw05kl/zgVtH2cNrn+ss8ZN2YahKRS3xK31aTxicboCVQ+gmsghbe6i
wCONO3fKpKa3MzKi6+f/YJ9WTZ1sXFMt/TIIuP41vGAAySvpBFVGMVQVDOpe8zyS7+bPvZgTkR6M
3dvF7IAdO5m3LKu53TpaKSORwD1UTGrsR8U2+8hkJH4zhkvYk8V+C9/OCDX96BmEP0KxUScxegts
6uLfRjQ7TiJoUIxP8GOl8NPiJCPes8SXTqpRhUFMnvASWHtxrYOOjvJN6wlhKTUcUv2nxGL79vOh
KhAPr0o/J73KslE4MSzNQB6mVFWgG84cjtZMcaB27W/8Q58f7IsTDwnut8NKqu723pHn/KwxMrBA
H7VXPsG13tgswmydQ/l8L/+Kq4poTi4/5KQpwxygdRr9I4ew+GnjlAW7otw/atdD+TZsVq6UFzii
5L/NWWw9E4/1yvdwBtD8kbBH9ZK+wxTduxB3BiC9t1uq+pYP4xoohtpVG/ihJoWfbDgK5Klj4SZ4
xS9KeBj12UAAAAF5QZ9KRREsM/8AAktx3MuYbfplBtuI7xgBO3rwGsJU9N3h8c6bIL0arwm6382m
BfvKWahpbcIZqhpV9AHcjcNM3cgRbHBJ+fAjmz0u/1bhORgZjbUmYl9x3rm4Dgyyg2gnalztSpEP
aID2FD2x1ZREFxGG4xwRlvj3uNNkJYQzewzkdXM9QCtwsaswy7R4Xi5VuvY3VGS+9n3Y58MVfgUI
zf4N1NJSX/S213/E6aS39N3V7czPq4zF/8sFm5L82aZHtTgTh1MPTPK7bCA2X5xBkXlvM1PY7CU6
f+MoBr9+ly2dCYdhAdzJcAwQx9Xo7WB95xXNL3b3LArsruD423/XCJqW4Y29tajA15tSfu2EubTb
3eliiXCfs4HACVwKWnSscGJWyt6bzf3jwoiL7MkTOj7C7/E+69EJ//Vp4VNUChxxWvtVzmwkB80O
1D5Ya23x0371Q4bF7GGHmpRvPOjuxD3uS8baN2lq4pzQNvZWeK5dQY2IOIFvl3EAAAD0AZ9pdEK/
AAPLwgYZIM/BfahK5IYPweADvlUGnu/H5dKOmbd/ZHst/uZroAwQnlq8a/SEl2A6jQ7HgQ1NIbfu
4PvwjjF97r0B17xL1uTeP9Y8IJgCX74TovlEnjv0flTgzIkhW5U1lyd0x/eFuclCl4SumPAM9PvD
8TfzIHxOsUYlUAppEA5JgxdbQlpRUcQ6ujDNM6Vy4oNihmHnfxKi2kscci+mD0qcmV72UMCs2bnd
kcaSTpoGxB9zJoCZ89ingqoPaDSBj91rXIlJqKP/wIXyeovsOUSOnOW+wsiE+oWlONmpnLBy8Bue
GtZB1o4fgK2vgAAAAI4Bn2tqQr8ABDgzwCTJqFtIVUAJExiIaz1s5Z1/XW8l/4bw6e2xErJB8U/L
SWfzwvWe8gD6+1u+W0FnPzuU+3Tapo+OeIqg39COB6tRcYLIAowTI67pqwkwaEXOJqIexe8VfmZm
knePQLOjdRlGF551cpz5FhLR7b2gQ5VVPbb0BTK8P4RiOuO4u3Y6maUGAAACDkGbbkmoQWyZTBRM
O//+qZYAAai7/SAIIWqpWdIlsEWo8yYJRLrIFEbWzEWPDiqwEyaKH2K2+m/khmDGcXfu+Np7gYzo
r2fORwDmrf6+9fUJn0qBxfYEAa7iMoDjMIvwrawvoKgR/pgqbGcKlKGBpIZfvGmOehBeXkkeQa2G
SEXgZDUrcBMRp1O80f4N/ETkUTr8+rsjS/XfVj+PNlKLnJ/mymgH6iSWfXapfHMScnmvuMVuIcV5
isfbTlNW9M8W7rjsM72POozKvMSd2Pwq56FlvCJ+cinLlpsU5pJoHf0Dehe1h/EYNgA79S+NadFT
7NklMRD8QAaNBLEqBu6UbRZQ7+yjq3qRGZkCprUZNwB3SiyyKosFIuLZ2BdtSlpNPYFlARi4a5B9
0kVUIPxpZ2V7LNxTftqfeeWffwDrZk0H//Fyf/D2xBTtoF67lktjIowvx0B/fxgFQuJ62RyKNDDS
fHzdPAh6M3SsCA9lXUha0zG5Xf/UG3WW6QVmrUh2vBcVM7HQwjo7cmRZ+8VzU6ApFFPif44qtP6j
ml6RL0fzoIe2iaqvdks51QPrTGY6OeHOT+to+9uY1c/u2oh7OLdnPbZivwFjlmvqaHrtjjkFP6Hs
Nsi+/KkHsVXtET9jZ7+Y0fShFx0s9Pt55xbSt7BGAxBTNSDbS16GAcj952bWW/5TyzeDQ0KxDpAc
gnMAAADUAZ+NakK/AAQXXjq9JmtIu1hoRxWwswmkwlSO2eHhABfL2Od6C4d+e1h7XUOodVABM4gJ
Ljoq2XPhoOYt20ul5fg7YVPc2Csq3NJWUagKDq2xJUwEIEhaSf96NIEqQSU2qRGwLlqk8JijeTsh
7SFmVcuH5sR0dvRJQxOLJQVmwVrCJzl2XJNhQ2qPvcMBBU34nn4xpgfpUrPhKjREL91fKiaeLqtd
QyBxYbhqB+mQbeYYjWFnl6+ANbJht2ozWNLRmBvrAOvNIcu+vmaFcvgnlElsJ+cAAAKXQZuSSeEK
UmUwId/+qZYAAajH/qgDSzEYHaZfbhRgtbWhDge6nDef5XnySDGLP5TMzHMFs2UtkGj3t/uMAUJE
f+gJ4Kr+7+PH4F+SUBZNugS9AKIO0hZKQilmGJWb30BZqMC0z0poNX4MuYkvkKQiGqNupoRXqgBa
u88OoZg/tcQLSKd0j/pGWOJFgFVbMwhWTKEVl2Mzuhgzg9tvDBJpX9qWf36nKCP/9f5rmqA4Zj1y
UA9tnpVxluJmFzdywhYSQxZcQy+ii1aDRt+Qb/tNGs7c4VQBmAjsCidWPWJrq+B2Q+ItXPcMbhKc
qnDRxV8Zt9i28TSEN06Im4+xkaMzVqzetcOUonL7nNCdp6jpIItbcgnz4/fT6KmAWlvQJNZwg3hJ
zSlWvQllzhB3PvYQwM5YYuizgLFqWD28BR8sTUxtK7sFYfNDEKEhiiZVzDMuhqwnOAbZhbdRIe1k
dwF2CsLeSBr77iRYqY0ypAD70laNP31xKilalH09+8Ad87MiOa3X0viXtXoJSiKWpoLajCZxT+ge
44JNV5jZ1Jdym0bxiKq2AtzQfGf5TE913iYbNZ1akaKfrIrmoZ/KDxLmZlY2Wn2CxbtxL8QTcrQH
t+mB5T0JGu2oTS24v5Tnm/DzmKFnylYTT7DWEpnxXknBhqj2rKuflnrqGom+El+DBiX0hvx2uepx
WYUoYGpleWo8wq1ghv+vT5qXiKIp5TOHu97F9xx84v1RO97IgssPssRbrgZUNp8xQatpZ2/7kGFj
xsqzUVyp5soxtprem2QHqqpCrGBjSl62aCDL5PgAPz0SRZ9jYbUpFC3TxgYjoGrcrqbxJRB+r/bD
qZI1XXehmSHxeZXrkVfp3wHJ4Ay8vccveKZxXz25AAABhEGfsEU0TDP/AALPSoj9OM71A1bQJyfY
9PBlYngAUG/G1bHTY3NUmqkxaAQqoQuvdIiv+d/ulbxI08jAr5MucPKe05zmYjY5Re/s9rO+lF+t
UKl+hiyEn/Lpp87t0Js7GyGDryQ/9co4pkjsMNsINd2+1d/8TDLytTpbhH2HJrgYe6zlD6r5FGd+
8GAl9218Y2anFIoJMk/DJfonVVfIwdkozI8YQsprwKE2Dd7SqCZ8CVL7RaUG4pKApsQT1an2vrEf
bxCSjUqoCoUc0t9J0knERx5JNRs3yYVlHNvx2sqvQNaAnRC0Nwg+6zkWeNgzeRBA7pOR/OR42n0y
m6yuM7sj+HnRlQqal4llG+W6/unnTY48I++eILeIZO/sY8SVRQcYx83LtHw/o/uv1i5DTMoR92T2
oBWdRs/faaB2so8ryJCFLOZcOyYUy7hzuWuvXzHqcnU47bTqkXScV27e2rRtmJhWAncxPyAzpj/L
kJoxmxLokMyrYACG8GD2AHewDW3zjTgAAAEbAZ/PdEK/AAQV0YqSbaC9gAzquHr4WVOrtMNSYBLx
K0mbsWvpYI81NM694ri8VZUi3+lNB+8sGcExuhVI6zHZIxhvOrTeYkFmqgB3JQrRBkjgE/xC3+FK
KltSo/ANVeMypeyOkNJjtXSIGwakTu5ty8I1DmnLT+xOHvSSELmQS2UI0kmZKlXY0+U1FqCkVBeH
3xwownJjSwDGmVJeYDbljSxwO72UeeTxplwhg0IgUgIkejrdP26CS3RIidGbKF5ccGPMBAxLcUck
DRO7JJO9/DIo+/ujIDV/gQA6tq/LeDxmm7eTcuvrpiJvZDXZX6NC/Un9jjNa4bH33XWAYgnTva2j
+1ZJrUn0HWQEuG+N2GAfUbIGW0n9pJxRQAAAASIBn9FqQr8ABTLMorNf4F9xuRgBOpGUMbBn0yel
Ff2G0oduMXcVhMLlzDv9ThNP/Sfq0XRP21WvQHgfUFfnroqX5p9QNjoeu1KKgIXt3YNOQBh4HKD5
ymdLpNpROcZriuuCzQWlwAlriTGNK3qqnHdHWPXY8nqS9cqjEZmt1PnbhocmZViZCm4sKQI9KChB
6Up35jt2UpwVSwZaidVO1tSaR5mJxD80sC03dkvbfytJ34k8tCnBszDLkeUmpg3rgMYeFdvn5Tjc
AGUjnghC+agqVq3RvAzGAl2WUwP156FzzfECNyMCRTQSbUqWm43Xyznwznmtr/r9TnrZ11n4xuPU
nFMSSm9xIZI1Af5lShZLZlgtHoiGh/BIERN11JUqf2ZboQAAAbBBm9ZJqEFomUwIb//+p4QAAzbz
ZfgA4wicl5SHL4hqd7lu96KzWig7frQ3mch06FHThfnzACeNXqSRnhfqFFM2m/uzhC8ApHzgPvG6
iBCT0spuurQ4Vs8SweVgzw5CUbZNDI70blTe14fJmJp9fMzZY2JG2p15bI6KYzm1jeviXhWEVkbt
H6M/nHom5mgP9bA+FCDEPURmPoNe+2p/9Ei1t9ZwxXq5JzE69YNZMHsiN2qhrwsdICw8WPEnFIqk
MLFZoYHTaYzEDgb6JyQ2asKtaPi7FxDd/sZdNMRbkxeV01PuOalGNhphqR4nzO/eg87vlDTqIRKJ
3l7CCOpsZd3WWq7CtcPTs+diRR92D9LHdc0fMOf4nTgZEsCh/vVquvDEBaKr8RtE8iEpoIUrZCBq
pV5z1L9RP3BtqvB1D0y2Q7Xf2s/PT1Itx/quqm0gBwh8DzmgzSwNRTwpaxWII5f4Ldtkd+OZ7mJl
CImA9pggX99xPrmcC+HchIAWr/utO9GqaFgLXfZbrx3ImChMlFlOiz4ZQk6HZdpiB/WiGqbQvgHJ
1DhMqK5QE19C6cHoFycAAAFmQZ/0RREsM/8AAs9KiP9Y2xLXEiqoHaBFgBOD+Ocrh+HBqs42RXA1
hiOQ4jPZ3qQwk99qcOC/8nuggNDIcuwHAR6ZvPsaEUAWspKnWhLSVAME/4Xg2+0Acd/lLQubfZJR
Ibdt7dncfNQXUm8WDh+3HIYLPCTr791WHwUyRIQ/thicPd3W2px/ahowZ9ibmIgihBqxRhNGkTvf
+s3YmZUT/Y8Rdf0UxF8C3M4M1uc5A3MPz5CN4SyjdDubE1RgBhWt6l3uK9nJoILZ8eqYodvCvAAW
mt/pIJdTx/LQzVWi6E8bQHaBQYfaRxh/Eqs6C50BP8M1DCtI91Sf70z0ZH+YhqcshOLSEyyaB0m6
InyrPeJYkR4hVLq7vtSNDY0NZA0r5kEJVfJWYcekMk80jB9U6ao2OKB720I4qTF2g6HUMu7VmCQG
eWfJKAPdSyK+zLUHEHLGArLQZUvN6IV2FfaiyFoPqfVPmwAAAMcBnhN0Qr8ABAlUrqT7z1ze78g6
AEh+0S49Q7F5H88O3JKMUSyoonMoXp0DEMbFGiKNgTt0Yr9VLyZ99Cpcr+Vqy9Xi7gHGBfup3jQt
PuplfJpIeTPd5KbQCDKCGwlx807gr7BfFg+3kBqD7Z5ZUZtR0YTS9NkDst/NtWwda+ljiHzJHQST
8Ar8D1IaZIUqrvX0jORLzS96abjjrT7nRBu7ETbwsB8eRioCyI7ii1iyBd5HwwDT8aTXfnp2pg+h
KUhwXfifCtXzAAAAvAGeFWpCvwAECVSXcQAeoZii4gBDYYnBlGjJ3R05zIRaqkrc4TDPWWTAz5L0
KpJO9QOy5taFvuqGAbo7leAo7UaE/ApyOQtY1vqDFhkRC9VfpxEM/S9dkIQtSKKdMaSnYuB0LCOm
raav54S0PgFse3rVSNNzRX9mgmLTMI4nYTEDzcJOFJYwoAl1EDzpBvvMi9UhOh3HRcZj63MwjNsR
04DlDk4aT1U8cpkxnGlC4gd8bNuepGP9bZv+N3XbAAABXEGaF0moQWyZTAhv//6nhAADNvHx0iSm
qhkYgZQKR83xvfGWU+OjQGJ+retyizg68di9kra4vv/7QcdY+XkV+XjV7ZWTFySQjwVvsqZnE0n0
x9A17vQIU34p1pcc2mOoEbVAgwTsCIJdw3aZCCIcU+Psl9o4gpXxYKqPcZ8bP8uvLnJtb0mN3WBd
zxbdYNMLqzWhwKYlZVb5Lj4e8XhqkjAAjPZEulmI7YRCcno9gGUo4YrESmijKFGZJNgiMdbkVpci
aGcKeLQtdZJIo2iZB5AgmuiHVW15IVyRqmk36FrzTt1//CZQ8r23S58vboLWXwt1pnXS65LMt6uF
3aSiM4Nx0F0Cr2vpikwWHHj2YczFxyua04Z0LhVzZizVqmRMk3OSqBVvrg1H4kQ0XMlonqEKbEsx
4/Q6QFZQxTjmioVU8wipZyEseSQkUIHygsLKkR+5diJ+cpd5OBOpgQAAAexBmjhJ4QpSZTAhv/6n
hAAGd9lRCIFMKOWpAC2TjtYLUHItZwpBM5bKuVh75hmnoPzsyUexwLKir7SmnVYM8vBWfKaC/nB3
K5Ti29kILtpFUMDYMsJYkMV4FExq3N+DyfFhpsL++3+GZ2X/8RAMG3/1TPurBL6VNkn74hLMich8
5v194blrAScmH1046JtjtF86tk4v5yc58B8M83YFxd4RjSRAfev99ftd0aBzp/l6SBLyTQQYYaVT
CEiLHHOST3tv2CFddpZj2XdnskoQZpf3OdNkCtEqEZ8hPiTfZtztOHv0HkTfwZ7gAHgjteUk+K47
UzfMuqiNENOk8PbW30OqQVzzIFhavttptaGyWQeM2z0zNTC2q8GfqREYhvBDXL/rMD9tD/y85CeS
+C2ai8RVA5/Ju+wErQuw9v7tKF2HPp845UyXh6uIDNkH5oyk3IjWplfJyAAUMbTTRt8sFlJcLBa2
YSKP7APPegAMORGQS7JD3qIntEdzr9nTnN5DGhG10sEM3P2vsNi8GGq3QrEsntt6M6/sXLLhcbCd
4IMzIgBP+32O5oHTFsrxmutVDHFjsS+jP9gA2iEueox9jQ0NdqfTbBoeX8Q6kbcHyfK/EIgT2xiJ
5RXCePvnSu16IOIiIZHFTq3M+IFvMkEAAAIVQZpZSeEOiZTAh3/+qZYAAZ5e8eDKdPndaADcS+BP
5jDwOmvtc6u9+WBGktH9kiHnZxMOH/DJxqj9eSsiSK78E596r62RNV5lVLLTbsk0uqUxMaBM6DIu
a5LGSGwZKP6w5fim7lqOwPuT9czK+sxok7q+TV0UfXgZXZpcdgqQD5CoA5paeFOtEFQ3r6AyuCyw
vAe/sceEUiH3phoTtvVhcLxjhVMFSCDRSBu+A1av6J//idevORJVjD/JDkzWEh68KkFqpyx3OzDM
ZZ0Hyz58lNqCmuOQfU8BLZA4Lac3qinp8Zf3CH8wa38dTCqRcqoW/7cVbj3MLkyx0aowIbb3Phck
qyIZY+DTIDnqsLC5w/i5HMA2Hra7UeC4/l9EgI/4mCQP81iMbxXuslU0NsYhlHR8W0IPHXmwKZzc
yOHko+BkqyUpTP0UQCUNvT1w9RB7obZNat6KvHpJlJVbptusSqQ1/AUzA4VF2/h2Kez4nMHYejEe
Ub5Oprz/AUUWrXmF6wQTDTt0nJWN31Whn7U+0Zxpvb3g42xiGwuL8bPXTPtipRHabSukAKL/9pMo
oXcxvjJvYQ+BnQ5cM9bKDyPgd080jSe4euXV5nCFqOq/iZlYRRYFXxOuS84nzIDXqTLm0TBJ7Zu1
BXSMSjjAS5akRPrqGSCQ2lOe4GyhGePk8vgpd7d9JxjkbxL15rjVFtyCd4zE+IAAAAJwQZp8SeEP
JlMCG//+p4QAAzoqRS1IAiAR9zKXc9WbsdIVuj5VMF/tSXptr+0SZXDqD9VTQSPqjJjdRhcBArew
u0ahxxcujChCvhLXCaZZ280qBEJJo1a7v3CEo6h8JVP5Bz15RP/QBLCeZMlpfZWdRT+c44x3A5JA
h8sIQXn1mUVe2Z6ke42+UDXOYkJoBfgvaPkZV378JpJZh7X8dOjvxom0qdENNNI/uZDnguHAeeC4
vP3p2NctBWAwL/tz0TS4QjnQQLZ9jIDSGoYccTH/wKCyFBaiSgrVy/4tvl11yoceJVC3r39S1bGB
rcG3nTeCUt9NG2hHdlcijhgRsAbt5EADj5EUgHHFAe+Jo+TB77XYDimfbfWnDMnhUJ1jxHKAPkve
4BeOWiq+CX4xsu8kHwhuX3Ny0uD5Lq8fLeCAC1iuFXAz0WUIkNijhsrI1Ij0BRbQSBR8uH6hNEel
532Dv8cqh3FSm+X9NGhfANaGhOaw7/orwaAk1Q3/4T6BF/M0FsWkPQanUl1Bqed66kFdHZi2o+S9
qqpAiLjZqJu2VeQwbrRd34DPoCK2XlD6Yh/3AzFv/czYVnk3cPP1GeZ7TmP8ozw+DvOZpYYsAZr/
TrOhSLVszxxxzgBCTKx96avvRv8vBs2MAmPAmgylUa3pJJCyV6256hBc1XDcN/LJP7fXyK5bx/aj
S1Qi8l+sgPLQWOa4oKiYlbhNKrTrmgjPcZxbiAMA/q2XVaMS2aECLZoCej6CVkcYyG8E9ISpX6oc
1nI90XNQC+W4GYxxdC5FmQfEhrHW/6MPAr3XJnuLlMXb6mdMRPxJQcPZa39lBwOnAAAAvEGemkUR
PC//AAN1EcJDD0Y9zu0sDypbgVJyJ50d7w1IdyZzRnPcVQaoZ8K3BQdAZ5t8BtIAQWuYXtZUH65t
9Dq/wYFhvfiZCdGbiYy040Y++cYqVz74L9VZ7AyvmLjNAAxBh1rxFDkSbOJqIdvH0OQGeTziSy8w
WacHPRfuSgb+WgWapkRDu9ci70vsWNT69PM3YhhlbdYUehmWnAuEdQIO5pj4PNi5vrVmNpOavxT9
TirT7BEfF19EDAMmAAABMgGeu2pCvwAEt2I6NnQ/QpAVlGYAM6Wic9LqvmaGr5KCIa769OCNKgYX
w2Cqq3pWQK2ph0PJzeskXaSIOPaiYp/kTViyDw3eYqvEdwAZkVSj2pH8qyJctE8DyQG6vY21Mopu
RCgDQYmwPqw2xzA3WrE6fBfrQGI0OgYank1eAKv/92BchAM7h71VL1zu4e+ptKwSfyr0aDPIhR9U
/TyPs2COH6QwfYg6sGJVlojpDwaX1qYhwDfDZY884YQhvKlLEb8THV5h9FZq6LHsGfO5n7XJVLk8
KsuStrNXv9e7ug0I17MO+LKNCQ6difYM9uk8C7Z7KPhXDPlYBzVqtjDWZQ7D6JU47ryjEFUDOeBp
TViWkkTtg/a0LrYtay4sWuf1/nij3Y8a67owSalqHOOjGJw+YQAAAeBBmr1JqEFomUwId//+qZYA
A0HtoOmG6mQA1K4Ih03jZZC6u6sj7S7uN9d/1Oh7t+Jmgdsqi7eQfkIIvINyxYGahAlSsYR7C627
IMPNfYheMDVe6JtRqyjv0knk3NQThR7cHJmX1ZDFmyWGF2tdvIa27hesQlXahd48VcVal1tQvMUH
zeEoFG76YY72t5q/5MCru2SQHjj6L3T5x1KfH0Vp0AZvFdT0yA4Gajq16MFu39JNA/lhHXZAfaWr
5R1/nwb3cEWFiwc32D/yD9sAlpOQG91I20JauoS5CWZGn+CFbnCLKb+f5roCISf8HavO0Q1KEms2
56GKoI0MfuncViSH6XpV7YIX2Dp55zR5brBMwY5AiCj8jJt1tJnmk6Y9JGAAe8xrlMjN0R4RL3O8
tGWXa3gYwqo6PEQ+PR/z4/bJQGMT8Q3SNFEO2q4rNvwasoIPPRfVgaOzuN5aoHf5fI3JV3sAt3EI
JHAG6ALsQBzJcXqjT4f8X98xuuUPRCzDUYZmK/mk6Rt6N7WPmdv8msSna6fQi8I4hGiJs7WxO1p+
6ypMsj+f4dsrvKRC7sZw8i6EDTMfeMECmKHziJb+oXtpihzaxVdVCGCkpnr9dqn/XTBTbck/3BrB
LWdJT7B+XKEAAAGtQZrASeEKUmUwId/+qZYAAaiZNUATFLSulmvk7YdYlOxdQXEKX74foXqfrYE5
uHGcJQou1h85j3ntrkMN8SXmF6T+pUqHNWIb4o+vBSHRqNr0s5eu1r1v2u0CqFGDjxLdr0SIHYVC
v6onyECczeUtdZi6Q0d+OWiVZ3EQpMggevnE3x+604IuLi1uvwZPGC5GnQhICSMn+l6e/Gei/VfR
QIGK2hTmzOv7BYWrQxqXP7nj4zYCM8EpyIfRMjnZK7CmtiP+IAeMbDG2NYi+kDzArVHuSr+5iRgy
1CiZuBAT8GfiyaBmSg2mByM00Q41RiFZDC9eelHo5m67CZNAPxW4/wrWlqJTrc4EtA/PynwDJFan
cJw3PLrkOF+LuhNDcF42aeQ4AKZ6wtRe8tCF6+qh/+GcLoFE+BhqsU2C4E5i5WahnhM43W8EviSD
SOX9yWZNbbyGa6V9M53S3V5yvBT/elohirxrfbM/YyGDXvURIMKPWA3ifEITi4rh6s9Zrt7FGoN2
sMlc1dRY/fFm06KJeiyJPd0Yjz1VJ0TK2vLWERRjNnO9OpRq54R0smpAAAABFEGe/kU0TC//AAN0
4zaRnHpjVT0szIgA8ifrFuhuls5bADDqpnXjvvi+Go8CHA30cfS0x0u8+YPwszIf/MWI6atf3jA9
eOjrzGNKmSp50vgUmW0QRoMZg+ZEWkwyIKNX1PMlkzml+AB9t/gBYy+kMZDaV8fW16k460HsiDqp
SYS4hhzUn+X4sBP9oHzirhSHsCXMa+9/xsC6Cv/ddMZd64WXrc8qQKGIDcw86EW8iLIM5q9QibTr
dmJIdDjjAWVcAQZUAq2QLJCA/83pd81yb1+80Rw4l2+bgPwkerW2TKP95nx4JnMybOeeI+NNL9Xs
gxzm6nkgjmzBjq7rr+Pn9D/SVhMZBa1iKqrP9SDbBgB1i7MgmAAAAHgBnx9qQr8ABClG3lRODd3L
ElkJNRM8B+zJTNA+5NhpASABMjmuHZ0CsEuz1G4TefBi12MJmuUaDixj8o+tBMhMFVPSc3dB6Sa6
44qb6QkIOml1LgQd597tXxYsRPXoUuvUtIZZ7gmTqMaPaP8DFB8XawmKE5tmHSUAAAGUQZsCSahB
aJlMFPDf/qeEAANLgPAgJVfl5UqefkNP+khwoZUOJjAcDtmKrW7op5Rfyofy/8G8vr5SnxnaxxCL
MIx8CIOSRO4QBmfmlrCMENpbBrJrRlcYUl3jz0p07ikt0JLwd+SoZr+YP5EJkw5vngq03kKsV9oP
STpVpnq5OXhih0cvtNXDJMBhqHf5qsojVfWkxY8dqpQrr/C6mR10Qxf57qmsf+H7Tv038ZtnuYB6
H/8WJW/1GPjXlqaH1FXSJyBj0B26UgGMqURHiIa7eiMjXP6WB81ihV7Xg4m6VPy9/f5/KJArhToh
sroRS6/0ZtoMpnubWgcW5gfm12hJd6vWl1xAvy1kV7Xzsv7/WnRXrAwx6xlcUX0N6kKpbw4lKAzB
y0qAE5XeJx2k+1q5ttmpBiK3talik3xzb4KBJyuaOklC47sNtNviH5ob4/HEliTc/wz2kiXM1Jzd
p9mCTwzcyPsxxvo4iJ2l2aa6RB6F/2f9QL5ti3Q6bfToWIKn0TGTZac3MxEvAOhSegPGOS2Rn7gA
AAD2AZ8hakK/AAPNXr/g+pqqozBJDr3LIAAh9bF8QkZEu0c97DLhUb87g4nsEYzK801WNNML6CgP
dZwcQ/1j98EAgbavRYmEo6khXraxjTAGA/l/Gxbtavy6/qZ8RlfavAgBGYm5+Ey4g/nqsUAt9Aal
H+YceTwS8ZNVns5FA3+nrk5oEqau7Man2G1flPjBDeW2CtpIw4jPFr5FuINLz5PbnhTBxVv++1sA
APfu+jmvOgUIpGydOSEgIQnkpfSYAzFk19FBLTrfKelr6PgmQHd9kardKATfwMjpQdTkBlksvG0o
4yt1E/BfoN4ryK2Dl2u3qliDzfGRAAABekGbI0nhClJlMCHf/qmWAAJj8jtnN7nr4ABcWxAZeTDy
RbMejuvDZCXGXIu/clDaXRqJzm8Hn8/NRKK/Ged12bfBEYQfE7EGtGe5nS4d+bi+2GAlJcfq4Lme
mieTbsq07fGY1y5AZ8USoFUmCPxBCiJk3wdeRFml9BQosiGExm5G4j8urtWK81koAoRrh0TMNHKL
ZYs7KIRQHlALZmyMhPdiw2h/fCSKnqT5QIQhlBwwqO9GAyUbj11ig8zQt4k65Fq32RUdPJnMqhRk
WfgpAjNH6K4PqXM67R8mdFsNCYqC2D5TiOfiAEvo1JBocYFL4F9zmemCjZ66z7YUaMnNIYCmtl1L
GhXUTWv7OqdkAgSA2dFc30ra1wgd0ykiSJujb9IhwPuxfd37tZe2+6AY4lr9x9xMGHFHaEyotu/o
QYXVs5iCoAglPBTMGcK3We7ULvuJY7gqA7raxfpZYN9veafNhzE04ra3ifqobw7nvc+Pf+DXsUJk
1kQC8AAAAdtBm0VJ4Q6JlMFNEw7//qmWAAJj8jtnx23hz8AAsUhAJR//8v8E0DLeI6MblAHjjWEN
z3wch7z+MUMjnrUDdFE3N6ypeHPcKZI90caPs8QRAG7IlBPWn9tIHHllX6vZB/DvOKHgdEbpVCj6
GC8PNmepxI7p1S9+3YbszX646KADThLabOXMkgv1rTQnue+k30E9fYEGBwDxeJ2Zj2hL94r5u8+0
9m6+oxvD205Y13pu5yw8aZg7zStuk84016p0jh+xCbS4MLWnQMX3niE7DhrbC79riPTvsT/xzGrO
E1Ej6YKKkn/ihdQDkaaQdbvvpxZoigU05eZRI9KwCTiQ80UJhYGvrNHAnOgoapCP5HPSkYnjLkWD
vsZm1YHEJTXKu5t5/jEvrFeUWyMEjrR5vNvxoNtPf02k7iUzNJF0ATCE29lF9kBEA6+jtA0AxT08
e+iooX324EgJF+57NN74/rZ1D6zWMGea+M1/DwnGR8PbBySpeBtFQFaH34WUg3FaiFnLMD1Tbhjp
xXCU241ImlRFLevO/IIcmxPg4S6Z0uY0QO4O1xA/Se2M9tSd/RQcPizLj3NsDVkQrWRvnUlkaEkN
tAf3hZLkePNpCqC54sqxK8ceOaI/SUOyVTSRAAABCQGfZGpCvwAEKUbzneXu/lHuZicmlLrZFtCm
iySUE4AO5JIDb03XeNFTPTXwTub8PxOZf/nD3bSbsrrAmQNbNBeeDKVeTxEEDdJvoon+m/RWxGEJ
8y4LjPs+cxmFnGAliGB4WW9XEA9qVCIhbDrJr1xEw96Y4TbKPITwmHNj06ThasK60/ECBV+lSgYG
rzUkNzCji1cBhJ610WkwrmBnTufM609o3B6HeNqgLgQ+L1hRA2fYaxPt+1xoWwtsbAaTTfIDX64r
0+r9rXYJmAtWa895RfEZ3NXSaEbPo7UkFfL63zavmBz7m36AHKkCxDwV2JVI6zoEEKk0Pn19LaL+
TfEsC9OX5Bb+8qEAAAHSQZtmSeEPJlMCHf/+qZYAAnPyOtBY70zzNJO1mAGpMQCUf//DLp/mFfLR
e2Ar1gPG7X7ldo3QLRIRZj+2WNL+ULQ6svL5JOqtd2BTiwmyWOgD9uCgWAQWP69CYRbl1n5dkutq
fZEhUBeOjFLw3i8iAAYf1b0YQon7Nx/0MZvDfRYnuKuPPSRMPrw//pU1PJ6TEcUP4LnQnJGF2ErB
6B/5Yjr412I7jhMUfRKzCgSSX4dqFJ78onqVvcG0Ffo0H9WWuds08LfHckp7EfYAm40UMLfY6kBY
LprcqT6uFUFZmwPdJ0bPs1nds3SkA3/kWNMUyUtCzci9LvADUaUJgTMRxXYr1asnazaumlEwpk9W
SId3vmfAIAJo6ndhoDmRkEUMy5j3CL6ob+wD0qX5iZGhqtdWozI71kW3iIsL6w3OPVSAUCAJIneo
/NPyE8c4ntOI9M3+hUeaj1JM563XwCvVGrtylkiKIdsPHnXVRbYRPIZw7iFk9yH9ER6WKb9/LBym
WMRtQsyrDUwnm+s5YIfAMf8nsXN9eGAAbJ+oXOnTZNMtdKe6AALe1DO7fSZUh95tJSAcyTP+dnaF
wFqYXSrUPT39+d+tCpbMBQMFpEJVcnuK2QAAAXpBm4pJ4Q8mUwId//6plgABnoZ9FgBusT1mAM2/
Crxz95pwIHF81G6A47/UXhLf/uHY+8gTarLKFlR/9IXO3oaH3Vybymi8No/od+BrL68FesX8AORg
0mPrJVBQsrVdHBbaHxT1sJ49rF9DRS2lJoqGGu4ge61M1/7xn/2iXTfz9lkw+vhz8lx78ELSs/GE
gJQnlxi0/ZHN9x4Cr/MKp6N/z+mq2hvMFvx8skPeYg2Rbp4a4UGRUe2Sp/VqicXVYScZJirXIyJV
yAcjqOwnK4hTu2mn/teBLfgAZIx/NxlUF8HfQX0aTBGxg2pXpcezENmlZsxGaX1Ao2AnVJLtGF2j
lfxM23Z0B+7ocHPTWk5/2UmGuzBso8vn22bkCt9aYhblDPqosQAzlGsc9NvzHgPLhD9gwK39kTcH
fi0g5YagPDI9VnQOShqL0rTkm90pzrqT5nROALim2JDjeBOwawhp0OftxtS3Px7fV2nk4qtwSAE9
mQnHAAscatsAAAFIQZ+oRRE8M/8AAktQKiy2sksLDv+8ADZxSfa/adu08WMdya29ABV78aw8faDx
zZjJx1PJeVGtPTmIr4y5SLsOzqyhBWpiIgfiEKMd+XtCRkAB5H//5CyeE9oI1KxHM5bSb1BD+sE1
rxXzf77nF0qa+OHJmyzkitZtbiYhIEmkpYSZFtkZOLibuGxXYwLciY5hS3zoTvqZSQCNmbY9mR/j
jRgyYobpmBregas/x0k2dy8sYMUfhIR2hYxWGJVD5fDpqbkuezXki20ZLMNsBG82BCN+FjuNbfSa
Ea8LpsE1m3RjJBXiJmOibQTnDC/w5wWC4zu/BOvDYLWU6j0NUkKvVmNDjCDYjXKokhTDmW7hJQjj
IszU4UWiN6t8+1GaC3QzTtHyaSG/IUxToxUuOT2OIXIYNhf7XKPDp1NJog9ZPO4LDOrEbDVPgAAA
AMABn8d0Qr8AApdgcZ8XsIrABLfa+FEnm1s2q6uPd/ZK8KyCKz9YN1nL1tFA+c6daeiUixdEQfp9
4xdO5N2deo8I8v05jGhB7nqnJgthlwwf83iSSHT9nP9Iip7+hSBLPQ6FV2ZOIZZvorxug3yW2xQv
on94FKQhkZbVmLrtkfFLU4DR+lKrUmOJ2/iW1uAUSonKYQbdFvr8OvJXLbHPs6jOPA1c5GhszFzF
50dWDI17N+Zi7DH80Byr27ihGzrxleAAAACtAZ/JakK/AAKXRmS6WqnFKlsAHu1mI8INoA/VINbb
BoCDvlMAUHtLJsNzHraBhVocjaVe6g1Xwq5Qg7v1ZGqLs5Wk3d0B/xi3GGw0ypsPigZGMb3NMaZ/
LcI81/kZfJQLhhwp4Tq7tnSpNO8h00/sbxAVx+/0pZrNFd1SK6jbP/wwmr0cDY/27Ea945Ahx3d0
EVHk+rK4NmXVFMOAt1ntFD/jqmaiAWmHIhE6UJEAAALBQZvNSahBaJlMCHf//qmWAAKp7iVYD3ce
dvCqIgAbVPn2QypTuj7Zl69TVIuqMmnNtiz35z/wXPVUrnYMTg5IzaM5QDgbKrkTETM7yKrc7ZvY
hu3xE1x59Oxe3Ts8bOZJSewWUuMF8CDhYtWpzGfpLL2q5FO3FJ8uddwLA6USJ+3o0lfIicSjA5bE
kMQ8pmqtWeW1n35mp9dm7wNzIev8CbzDuQgdNwWabgR4cov/n9vUvmTd6EIeBoCQDguhpaeR+wXq
DXSrW3qyUVjGbgpRCNXzkCv4Cm+iQfnprqHk8k8/4aaoTwV0++14Ryrzl5mHktPnZ4RCQaPgaXlD
ODre0Vavy2hNYyL7k72eMOkb5wkN2K4iXGmDYI5+nLBViObxoTnntIvHSR3ELtYipOiSCoqYpNlz
MfW74XeQ4M4aiIpFf1IqKNDiJdn9yrJFtZsJOzERbUvGVYADw3JbY6E183P5EgK7BbLtl9EUxeGI
KQsYfD7QweWLLoJx+x5SXixy0EfY6TF0s3hP2BB0CR4c/W72A4Q0/RBfbnoK6lHVDQUJqrfE1Uq4
j20QAhtQXyBKd47gTkGtNbYh0sztM2LXd5729wSQvBKmjbrCy7FYrrB2CZEdUEk072lI8ERLm2bR
i9nUfJXfzr/QE/h1Gq2+akmhXJH578A7BgK+xxRCUEI8VqnZP70oSoFXT2B/PEJQUZjH/f4jj8fS
KRNq/guVlCbPlWF+LEA7s2QOgt6WFXhydCFmzpaJwBF6AVImSi8W8vFPpZNICuMR18dlXO3iEKy4
3v1qK/Dg5By989CS9dUPcvqRLka/T9gzRM3SK0hZDQhj7+fnK0zCzaW42udNi41vSHEUwK14kOoL
UCI8xcVGIZ2r5nji1xl//fpfIbgG6IRQu5JtPTi2wo3iEREhQbX0/s1C6u/PZkczdhljLl9AAAAB
AUGf60URLC//AAPNEa8gCx0N16EcAJasxb1B7I8e2VN+s99SDmz1KY95FUyMBXYagscSMwHH7HUi
2sEN8C3cV7mJz4S6nwhpqHyeYUGDEW9GZjBK4RxfQFV0aD1Y1OsKxscHjo1bNTwiGHz4AoHaOMPB
HNOBbLITfcTj1IgDGq52EGjZsoBJL4M34nvEZT80namRDPFlvNw37aQEiHxink+hZq/G7bN1KyQh
1xWltt2Fr9BXZ4L8U1ZSJSaiFGy9DXF1ZxzEWCwoerXPjD95RpW+yaqPxhqn9vKbcCV8wtLqbPI8
e9Omq6+uY4GzzM0AefYO8GZxmPdyk3Zkt5OeQtroAAABTQGeDGpCvwAFHsAwh+nvfvp39JeFRegB
MIZj7Ij7QhldLCYmtAsl4BAWY3GeoOT+1prd2l6Kn30sTWm+wiUTzQPbrfgDJYgDXjHvbAKqm93/
PAlatNoTO8sEJXWNc7oqe9m22YuU3osqw+SjxcIlh0u+OtbuBI7VcBURZB+qw5okBrDe351+AYi6
C/Mtfwe7J4JfmuFE7MfKTHVl3lyRs5LnDI9gDU2e8sfg09RTBjhw+yNLoAmcff4XKLt8ZBrLTVSt
cOs/4qKdD3qFZciKBbxJiBNzzoyHF5q8IDWSlBCbSbuqAXLhawCqrTi+iFvTDznm68wSPlUJFB2n
NlVoFLfM6x9BnqgyY9HFUUinFWptWLr05RDYCzjO2yQxVuqBy76X0dEPufCKQzNCCDmsXHrIFpt/
jYsmIblxTCBsT7bCz+++mFz0k+/NfufLoQAAAiJBmhBJqEFsmUwId//+qZYAAaAXLP/UAJX3mcQ2
01uf9/+clOnp8CkfGBYyPE7z3ywiIHFrCHdL/+CiL27Jjnn+WZX5qh0eWGumsWNAFvJVwah2WGL4
Kr/ZX9JeHUxuQtFLLQDM2lFz1EdAhYFqfhH5OcUxxDCt1V2rtPdSw/UYnZucC75DwWZd0wo85cGk
s9stYkX+EHID0zNE24b3Mm3DOkX5Fe650x20z4tvRKhGflC2QTK+tTeWl/ahK1aFYeLCosMeiL2o
bmOyHLXZUlQ5aAX/8L18qT27XkeaQAbB8JmlUyM8yA2gcfMFzc25hbm4DonjJPJrRBEms6VGdYTq
8UTD3vm1pJT/z8SSkFeT2YAjnp17znRvTwza4hqg1WUuy3cJg4g+tKTHz6aAySUEDmV4wvQzueq0
mCpvZ3QIxQXeD6SI8Wr1jh0zAY6A3MgtX/kcLi7N8C5MCQpUk0LK5AObWdx09wmEj/iRPCLurwbr
JEu8x2nadsDlopr/usRw2DkARfCMP9vJD5pQTkkyJkRf7/h7gXd9sejewgdioIgubMVJavhdrx0s
qwFjnyfSquBMlxiybChz3xWKPPNGm8Ed2+xItb6dfKWxXc9tJe9l1bfsPsHTGp68thKq/UshT5fh
DW6jrtKQvoEPdcLhQsAFsoo99kcNWKPTHzzYpLeWVsvW1MT0G9Iq359TTpRpRONNokyWBuaM2Vam
jjFzliEAAADgQZ4uRRUsL/8AA80RPvbT8sp9gALhz0sf16NBGlPvtwQ9TcNj/yQMye8fq5Nk6RSE
rovd1YV4cJ5xAN2sAR0JvJxHSgpDSqBy+US2EOvMyPlWGf4aIK3Y2r/as4ialp2z92Gbcj4VZulD
jmjndBROqJ8liiSfTI7gEehyG1BJfGBoCaSOZOJW75eVk35sJaWTjuLooSj66lP5BcjQ6273Q5di
VBr6ZEyKCWnxZxoVuYdgytzo9oS/uRtqaWVLKy8vHuezx2UercV2FyUJ5312MOsIcgKq4gCuVAFO
elZhYGEAAAEwAZ5PakK/AAUewXdTVK87AAhBOvpSrq27ouLupsxPFo5HP5kjso492wwvkueJi/4k
+FXlHnNwzSHY5+Of7HjR827awuiS7cT4AeA6+cKZg4dOcGVC4XFfitgW6oAxe7i3VrTvqcdNwPPX
bu17A9Rmp6JvdTph5jj+hWypjwNv0lAF5G3Ma15UDEJgKXFT6omU8+xM1AOX2S3HqZLyA+9FBnoI
kqux2jBj5+sTB536vNIZwT9F955nMy3JKwFy7lOmBKdZZ6i+gBsRajdlDt3l+eJHaD2rgD1a/kui
xYgtxEYpu/wUYO/oEDB9YfM3PSPA3lr2wdrsmIywXfUaaAel5TjGTtBnBx55IIC+nIRjpqZPwZnz
i/mhnJJ2qWVmdLmlkaXTVim30uIKtf1dTKVKQAAAAcdBmlFJqEFsmUwId//+qZYAAZ8MoXNEABXP
5rw/syvVBj0wfmq3UNdPIZqd1oBGJkV/AsbGrRLfgCuBdtEfe50qgpgmuu1jdnqzaO34S4oLr9Vn
NWclaRjilSdmkZM8Nj9wLGjjjIl7vD8wwFY2dF9neI/bzpvIZZ3TUJucqCTsdLz8kyX8yhhWegbH
iJkWmHgb5vKHNvY05m1bqD5cfxsb2qWdADnTtqB++IrCpy9DEDWsEKGlBwfyIdI2LWXdn57qBsAg
8KTEcEwPkmxI3oABYDE/iviC5WmrX8cpb8sBbXpPZpWc5/t9w0GbmAmBalmCscbgTwj5eiVDdaAI
FW1qCquL02UFaf0mUET4uZP5g7qe8dlAXv1550MzvSgsyMmzg7V6xiRPI49jGSG1UllTtuVEH2EL
mcgBDIWTo7tHgLAxNUma94DeFuAD+4SxPC8Z9aKEShzyTkN5G1T3XJcRHHZegjhpz+Z37hgc/E4o
GWtxEKyrhMI4g/Z8cV3t851UpWAFur8w4Z413m4vLg1hwPqG9Q4rimZ3E0gL1B/KXTEz8EdiV5bA
4dwBtz4aty6hGsLjIdAQfLkWtLcoUyLCPIW3V+ZQoAAAATBBmnNJ4QpSZTBRUsO//qmWAAGflmJ1
8IAWtMn61PAxmOhsZj0dwfHFifFD5ehISxpP82v1ytVI3PRpBIkoz7HX7YOLymGczF9Rb25VeMDx
jTY2nwy4+lmfZ3IWI5X0F8WXZDWnYyLnht9gwNJOCDRjef+6CttllvU2rJMjWi/1s3r9u5o7KOPK
RYsjJPqhVKpiAIvqdOfvsTJJIh8uRkS85igh4jqdCZoLd5ZaVrJq9ViwRrDF96gKvgcX+kqxMeXn
M5U74LkaZduFLoHpkMoQwThFLX2zxcNfMDNdRHMUTuZ21JdyPV3t6iV5A81yC3TKinL3JUty3ToC
UwfcEyB/uNggbm+Bq7CV1IThWrgXNH19vmFbhCKtRTSTMh1SMlLclpEKnYuUlAQ3MJXeFq+BAAAA
pgGekmpCvwAFMsyMEFUABDLBENeUBjKseqF1oKpGxkrqwSsMG0I/vIwVEoyhYp4RSCwFJZd1StA8
ZmaTSkpQzVeX2kIoYK5zg+01KfoD65AkWzh2zQCdmcGS124/elhIffRWr+c364VvZp99NpxciX+I
EUUPDCcP2B/TlqtLxZOgjJsN6cku89sqoUlLYS7yBDWR+rcEMXu3t1NQvh/eiAGt3VjC0OAAAAG3
QZqXSeEOiZTAh3/+qZYAA0HtoPRo1Kz75H1ABfEwafgytDo+4e/8Gr0xUhgrI5vh0SKYDe1uLeex
eYkAvJuC1mWaZY+7NoA7QZ0aZe/gIgn4r+6cZH582lspttlJ5lfrNFUR67B6r8fREHq3qW/vmbfV
7lpCwgqxTVnmKo+IWIOAkxKjts68c2fvF2WLXU7enMCjKR6tv58LnFYWuLi4riPlYaG3HXEFCEUr
RKfoZ3PaupTFB+nj6PO2rR95CuQpWSs+/AJ2fJZk/DskJDSksYKnrNxf8HFda1xJoyYWK80ygzrl
uPTnwl8pfz+yxJ1l5qZ6qrfqis5Dp+DJwSAAql2o+z+SgOOgBfsm9vLMleDYE3vS+07nkD78kmyh
UHfH21ooBmR50j6kR6TIslMCuGeSzkb7+QN+BynSnZ6coBaG8iZyaSMa3rhLDckhAnz3K+0k1Zm5
Jp1Hz3qeA83u1BNxnqTi02Dqi0M6P751XHRFCnpXOyjytC48sECwqoQGdwnT0cpTjM+H/Xuq17UM
c617kgQfxY1aMindMqzIf0HKNYRN1MvUBnM636dqyu/ufZU4d0cOZgAAAQNBnrVFFTwz/wAC1qHQ
wBha5RG4yxDarQTbCTDARaUa4AA7KtCVUjnGrVq2x05HaZ0PUWsP1z8H9XXMt/L9vH8h2aufUZZm
vMocPddcQUjGuAeo2d6HtVS181XSwmsuX95dkicKIF/Qo8xnUZ3Lddcfm9qbp/5C+34qAU/wOqmN
a0X9SDw4yHOGPxcDyELv/Z7WgPakq4JNKcfbjc+h73zu6iKMk9grK4b+9kkCitRnEHXZMdYDV/7c
+Ed6SOf1EXqbaYkvE0ZFSh3XYQva1GDhVXhTvr9g9hPL9FOwQc8UHX/DtmOMSH7z64AjSwO/8oqi
h54G/s7+NKEyJtmxIVnmn7WVAAAAzQGe1HRCvwACoQ+Z4DXmOcq8f15f3qE7IAC4FYPiU1Iiakh7
uec1r0eNBXbMzAgZQns/SMyx9jbt3qiEfS0zvq8waZ9F78yLP4mdOG+8VejRtH1//5RgrxaTZbfJ
8mNGyWcg5ngdpO8pCGt2aFeoyGJN7n0Cv5DjPmblZIx0w0Rx67VhTlQ8WFLoX4/G+YoB0TuD6AtT
WPHKEkLx2zOAhfj+5Lx+333JLr0mxhQEp/oOrZWW/PUEiIQiNlHxCGYLhIh3Bl7IXtcjNyDr6/gA
AAELAZ7WakK/AAKfvE/E6gAP7v9l9HIBVpuB7/3R8KMfFyidNFMMQMjXuq4hd3Xk0bEA7/KJjtXQ
r1Kd7iWAhQPEE75/YN4MCBxbNBG7be7/m+6DcL5O1UFgDByWUwMiFa+Gy0t75GYNzIRvJ5XqcFfa
mYQ6bhIaF/0Epi4eTsQeDwZ9c2K0ZNsp0aA62C+lQmYWctUjJiCZbAzRCgaHktOji5p6AoXRM1dO
jp7m6xDhJMZiU8kDmI9xGpeDdUHnsmVcuyUU5kh41pGV5AqsvmvzwBHkZ/eoP6SJdtjPKNvZml6G
Lvt2vzS4rasLvVDxo8FpWklGxQQqJ/v7hmN0je6lJnQ8yvH5YlHfv4XhAAAB7kGa2UmoQWiZTBTw
3/6nhAADN2Am+MoACw4mg4v//j/nQV25/3/kqQOhypbZ7FNO8r2tj0cOYeVH9CGAEwx0ElOCde9L
kVirtKHUQ32XEivkoe+80xcT2S3Gkq16kAo7iqbWFR3YOpnKr/v0gXxvuBQAGHbqmENDyl4GNSjy
fnUGVpSPxzDLBGpY56AlD7wD+2U9VCZ794xF3unf5KyakbXjUuY5xLq8FigOgcelDCvvyLilpXxn
Bil+xKIhNGh0pVQxm9/bh0fM1WOETLuJLoZ6pq2gJvEf7AqcjJfaAuwbFuikYSxNrfZvdXSNIGRg
i22+5KmvVaNPdjz6Rg3nqa8igTcn3lrw8Jz5XbUvhApOVcaznumtFaKOL5D3AwH2oBKPBdXLUcDY
q1jtuc/XhBzcpyDyJUxlnHUv6JneINMkkGuMhpXlqBaQuN2PYxBm7cbdPwqQ9TWOL/IeO2GT8Gwv
JzlQuK5aaPeTY5J36uokP86kuNhsGSJQBmxBEWgGUKzcw96mffXgO4J7j9OX/BT83SSxVVJXogYR
W8ptqVJKt8LrgqHz49mYL2u6j5dtjYYYEn8qLxXHapMXBRRxhjrhlW73uV5NTGhkrEPh/RfRucrH
742Si2t6C+2msgNIDQiifvf9gpcFlLE6mreRAAABBgGe+GpCvwACoKVCqwAgs8cnzXB58dnP9gpH
uIDNbXrYJn0x7uddcsgjOktBk5aqI5lQcPG4BWr8TX5ie9qM8ITMebFqSnq4zh8UENNpzuB4R5Jx
VTkHxEdeS7m+xuobpi0ENoecMK1jx5JS6KGGuwNNYaOkLcUarF72Zmj+q1vleh/dxWlxQ9miPqnc
6xaDiZiofCjb0Cuce8WttDNzkadRkWgaG4GhFvWxY31prX1G2suhTYwpzcofX0I4qP1ivIUUztDo
ZtWFxrE67mCbaT9+gGR6qaWVM5kkpP4CG4d4kn4xIA8IabaUsoCy2lbYszXh6pup8J5Jxopr6EvY
tzXVeJNxFtMAAAGsQZr7SeEKUmUwUsN//qeEAAM2iDpA09KlIZ9QA4MjN5aJ2veZkdzZhcPM6THn
PgGxotVHFRMAr5Wf2WFRCQHH5mQTfG2ZxoGj8vBpcSknoAKtf/gLIB0dYkLdni3ag8jsuvrKpevw
2ciLbeLARnNjryYYX3tl7CBbKGaulK3bRAYyYeKANngO7JUziebAOQkvsGh8i+oihwTtSEgyi/11
oxE+L8XxSOD442d6RvSK1kigqEWUwzcjtgRcToT4zYNP2248/Xg2EZPk9oeH3q9kM7IgeHVgsP2x
PbbM2sCanQM1zwwtyhYBlLuOlbJtrMsgOiOVsZmomWn7brgOX025QZ+Okf3rHMSzKFQislze8pY4
tIkb9+WnsJNcqQ/s2SEwqQ96cuQjhP5tP0NYvzLqR8I3iBYouE7pKJQCTuXFaaHhq3crllSOKsv9
3yVKJ14sMvi+LBJudnCl+akd7erG+D2mi9vIrwHSrh5GXaahaseFaVL3Rxn/SNHcFJccuwvTYd+A
9uoxuCK+G6MvplOWlWUvrr4c4CKupM6SUdCuvWz3sA9x0TTgDuHEJ4UAAADxAZ8aakK/AAVDkx7t
921ACReE/H/qzU39JunfZmTkN8K80bdQyOwHxw2BEfFinR3gQn51TdJRfcYZ/+5+2Fndh+8H9Sa4
b7P9wvFXT0bKuNSDA8GPEDRDiA463S9yFJ97+ekrUP5Sov34TLfTLhur83UcuqxIDAGSpQLP4Jlk
dvIcYW7ulOz4ziejwV4FXDjbcCZMgSw9rCCWlnc7WpjDrZmImqMElXH9wX87DnMXrkXxP0t5N5g1
aLuCN8yildv7H9ADernJQGhYziX15sxwsLJkz7GqweXITzq8I85P6Ozm4YU1eE4VnYw8e56UNdDa
3AAAAPtBmx1J4Q6JlMFEw3/+p4QAAzhT2K4AhFfFvzJJtmQWGW8b7FUUHUc/agf1hIVFrwPkT7VY
1rxtPUIrF53RbN1qW7Cc9JDVrJPLEowXLE2IlurGVOwHi7v5kwp5skpNuKR8cOtP2J/aGa4ycUPj
KqmCDmVpxrJE9X7XhDzMDJIjTOIWCiOkR+nnhA8jpW+xI/2FG1FThWz3MSaQ5yNdbbQQ5zccKrk4
Q2BuP9/GbOE4HYuvGvx8MN7/MTP2NYG3f1Hoai9TiyAqA09GcdPDcqbeRYnlpx+Dwy7bdvia4uv9
ir0pFFOpKlVvFkbkM2rqft8XMOxq+DWKO2kxQQAAAMkBnzxqQr8ABTLMhFkYcsSpf6jACdPxBCZf
15FayfwvsSRvhHgj51vAYwLsT6IfLY76d1WvxD5t8yY8ELQhlqT0z48gzxPHu+G6lqHOnAmFB5om
e9W96fkeWKNxSqGnGBGK284Bp5vsbq10fQPnXPYs6TkCKgmj85xkBBancgN3KK1iNeoDmdsXd4Ls
7K8kwlG1RgUuh6e+MfEJY1e6vsBuyqGnmxAC3b1gEvelvVJdvrTQUZXqfgfiMZ2fdOr+H61OUogM
kAfLBI0AAAESQZs+SeEPJlMCG//+p4QAAzoMEla4AaiHIMfILH74YK01y66sPrrvrIEUnQE9IA7f
Z0NHMyOChRH/CjcEx9Q0ljkUF1cDuzysGJ1pGa+u639U9l+cJ47lAh1NELhM9Gyb4bdrcKsCii5H
4t/CVTTqb5RyJlvF1KP+sxmfBuuWRfJcoAMiJnCp25utAlc+NbOogBZAMFdmc78PWeq0YsKiS1tL
ARx8RwpmlH0OGx47SA4apD/EiQhXrO1/BEwQBmmPDNnaiPVeVRpMfC0yf7yisQpbvUyyIrBF3Q+Y
Fhu0PZWhivqVI5tLS2FA1QbN2nIgrDwnZ3BWqF2cBS7M/X35ulcz+Q5+Yk74RFDr7vhfN6KpQAAA
AZJBm19J4Q8mUwIb//6nhAAGd9lRDk58gAaocBqbFm5UI1A2yvjcQJOom3gH+e1g0OylePY4j5Ls
NvnWfPuzCmIasHJvk094fiQB2C/rqA90NgdBuu/6xhm71hpRdkt4ewylAMfcWvqZNAujYoCQ4y3A
Fah//+fzBZvcvqxmzCPEVF7z9t4HJeobToPKEfyB17SEZsaFx17TNihjyOK9zjdBGTbSg3m+MSP5
5gOrN/liyHdOovKCaoMNMkqRTf/XFtruevPL0qq9hRHwpY0bDSpjGuUb0SUAPR01YpDwyHl9oI7f
FuXqd1aM4JuadSX6hdzpu5zPfBVxhj/KW6MUfqVcIENt3VO1txMEziLqEfPSR1G1QaW6taltlKwZ
lEd8fVtCtZgUSiGQhrfpBFBSkLKHWckeISu8E93i4gvF/iDzX9+uJEcagRqK9VKfCIkuXbg0+rl2
LMAKci9B0aB7Z9Tra0t8Zn+PxxKa9tE6AeCVgr1oh9Ne10+v7WC78TrHTLtG+zfCqESfgGvyGa8P
IUfxsGgAAAGgQZtgSeEPJlMCG//+p4QAAzaIuxcnz3taWI7SzS3qfGAC1PvLNzDGcNyQqrAj0KzF
/rnsIo0xpZ74LdLlfDGzeQDWTiwadDfsm/kkkzhfOmPIwxsfIPPQcBbvWXS+fxTsbUGpXsqoF0/+
9Irk7zR8feZfhZmW/Y9t5/GZ/lkcd3vC3NPtKG8BvdajcHl2RgdY+FMu1l8o3Xa7BZqfq+1mlt+r
OwJ6XWQjOKg5gZPX7aP7aWbc10FijL+I5bCQRJHVyUKprB68WHloKOUTleVqzn1LeL1L7Bha3oRu
ljZ8HSHLfgADgjKyPJq8fscNcYNLKtAzsLGqLMvtM5aRt7abP8cC7mG9zjlui9B0JX+8ReVcZjUq
1+RN3pX+SRmoOGY0ZvEgdFbLs0iJoXok13aZHxHDlbw5diaImP+Yvx8DN2a5YrHMEXb8MSsr+7Z1
UfVlQvtWYwQMtFBqygwYWe9xhRtuNlWG7+YGY6CnoVryC2QN3Npg8YpcOB2RTTIRYehZApPLn/JJ
RbL8/YxZdrXjsOObDjReZ5g2a7jK1BAjbiEAAAHnQZuBSeEPJlMCG//+p4QAAzbsFl4gBu328Peg
pKc6RUnsN5JcMpfTjKq1SLymru4bl/AwUBEo1wMxKT7oIZt+U+6wNq5jIYQRkheGonNL1jrChAVr
//kELh2DchK7xjK7orot465hyDKaXVuG478lCyHNzD4pICF/LJFJDZt1KHvBRUG2R+Hic10aUe4b
W9QtXW8VGVR9p4lJulhBkCWlk4MbDTGyVoGUCXaB+CiEK3McpLa4QTLvRYoHgjQ0bFhZubbBNnSv
vN/oZzOfMMh1LA0bLriyBGILs2hxszcY8CSFDGwX8B1ptVYd1xr+nBmk322qsgGKBi2+6hRhARoJ
ZgT9zRAGoY6Z6qM21mllvkNNE4yVLo7uLogmp7TIxWVJX8XBqCDBNy6v0+0ilsNIDKkMF0VpiEkO
Pi0QJ4Eq8LxS3MviRf6Nn9xWDlPI/okbYcfbZd9KLtX1ZOMsT1L+bXrbfASKRnLREu6Ek0yNAro1
kCvGJ2g5qMsmjtf49KbrogumHbz2SW/SLsQgYK5ZukP6spR54833sTT8FopDr/EgWwq2HolJyyJn
S2dsKQcThdgqoVWoLikLi04JnQTqYJEF1Qu1sUyP9opkE6zmeQAesBWCmkDxB5YMz9MNgqmQTotH
P4CQEAAAAaJBm6RJ4Q8mUwIb//6nhAADOgwSq4oAS9zntns08mJZiDgS/wHkOKkYjLzFUdJ8JiUD
NIb2oaQU5hC6ry+BK33OTh3bVZKLmXtWwzxJ5cNapz/k60T56PJrj82kIB2xNc22XX8si4XNW/oG
KmAyXEQ3n66QywU8H19nYswgNVDtm1yGcasQ9d5VXcp1lab3Xu9VZ4F97D36vzeK8Y+wwrbbKU1v
Xta/BQy8UyCfd+kkMokuTeT8k7ig5P5iWK9EDWk8p3/rjUyM/qb9MC3OYaRC42Voa2abSL9qss3d
eW0iE+iyfl4fZBKMIm/KRnP7iaMO5806E9vF5cJIlKtY52/XUaVv4sCep6EnU4wWgp50c1VWIfC6
GCyg29/M/Zo2HIzCyMj3sw86d0/LlxWFOe3To0pdGBLfdmb3qEOqbMVvP7/i3O9hwrtpvPh5TjAr
Fa6MhCB6qIkbIwUQHnbzuxRnEtL6mWbP7ZhRzPbSHiCS43c4Ynm0nzT17Esgl1mdaz/BNky0p83V
2fkzqBS6s6H2FPtV11b3ZChzjHFPfI5dK9lLAAABAkGfwkURPC//AAPNEcYKb122YAICJVUY9qke
/5Lo7533WN9ok0LavYB4nXddSZQW3RefFN1K1l1Esw61cf53CM5txiwqQ6MOTY6nAFgzFuZGtFTM
LY2yKPV6nkus6Ib0ZkbqrrVJ/+l7HQtwHwTUAlnBfVOUj5tEqw/6DOPN73/OrVRfC7jtfoxlWde1
ID+HnEzQzo4pdjjwR9eFrPuMozJNuIBgSqjJB0dzgVHbLofDJeZfDw2rNsfz2MMKMR87i6GaFWqF
5VBq9ZPuhiaIolweG8/cLIhnHGNFhjQlhKiKtQQEU3fsc9w765JO7NkU/7WR2VO8PQCC5nSywDHZ
uvLqgAAAATIBn+NqQr8AAqHJ+3chPQ/iA+8omjA5Y5bQdIATTj59fIn+uF1gqZJWHAwZpWjsdlsh
ntd9QL/p97hNjGLRUEqn9dCILxhQHqcaITPDF/I4Z+l4W53+cJ87rAbSb2ITwbeP20jxhNlcBalu
z9/eYqSxh1HuCO+7bDkA98Du82lkCCnoIdKK6AToChg935q/6NK+58RSpf/8dmx4251UhtF7QNDM
Wkvc1fApn5/SbaGLcdpvJ8NxACuXKSDF4shwAZB9rYUolAKCFnKPrgdMgRfo2WlTP51sb51wLIFn
sVlyZjkqbmagnov3ydHqAHDdegBti+ryd3S32fLoRt2bhuYscn750QFauEDeLjQK2PWBJTEDhI7v
bGkG3vYUFmYZwfqhRMVRyxRpHe9ktlnkb8c1S4EAAAFmQZvlSahBaJlMCGf//p4QAAyQGW50310f
wAVZ2p69+VVWohNktASEBJHuAtYqMNzG1U4dPoDExpSVZF8WxRm15PpPcL9Z5T7iIA0uQGVteuEZ
iamOnQ9OC2aLbB4zHAqP3xSbkJAiS/f8PNsHYQLWXmuVSKEBWoIytSsj7TbWbYhmjbuWkO+lb4G/
P0O/Giki6EU4icYGW0hw0OVqo5QUqnvy1n51tLEhnFssPs/wTuQ66X784ylEwvXff3Z4pxQ1KJC0
S83S9MwKfi2F3plfOetldJOD/d73itU7bhM3reL7c7EHPDMiypN7Emdn+GR/zx3OIyIaPzZ818dQ
L2TpgbhM1p6xLWey3IonbFR39SEjph27lVGplJqqdj06/vDXTUHUmmNjxdHoeBmKoDu7yDd/wwuk
xTUIS39MftKoPRxfyxUYDDhWKy3I6I0QTVj69cIpUBqNheiCMOftV9RzWv9dIRrc0QAAANZBmgdJ
4QpSZTBREsK//jhAADDgSLnZXEkCJZ7nNjWhTtWCQAfU63q0zbMus+opdzx1yMUq+K87C9BDhFQH
ohDn8RCR9ZnNVhVjLXHiIizfmCC29jLRt4YK/A0liDPPGmgV02/vmud4YuNG0Tn/+FEh8RANCyW7
6V6x9M2MXJgjlZRlV1Lbzs0ePOSwu4clxn4pKT48VGJ4iX2ycznclLGljNTTqxFyzfcihl4C3qUh
muCqlv95qg8T0Vn5NTWg5Vv/9F4BpFRJLsyyjIDeyV9HEP9vBTe7AAAApwGeJmpCvwACn1XcQAAS
5LDOrvmaiKOuJ/gdYdF8+NKhDfNbH/kjj3qd0p9vUzAsKz8JYXSt3VFoDQu3aKKHpDbpHQaOBLAg
5RXCzHbANzREfDZHxcbOGtonq3roZ51hPHtdAloGfWWcXC3lN4lcsgLH423dCwuL8Vz3BTz6mu34
Y9lqhZyPI3I9nkAxpvNHZ2knPT2EshQxBedUyjxPL83oY29S9R8pAAALlm1vb3YAAABsbXZoZAAA
AAAAAAAAAAAAAAAAA+gAACcQAAEAAAEAAAAAAAAAAAAAAAABAAAAAAAAAAAAAAAAAAAAAQAAAAAA
AAAAAAAAAAAAQAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAIAAArAdHJhawAAAFx0a2hk
AAAAAwAAAAAAAAAAAAAAAQAAAAAAACcQAAAAAAAAAAAAAAAAAAAAAAABAAAAAAAAAAAAAAAAAAAA
AQAAAAAAAAAAAAAAAAAAQAAAAAGwAAABIAAAAAAAJGVkdHMAAAAcZWxzdAAAAAAAAAABAAAnEAAA
BAAAAQAAAAAKOG1kaWEAAAAgbWRoZAAAAAAAAAAAAAAAAAAAKAAAAZAAVcQAAAAAAC1oZGxyAAAA
AAAAAAB2aWRlAAAAAAAAAAAAAAAAVmlkZW9IYW5kbGVyAAAACeNtaW5mAAAAFHZtaGQAAAABAAAA
AAAAAAAAAAAkZGluZgAAABxkcmVmAAAAAAAAAAEAAAAMdXJsIAAAAAEAAAmjc3RibAAAALNzdHNk
AAAAAAAAAAEAAACjYXZjMQAAAAAAAAABAAAAAAAAAAAAAAAAAAAAAAGwASAASAAAAEgAAAAAAAAA
AQAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAABj//wAAADFhdmNDAWQAFf/hABhnZAAV
rNlBsJaEAAADAAQAAAMAoDxYtlgBAAZo6+PLIsAAAAAcdXVpZGtoQPJfJE/FujmlG88DI/MAAAAA
AAAAGHN0dHMAAAAAAAAAAQAAAMgAAAIAAAAAFHN0c3MAAAAAAAAAAQAAAAEAAAVYY3R0cwAAAAAA
AACpAAAABAAABAAAAAABAAAGAAAAAAEAAAIAAAAAAQAABAAAAAABAAAIAAAAAAIAAAIAAAAAAgAA
BAAAAAABAAAKAAAAAAEAAAQAAAAAAQAAAAAAAAABAAACAAAAAAEAAAYAAAAAAQAAAgAAAAACAAAE
AAAAAAEAAAYAAAAAAQAAAgAAAAACAAAEAAAAAAEAAAYAAAAAAQAAAgAAAAABAAAGAAAAAAEAAAIA
AAAAAQAABgAAAAABAAACAAAAAAIAAAQAAAAAAQAABgAAAAABAAACAAAAAAEAAAgAAAAAAgAAAgAA
AAABAAAEAAAAAAEAAAgAAAAAAgAAAgAAAAABAAAEAAAAAAEAAAYAAAAAAQAAAgAAAAACAAAEAAAA
AAEAAAYAAAAAAQAAAgAAAAABAAAGAAAAAAEAAAIAAAAAAgAABAAAAAABAAAGAAAAAAEAAAIAAAAA
AQAACAAAAAACAAACAAAAAAEAAAQAAAAAAQAACgAAAAABAAAEAAAAAAEAAAAAAAAAAQAAAgAAAAAB
AAAGAAAAAAEAAAIAAAAAAQAACgAAAAABAAAEAAAAAAEAAAAAAAAAAQAAAgAAAAABAAAGAAAAAAEA
AAIAAAAAAgAABAAAAAABAAAIAAAAAAIAAAIAAAAAAQAABgAAAAABAAACAAAAAAEAAAQAAAAAAQAA
CgAAAAABAAAEAAAAAAEAAAAAAAAAAQAAAgAAAAABAAAGAAAAAAEAAAIAAAAAAQAACAAAAAACAAAC
AAAAAAEAAAoAAAAAAQAABAAAAAABAAAAAAAAAAEAAAIAAAAAAQAABAAAAAABAAAGAAAAAAEAAAIA
AAAAAQAACAAAAAACAAACAAAAAAEAAAQAAAAAAQAACgAAAAABAAAEAAAAAAEAAAAAAAAAAQAAAgAA
AAACAAAEAAAAAAEAAAoAAAAAAQAABAAAAAABAAAAAAAAAAEAAAIAAAAAAQAACAAAAAACAAACAAAA
AAEAAAQAAAAAAQAABgAAAAABAAACAAAAAAEAAAgAAAAAAgAAAgAAAAABAAAEAAAAAAEAAAoAAAAA
AQAABAAAAAABAAAAAAAAAAEAAAIAAAAAAQAABAAAAAABAAAKAAAAAAEAAAQAAAAAAQAAAAAAAAAB
AAACAAAAAAEAAAQAAAAAAQAACAAAAAACAAACAAAAAAEAAAYAAAAAAQAAAgAAAAABAAAKAAAAAAEA
AAQAAAAAAQAAAAAAAAABAAACAAAAAAEAAAoAAAAAAQAABAAAAAABAAAAAAAAAAEAAAIAAAAAAQAA
BgAAAAABAAACAAAAAAEAAAoAAAAAAQAABAAAAAABAAAAAAAAAAEAAAIAAAAAAQAACgAAAAABAAAE
AAAAAAEAAAAAAAAAAQAAAgAAAAADAAAEAAAAAAEAAAgAAAAAAgAAAgAAAAABAAAEAAAAAAEAAAgA
AAAAAgAAAgAAAAABAAAGAAAAAAEAAAIAAAAAAQAABAAAAAABAAAGAAAAAAEAAAIAAAAAAQAABAAA
AAABAAAKAAAAAAEAAAQAAAAAAQAAAAAAAAABAAACAAAAAAEAAAgAAAAAAgAAAgAAAAABAAAIAAAA
AAIAAAIAAAAAAQAABAAAAAABAAAGAAAAAAEAAAIAAAAAAQAACgAAAAABAAAEAAAAAAEAAAAAAAAA
AQAAAgAAAAABAAAGAAAAAAEAAAIAAAAAAQAABgAAAAABAAACAAAAAAEAAAYAAAAAAQAAAgAAAAAE
AAAEAAAAAAEAAAgAAAAAAgAAAgAAAAABAAAEAAAAAAEAAAYAAAAAAQAAAgAAAAAcc3RzYwAAAAAA
AAABAAAAAQAAAMgAAAABAAADNHN0c3oAAAAAAAAAAAAAAMgAABIZAAAELgAAA7cAAAQEAAACTwAA
AfIAAAJ6AAACuwAAAdcAAAEXAAACigAAAjoAAAJqAAABMQAAAVEAAAEsAAABvQAAAJkAAAEnAAAB
mAAAAVQAAACeAAABbgAAAZUAAAJkAAAA+wAAAckAAAEKAAABRAAAAMkAAAFKAAABhwAAAlYAAAFt
AAACYQAAAQgAAAFpAAACUgAAAZYAAADoAAAAcwAAAVEAAAH1AAABTAAAAkAAAAIdAAACEAAAAQ4A
AAFUAAAA2gAAAScAAAHHAAACawAAAW8AAAKUAAABQAAAATMAAAI9AAABwQAAAMsAAACqAAAA1gAA
AmAAAAEzAAACyQAAAYYAAAEgAAAA/wAAAX0AAADNAAABaAAAAcgAAAL3AAABmwAAAZUAAAIJAAAB
WwAAAnUAAAJxAAAAzQAAAMEAAADdAAACuQAAARkAAAKJAAABcgAAAVoAAAHdAAABUAAAANsAAACp
AAABhwAAAvMAAAGsAAACjAAAASEAAAE8AAACNgAAAk0AAAEHAAAAnQAAAP0AAAGoAAABngAAApMA
AAG+AAABaQAAATQAAAFyAAABRgAAALMAAAGsAAACvAAAAWAAAAJqAAABQgAAAR4AAAHpAAACHAAA
APYAAACHAAAA7AAAAaMAAAKMAAABnwAAARgAAAE2AAAB1wAAAWkAAAEjAAAA0gAAAlUAAAFEAAAC
+wAAARQAAAGIAAABWgAAAoAAAAF9AAAA+AAAAJIAAAISAAAA2AAAApsAAAGIAAABHwAAASYAAAG0
AAABagAAAMsAAADAAAABYAAAAfAAAAIZAAACdAAAAMAAAAE2AAAB5AAAAbEAAAEYAAAAfAAAAZgA
AAD6AAABfgAAAd8AAAENAAAB1gAAAX4AAAFMAAAAxAAAALEAAALFAAABBQAAAVEAAAImAAAA5AAA
ATQAAAHLAAABNAAAAKoAAAG7AAABBwAAANEAAAEPAAAB8gAAAQoAAAGwAAAA9QAAAP8AAADNAAAB
FgAAAZYAAAGkAAAB6wAAAaYAAAEGAAABNgAAAWoAAADaAAAAqwAAABRzdGNvAAAAAAAAAAEAAAAs
AAAAYnVkdGEAAABabWV0YQAAAAAAAAAhaGRscgAAAAAAAAAAbWRpcmFwcGwAAAAAAAAAAAAAAAAt
aWxzdAAAACWpdG9vAAAAHWRhdGEAAAABAAAAAExhdmY1Ny44My4xMDA=
">
  Your browser does not support the video tag.
</video>




```python

```
