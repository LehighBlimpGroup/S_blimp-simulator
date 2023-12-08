# Spinning Blimp simulator
Spinning Blimp simulator for Paper (reviewing): 
[IEEE ICRA 2024] The Spinning Blimp: Design and Control of a Novel Minimalist Aerial Vehicle Leveraging Spinning Dynamics and Locomotion. Leonardo Santens, Shuhang Hou, Diego S. D’Antonio, and David Saldana

Spinning Blimp Robot: a constantly spinning blimp robot with two motors with propellors and two wings to move in 3D space.
Here we use Jupyter Notebook to build the simulator environment with Pylab. The inputs of the robot are just f1 and f2, which means the motor force of Motor1 and Motor2, and the outputs are the XYZ plots VS time, f1 and f2 VS time,  

Please try the 
# waypoints following.ipynb 
first if you are not famillar with these simulator.

Other .ipynb files are testing settings for the paper. You can try to run the simulator in different robot settings(motor force, tau, weight and size of the robot...), or different tasks(go to waypoints, follow a trajectory...).

The Controller folder contains the real controller files (send commands to robots in real time) we used in the paper. You can take that as reference.
