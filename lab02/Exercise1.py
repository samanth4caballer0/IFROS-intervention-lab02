# Import necessary libraries
from lab2_robotics import * # Import our library (includes Numpy)
import matplotlib.pyplot as plt
import matplotlib.animation as anim

# Robot definition (planar 2 link manipulator)
d = np.zeros(2)           # displacement along Z-axis
q = np.array([0.2, 0.5])  # rotation around Z-axis (theta)
a = np.array([0.75, 0.5]) # displacement along X-axis
alpha = np.zeros(2)       # rotation around X-axis 
# print (d)
# print (q)
# print (a)
# print (alpha)


# Simulation params
dt = 0.01 # Sampling time
Tt = 10 # Total simulation time
tt = np.arange(0, Tt, dt) # Simulation time vector

# Drawing preparation
fig = plt.figure()
ax = fig.add_subplot(111, autoscale_on=False, xlim=(-2, 2), ylim=(-2,2))
ax.set_title('Kinematics')
ax.set_xlabel('x[m]')
ax.set_ylabel('y[m]')
ax.set_aspect('equal')
ax.grid()
line, = ax.plot([], [], 'o-', lw=2) # Robot structure
path, = ax.plot([], [], 'r-', lw=1) # End-effector path
plot_limit = 100


# Memory
PPx = []
PPy = [] 
q1 = [] 
q2 = []     
timestamp = [] #just for visualisation of joint angles 

# Simulation initialization
def init():
    line.set_data([], [])
    path.set_data([], [])
    return line, path

# Simulation loop
def simulate(t):
    global d, q, a, alpha
    global PPx, PPy
    
    # Update robot
    T = kinematics(d, q, a, alpha) #transformatrion matrix 
    dq = np.array([0.4, 0.3]) # Define how joint velocity changes with time, (not end effector important!!!)
    q = q + dt * dq                                                                  
    
    # Update drawing
    PP = robotPoints2D(T)
    line.set_data(PP[0,:], PP[1,:])
    PPx.append(PP[0,-1])
    PPy.append(PP[1,-1])
    path.set_data(PPx, PPy)
    
    # Update memory for plotting
    q1.append(q[0])
    q2.append(q[1])
    timestamp.append(t)
    
    return line, path

def plot_summary():
    # Evolution of joint positions Plotting
    fig_joint = plt.figure()
    ax = fig.add_subplot(222, autoscale_on=True)
    ax.set_title('joint positions')
    ax.set_xlabel('Time[s]')
    ax.set_ylabel('Angle[rad]')
    ax.set_aspect('equal')
    ax.grid()
    plt.plot(timestamp,q1,label='q1')
    plt.plot(timestamp,q2,label='q2')
    plt.show()


# Run simulation
animation = anim.FuncAnimation(fig, simulate, tt, 
                                interval=10, blit=True, init_func=init, repeat=False)

plt.show()
# Plot summary after close animation
plot_summary()