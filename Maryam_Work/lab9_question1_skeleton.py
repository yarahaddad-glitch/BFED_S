# Required libraries
import numpy as np
from scipy import linalg
import matplotlib.pyplot as plt
from spatialmath.base import *
from roboticstoolbox import models, trapezoidal

# Useful variables
from math import pi

# -----------------------------------------------------------------------------------#
def lab9_question1_skeleton():
    # 1.1) Set parameters for the simulation
    p560 = models.DH.Puma560()  # Load robot model
    t = ...  # Total time (s)
    steps = ...  # No. of steps
    delta_t = t/steps  # Discrete time step
    delta = 2*pi/steps  # Small angle change
    epsilon = ...  # Threshold value for manipulability/Damped Least Squares
    W = np.diag([...])  # Weighting matrix for the velocity vector

    # 1.2) Allocate array data
    m = np.zeros([steps,1])  # Array for Measure of Manipulability
    q_matrix = np.zeros([steps,6])  # Array for joint angles
    qdot = np.zeros([steps,6])  # Array for joint velocities
    theta = np.zeros([3,steps])  # Array for roll-pitch-yaw angles
    x = np.zeros([3,steps])  # Array for x-y-z trajectory
    position_error = np.zeros([3,steps])  # For plotting trajectory error
    angle_error = np.zeros([3,steps])  # For plotting trajectory error

    # 1.3) Set up trajectory, initial pose
    s = trapezoidal(0,1,steps).q  # Trapezoidal trajectory scalar
    for i in range(steps):
        x[0,i] = (1-s[i])*0.35 + s[i]*0.35  # Points in x
        x[1,i] = (1-s[i])*-0.55 + s[i]*0.55  # Points in y
        x[2,i] = 0.5 + 0.2*np.sin(i*delta)  # Points in z
        theta[0,i] = 0  # Roll angle
        theta[1,i] = 5*pi/9  # Pitch angle
        theta[2,i] = 0  # Yaw angle

    T = ...  # Create transformation of first point and angle
    q0 = ...  # Initial guess for joint angles
    q_matrix[0,:] = ...  # Solve joint angles to achieve first waypoint
    qlim = np.transpose(p560.qlim)

    # 1.4) Track the trajectory with RMRC
    for i in range(steps-1):
        T = ...  # Get forward transformation at current joint state
        delta_x = ...  # Get position error from next waypoint
        Rd = rpy2r(...)  # Get next RPY angles, convert to rotation matrix
        Ra = ...  # Current end-effector rotation matrix

        Rdot = (1/delta_t)*(Rd - Ra)  # Calculate rotation matrix error (See RMRC lectures)
        S = ...  # Skew-symmetric matrix S(\omega)
        linear_velocity = (1/delta_t)*delta_x
        angular_velocity = np.array([S[...,...],
                                     S[...,...],
                                     S[...,...]])  # Check the structure of skew-symmetric matrix!!

        delta_r = ...  # Calculate rotation matrix error
        delta_theta = tr2rpy(...)  # Convert rotation matrix to RPY angles
        xdot = ...  # Calculate end-effector velocity to reach next waypoint.
        # Try using a weighting matrix to (de)emphasize certain dimensions

        J = ...  # Get Jacobian at current joint state
        if ...:  # If manipulability is less than given threshold
            m_lambda = (1 - m[i]/epsilon) * 0.05  # Damping coefficient (try scaling it)
            inv_j = ...  # Apply Damped Least Squares pseudoinverse
        else:
            inv_j = ...  # Don't use DLS

        qdot[i,:] =  # Solve the RMRC equation (you may need to transpose the vector)
        for ...:  # Loop through joints 1 to 6
            if ...:  # If next joint angle is lower than joint limit...
                ...  # Stop the motor
            elif ...:  # If next joint angle is greater than joint limit ...
                ...  # Stop the motor

        q_matrix[i+1,:] = ...  # Update next joint state based on joint velocities
        m[i] = ...  # Record manipulability
        position_error[:,i] = x[:,i+1] - T[:3,3]  # For plotting
        angle_error[:,i] = delta_theta  # For plotting

    # 1.5) Plot the results
    fig = p560.plot(q_matrix[0,:], limits=[-1,1,-1,1,0,1])
    fig.ax.plot(x[0,:], x[1,:], x[2,:], 'k.', linewidth=0.2)
    for q in q_matrix:
        p560.q = q
        pos = p560.fkine(q).A[:3,3]
        fig.ax.plot(pos[0], pos[1], pos[2], 'r.', markersize=0.5)
        fig.step(delta_t)

    for i in range(6):
        plt.figure(2)
        plt.subplot(3,2,i+1)
        plt.plot(q_matrix[:,i], 'k', linewidth=1)
        plt.title(f'Joint {i+1}')
        plt.ylabel('Angle (rad)')
        plt.axhline(qlim[i,0], color='r')
        plt.axhline(qlim[i,1], color='r')

        plt.figure(3)
        plt.subplot(3,2,i+1)
        plt.plot(qdot[:,i], 'k', linewidth=1)
        plt.title(f'Joint {i+1}')
        plt.ylabel('Velocity (rad/s)')
        plt.axhline(0, color='r')

    plt.figure(4)
    plt.subplot(2,1,1)
    plt.plot(position_error.T * 1000, linewidth=1)
    plt.axhline(0, color='k')
    plt.xlabel('Step')
    plt.ylabel('Position Error (mm)')
    plt.legend(['X-Axis', 'Y-Axis', 'Z-Axis'])

    plt.subplot(2,1,2)
    plt.plot(angle_error.T, linewidth=1)
    plt.axhline(0, color='k')
    plt.xlabel('Step')
    plt.ylabel('Angle Error (rad)')
    plt.legend(['Roll','Pitch','Yaw'])

    plt.figure(5)
    plt.plot(m, 'k', linewidth=1)
    plt.axhline(epsilon)
    plt.title('Manipulability')

    input('Enter to continue\n')
# ---------------------------------------------------------------------------------------#
if __name__ == "__main__":
    lab9_question1_skeleton()