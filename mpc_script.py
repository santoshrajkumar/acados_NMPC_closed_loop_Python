from setup_acados import setup_ocp, setup_integrator
import numpy as np
import matplotlib.pyplot as plt


"""
This section set ups the OCP and the Integrator
"""

# initial conditions
theta_0 = 35; #degrees
theta_dot_0 = -5.73 # deg/sec
x0 = np.array([[np.deg2rad(theta_0)], [np.deg2rad(theta_dot_0)]])

# constraints on the states
theta_lb, theta_ub = -40, 40 #degrees
theta_dot_lb, theta_dot_ub = -11.46, 11.46 # degree/sec
lbx = np.array([np.deg2rad(theta_lb), np.deg2rad(theta_dot_lb)]) # lower bound in vector form
ubx = np.array([np.deg2rad(theta_ub), np.deg2rad(theta_dot_ub)]) # upper bound in vector form

# constraints on control
lbu = np.array([-2]) #lower bound , N-m
ubu = np.array([2])  #upper bound , N-m



# OCP & Integrator Parameters
ctrlDt = 0.1  # sec, time interval b/w each MPC division
mpc_horizon = 0.5  # sec, time horizon of the MPC scheme
mpc_div = int(mpc_horizon / ctrlDt)  # # of divisions of the MPC horizon
simDt = 0.05  # sec, sampling time of the plant/integrator

# construct the acados OCP solver
ocp_solver = setup_ocp(mpc_div, mpc_horizon,x0,lbx,ubx,lbu,ubu)

# construct the integrator for the pendulum system
integrator = setup_integrator(simDt)

# number of states and control
nstates = ocp_solver.acados_ocp.dims.nx
nctrl = ocp_solver.acados_ocp.dims.nu


"""
This section runs the simulation
"""

tsim = 10 # total simulation time
simX = x0.T # storage variable for the states
Umpc = np.zeros((1, nctrl)) # # storage variable for the control

time = 0 # time stamp for each MPC step
time_vec = [0] # time vector containing timestamps for the integrator
comptime = [] # storage variable for computing MPC computation time at each step

# Simulation loop
while time < tsim - ctrlDt:
    # solve for the MPC problem
    u_mpc = ocp_solver.solve_for_x0(x0_bar=simX[-1, :])
    comptime =  np.append(comptime, ocp_solver.get_stats('time_tot')) # store computation time
    
    # use the computed control in the integrator
    j = 0 # integrator step
    while j < int(ctrlDt / simDt):
        # Simulate system
        simXtemp = integrator.simulate(x=simX[-1, :], u=u_mpc)
        simX = np.vstack([simX, simXtemp])
        Umpc = np.vstack([Umpc, u_mpc])
        j += 1
        time_vec = np.append(time_vec, time_vec[-1] + simDt)
    
    time += ctrlDt

        

"""
This section plots the results
"""

fig, axs = plt.subplots(nstates + 1, 1,figsize=(8, 8), sharex=True)

# Plot each state in a separate subplot
for i in range(nstates):
    if i == 0:
        axs[i].axhline(y=40, color='black', linestyle='--', linewidth=1, label='Bounds')
        axs[i].axhline(y=-40, color='black', linestyle='--', linewidth=1)
        axs[i].set_ylabel('$\Theta^0$')
    else:
        axs[i].axhline(y=11.46, color='black', linestyle='--', linewidth=1, label='Bounds')
        axs[i].axhline(y=-11.46, color='black', linestyle='--', linewidth=1)
        axs[i].set_ylabel('$\dot{\Theta}$  $^0/sec$')
        
    axs[i].plot(time_vec, np.rad2deg(simX[:, i]), color=f'C{i+2}')

    
    axs[i].grid(True)
    axs[i].legend()
    axs[i].set_xlim(left=0,right=tsim)

# Plot control signal in the last subplot
for j in range(1):
    axs[-1].axhline(y=2, color='black', linestyle='--', linewidth=1, label='Bounds')
    axs[-1].axhline(y=-2, color='black', linestyle='--', linewidth=1)
    axs[-1].plot(time_vec[:-1], Umpc[1:, j], linewidth=2, color=f'C{j + 1}')

axs[-1].set_xlabel('Time [s]')
axs[-1].set_ylabel('$u$ (N-m)')
axs[-1].grid(True)
axs[-1].legend()
plt.draw()
plt.savefig('output_figure.pdf', format='pdf')
