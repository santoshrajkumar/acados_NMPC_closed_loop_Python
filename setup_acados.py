from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSim, AcadosSimSolver
from ode_model import inv_pendulum_model
import numpy as np



def setup_ocp(mpc_div,mpc_horizon,x0,lbx,ubx,lbu,ubu):
    
    # create ocp object to formulate the OCP
    ocp=AcadosOcp()
    
    # define the set model
    model=inv_pendulum_model()
    ocp.model=model # atatch the model to the ocp
    ocp.dims.N = mpc_div  # horizon divisions
    ocp.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.nlp_solver_type = 'SQP'
    ocp.solver_options.nlp_solver_max_iter = 100
    ocp.solver_options.tf = mpc_horizon # mpc_horizon length
    ocp.solver_options.nlp_solver_max_iter = 100
    ocp.solver_options.nlp_solver_exact_hessian = False
    ocp.solver_options.regularize_method = 'PROJECT_REDUC_HESS'
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 1
    
    
    # Initial constraint
    ocp.constraints.constr_type = 'BGH'
    ocp.constraints.x0 = x0
    
    # Set constraints for states and control
    ocp.constraints.lbx = lbx
    ocp.constraints.ubx = ubx
    ocp.constraints.idxbx = np.array([0, 1])  # Indices for state constraints
    ocp.constraints.lbu = lbu
    ocp.constraints.ubu = ubu
    ocp.constraints.idxbu = np.array([0])
    
    # Define cost function in OCP
    ocp.cost.cost_type = 'EXTERNAL'
    ocp.cost.cost_type_e = 'EXTERNAL'
    ocp.cost.cost_expr_ext_cost = model.cost_expr_ext_cost
    ocp.cost.cost_expr_ext_cost_e = model.cost_expr_ext_cost_e
    solver_json = 'acados_ocp_' + model.name + '.json'
    acados_ocp_solver = AcadosOcpSolver(ocp, json_file = solver_json)
    

    return acados_ocp_solver
    
def setup_integrator(Tf):
    # Create the simulation object, which links the model to the solver
    acados_sim = AcadosSim()

    # Attach the model to the simulation object
    acados_sim.model = inv_pendulum_model()
    
    # Set simulation-specific options
    acados_sim.solver_options.sim_method_num_stages = 4  # Number of stages in the integrator
    acados_sim.solver_options.sim_method_num_steps = 1   # Steps per stage
    acados_sim.solver_options.integrator_type = 'ERK'    # Explicit Runge-Kutta method
    acados_sim.solver_options.T = Tf  # Time step for integration

    # Create the simulation solver object
    sim_solver = AcadosSimSolver(acados_sim)



    return sim_solver
    
    
    
    
