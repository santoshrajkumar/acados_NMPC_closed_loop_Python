from acados_template import AcadosModel
from casadi import SX, vertcat, diagcat, mtimes, sin, cos, transpose 


def inv_pendulum_model():
    # System dimensions
    nstates = 2
    nctrl = 1

    # System parameters
    g = 9.81  # acceleration due to gravity
    l = 4     # length of the pendulum
    d = 3.5   # length of the pendulum at which the
    m = 0.15  # mass
    k = 0.3   # stiffness
    c = 0.05  # damping coefficient

    # System states & input
    x= SX.sym('x', nstates)  # x = [theta; omega]
    xdot = SX.sym('xdot', nstates)  # xdot
    u = SX.sym('u', nctrl)  # input M

    # Define dynamics (f(x))
    part1 = -(k * d**2 / (m * l**2)) * sin(x[0]) * cos(x[0])
    part2 = -(c / (m * l**2)) * d**2 * (cos(x[0])**2) * x[1]
    part3 = (g / l) * sin(x[0]) + (u[0] / (m * l**2))

    dyn_expr_f_expl = vertcat(x[1], part1 + part2 + part3)  # f(x)

    # Define Cost Function
    Q_x = diagcat(1e3, 1e3)  # weighting matrix for states
    R_u = 0.1  # weighting matrix for input

    cost_expr_ext_cost_e = 0.5 * mtimes(transpose(x), mtimes(Q_x, x))  # state cost
    cost_expr_ext_cost = cost_expr_ext_cost_e + 0.5 * mtimes(transpose(u), R_u * u)  # input cost
    
    
    
    model = AcadosModel()
    
    # populate structure of the model
    model.name = 'inv_pendulum_ode'
    model.nstates = nstates;
    model.u= u;
    model.x = x;
    model.xdot = xdot;
    model.u = u;
    model.f_expl_expr = dyn_expr_f_expl;
    
    model.cost_expr_ext_cost = cost_expr_ext_cost;
    model.cost_expr_ext_cost_e = cost_expr_ext_cost_e;

    return model

