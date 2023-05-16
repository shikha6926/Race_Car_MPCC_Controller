import argparse
import os
import numpy as np
import rospkg
import yaml

from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
from pacejka_safety_model import safety_dynamic_model


def build_acados_solver(N: int, Ts: float, acados_source_path: str, model):
    """
    Creates the required C-Code for the acados solver

    Parameters
    ----------
    N : int
        Time Horizon
    Ts : float
        Sampling Time
    acados_source_path : str
        Path to acados source location
    model: Callable
        Model function e.g. pacejka_model

    Returns
    -------
    None
    """
    # create render arguments
    ocp = AcadosOcp()

    # export model
    model, constraint = model()

    # define acados ODE
    model_ac = AcadosModel()
    model_ac.f_impl_expr = model.f_impl_expr
    model_ac.f_expl_expr = model.f_expl_expr
    model_ac.x = model.x
    model_ac.xdot = model.xdot
    model_ac.u = model.u
    model_ac.z = model.z
    model_ac.p = model.p
    model_ac.name = model.name


    # define constraint
    model_ac.con_h_expr = constraint.expr
    model_ac.con_h_expr_e = constraint.expr_e

    ocp.model = model_ac

    # set dimensions
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = 4

    ocp.dims.nx = nx
    ocp.dims.np = model.p.size()[0]
    ocp.dims.ny = ny
    ocp.dims.nbx = 3
    ocp.dims.nbx_0 = nx
    ocp.dims.nsbx = 1

    ocp.dims.nbu = nu
    ocp.dims.nu = nu
    ocp.dims.N = N
    ocp.dims.nh = 2
    ocp.dims.nsh = 2    
    ocp.dims.ns = ocp.dims.nsbx + ocp.dims.nsh   # total number of slacks

    # number of nonlinear terminal constraints (safe position)
    ocp.dims.nh_e = 1
    ocp.dims.nsh_e = 1      # number of soft nonlinear terminal constraint
    ocp.dims.ns_e = 1      # total number of slacks at terminal


    # number of parameters (xtrack, ytrack, + terminal params)
    ocp.dims.np = model.p.size()[0]

    # COST ----------------------------------------------
    ocp.cost.cost_type = "LINEAR_LS"
    #  - will set all except first stage W to zero before solve time
    ocp.cost.W = np.zeros((4, 4))

    # selection matrices for x and u vectors:
    Vx = np.zeros((ny, nx))
    Vx[0, 6] = 1.0
    Vx[1, 7] = 1.0

    Vu = np.zeros((ny, nu))
    Vu[2, 0] = 1.0
    Vu[3, 1] = 1.0
    ocp.cost.Vx = Vx
    ocp.cost.Vu = Vu

    # set intial references:
    #  - yref will be later be set as the learning input u_l
    ocp.cost.yref = np.array([0., 0., 0., 0.])

    # set slack costs for soft path constraints
    ocp.cost.zl = 5e1 * np.ones((ocp.dims.ns,))
    ocp.cost.zu = 5e1 * np.ones((ocp.dims.ns,))
    ocp.cost.Zl = 5e1 * np.ones((ocp.dims.ns,))
    ocp.cost.Zu = 5e1 * np.ones((ocp.dims.ns,))
    ocp.cost.zl_e = 5e1 * np.ones((ocp.dims.ns_e,))
    ocp.cost.zu_e = 5e1 * np.ones((ocp.dims.ns_e,))
    ocp.cost.Zl_e = 5e1 * np.ones((ocp.dims.ns_e,))
    ocp.cost.Zu_e = 5e1 * np.ones((ocp.dims.ns_e,))

    # setting constraints
    ocp.constraints.lbx = np.array(     # lower bounds on x
        [
            model.vx_min,
            model.T_min,
            model.delta_min, 
        ]
    )

    ocp.constraints.ubx = np.array(     # upper bounds on x
        [ 
            model.vx_max,
            model.T_max,
            model.delta_max,
        ]
    )

    ocp.constraints.idxbx = np.array([3, 6, 7])   # indices of bounds on x
    ocp.constraints.idxsbx = np.array([0])

    ocp.constraints.lbu = np.array(     # lower bounds on u
        [
            model.dT_min,
            model.ddelta_min,
        ]
    )
    ocp.constraints.ubu = np.array(     # upper bounds on u
        [
            model.dT_max,
            model.ddelta_max,
        ]
    )
    ocp.constraints.idxbu = np.array([0, 1])  # indices of bounds on u

    # halfspace track constraints
    # placeholder, the bounds below are set by c++ node each solve
    track_radius = model.constants.track_width/2
    ocp.constraints.lh = np.array([-track_radius, -track_radius])
    ocp.constraints.uh = np.array([track_radius,  track_radius])
    ocp.constraints.lsh = np.zeros(ocp.dims.nsh)
    ocp.constraints.ush = np.zeros(ocp.dims.nsh)
    ocp.constraints.idxsh = np.array([0, 1])

    # terminal constraints
    ocp.constraints.lh_e = np.array([0.0, 0.3])
    ocp.constraints.uh_e = np.array([50.0, 1.0])
    ocp.constraints.lsh_e = np.zeros(ocp.dims.nsh_e)
    ocp.constraints.ush_e = np.zeros(ocp.dims.nsh_e)
    ocp.constraints.idxsh_e = np.array([0])

    # set intial condition
    ocp.constraints.x0 = np.zeros(nx)
    ocp.constraints.idxbx_0 = np.array([0, 1, 2, 3, 4, 5, 6, 7])
    ocp.parameter_values = np.zeros(ocp.dims.np)

    # set QP solver and integration
    ocp.solver_options.Tsim = Ts
    ocp.solver_options.tf = Ts * N
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"


    ocp.solver_options.sim_method_num_stages = 1
    ocp.solver_options.sim_method_num_steps = 2
    ocp.solver_options.print_level = 0

    ocp.solver_options.tol = 1e-4

    ocp.acados_include_path = acados_source_path + '/include'
    ocp.acados_lib_path = acados_source_path + '/lib'

    # create solver with agent specific code files
    filename = "c_generated_code/acados_pacjeka_safety_model_solver_config.json"
    acados_solver = AcadosOcpSolver(ocp, json_file=filename)



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Create a new Acados safety solver for the pacejka model')
    parser.add_argument('--config', type=str, help='Name of the configuration file', default="solver.yaml")
    parser.add_argument('--acados_source', type=str, help='Path to the acados source', default= os.environ['ACADOS_SOURCE_DIR'])
    args = parser.parse_args()

    rospack = rospkg.RosPack()
    controller_path = rospack.get_path('acados_pacejka_safety_model_solver')
    os.chdir( controller_path + "/src" )

    with open(f"../config/{args.config}") as f:
        cfg = yaml.load(f, Loader=yaml.loader.SafeLoader)

        def loadModel():
            return safety_dynamic_model(cfg["model_bounds"], cfg["track_constraints"])

        build_acados_solver(cfg["solver_creation"]["N"], cfg["solver_creation"]["Ts"], args.acados_source, loadModel)