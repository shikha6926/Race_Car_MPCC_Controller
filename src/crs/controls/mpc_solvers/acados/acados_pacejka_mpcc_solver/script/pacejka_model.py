import math
import numpy as np
from casadi import *
from typing import Dict
import yaml
from yaml.loader import SafeLoader

def pacejka_model(constraints: Dict[str, float]):

    # define structs
    model = types.SimpleNamespace()
    constraint = types.SimpleNamespace()
    model_name = "pacejka_model"

    """ constraints """
    model.x_min = constraints["x_min"]
    model.x_max = constraints["x_max"]
    model.y_min = constraints["y_min"]
    model.y_max = constraints["y_max"]
    model.yaw_min = constraints["yaw_min"]
    model.yaw_max = constraints["yaw_max"]
    model.vx_min = constraints["vx_min"]
    model.vx_max = constraints["vx_max"]
    model.vy_min = constraints["vy_min"]
    model.vy_max = constraints["vy_max"]
    model.dyaw_min = constraints["dyaw_min"]
    model.dyaw_max = constraints["dyaw_max"]
    model.delta_min = constraints["delta_min"]
    model.delta_max = constraints["delta_max"]
    model.T_min = constraints["T_min"]
    model.T_max = constraints["T_max"]
    model.theta_min = constraints["theta_min"]
    model.theta_max = constraints["theta_max"]
    model.ddelta_min = constraints["ddelta_min"]
    model.ddelta_max = constraints["ddelta_max"]
    model.dT_min = constraints["dT_min"]
    model.dT_max = constraints["dT_max"]
    model.dtheta_min = constraints["dtheta_min"]
    model.dtheta_max = constraints["dtheta_max"]
    
    """ Dynamics """
    # CasADi - states
    xp = MX.sym("xp")
    yp = MX.sym("yp")
    yaw = MX.sym("yaw")
    vx = MX.sym("vx")
    vy = MX.sym("vy")
    omega = MX.sym("omega")
    T = MX.sym("T")
    delta = MX.sym("delta")
    theta = MX.sym("theta")
    x = vertcat(xp, yp, yaw, vx, vy, omega, T, delta, theta)

    # CasADi - input
    dT = MX.sym("dT")
    ddelta = MX.sym("ddelta")
    dtheta = MX.sym("dtheta")
    u = vertcat(dT, ddelta, dtheta)

    # xdot
    xpdot = MX.sym("xpdot")
    ypdot = MX.sym("ypdot")
    yawdot = MX.sym("yawdot")
    vxdot = MX.sym("vxdot")
    vydot = MX.sym("vydot")
    omegadot = MX.sym("omegadot")
    xdot = vertcat(xpdot, ypdot, yawdot, vxdot, vydot,
                   omegadot, dT, ddelta, dtheta)

    # algebraic variables
    z = vertcat([])

    # define params
    xd = MX.sym("xd")
    yd = MX.sym("yd")
    grad_xd = MX.sym("grad_xd")
    grad_yd = MX.sym("grad_yd")
    # theta_hat = MX.sym("theta_hat")
    phi_d = MX.sym("phi_d")
    Q1 = MX.sym("Q1")
    Q2 = MX.sym("Q2")
    R1 = MX.sym("R1")
    R2 = MX.sym("R2")
    R3 = MX.sym("R3")
    q = MX.sym("q")
    lr = MX.sym("lr")
    lf = MX.sym("lf")
    m = MX.sym("m")
    I = MX.sym("I")
    Df = MX.sym("Df")
    Cf = MX.sym("Cf")
    Bf = MX.sym("Bf")
    Dr = MX.sym("Dr")
    Cr = MX.sym("Cr")
    Br = MX.sym("Br")
    Cm1 = MX.sym("Cm1")
    Cm2 = MX.sym("Cm2")
    Cd = MX.sym("Cd")
    Croll = MX.sym("Croll")

    # parameters
    # p = vertcat(xd, yd, grad_xd, grad_yd, theta_hat, phi_d, Q1, Q2, R1,
    #             R2, R3, q, lr, lf, m, I, Df, Cf, Bf, Dr, Cr, Br, Cm1, Cm2, Cd, Croll)
    p = vertcat(Q1, Q2, R1, R2, R3, q, lr, lf, m, I, Df, Cf, Bf, Dr, Cr, Br, Cm1, Cm2, Cd, Croll)

    # dynamics
    Fx = (Cm1 - Cm2 * vx) * T - Cd * vx * vx - Croll
    beta = atan(vy / vx)
    ar = -beta + lr * omega / vx
    af = delta - beta - lf * omega / vx
    Fr = Dr * sin(Cr * atan(Br * ar))
    Ff = Df * sin(Cf * atan(Bf * af))
    f_expl = vertcat(
        vx * cos(yaw) - vy * sin(yaw),
        vx * sin(yaw) + vy * cos(yaw),
        omega,
        1 / m * (Fx - Ff * sin(delta) + m * vy * omega),
        1 / m * (Fr + Ff * cos(delta) - m * vx * omega),
        1 / I * (Ff * lf * cos(delta) - Fr * lr),
        dT,
        ddelta,
        dtheta,
    )

    #Load coefficients of spline from yaml file
    with open('/code/src/crs/controls/mpc_solvers/acados/acados_pacejka_mpcc_solver/script/fulltrack_spline.yaml', 'r') as f:
        data = yaml.load(f, Loader=SafeLoader)
    
    # Convert data to [x_a[spline_idx], x_b[spline_idx], x_c[spline_idx], x_d[spline_idx]]
    x_coef = np.vstack((data['X_a'], data['X_b'], data['X_c'], data['X_d'])).T
    y_coef = np.vstack((data['Y_a'], data['Y_b'], data['Y_c'], data['Y_d'])).T
    x_coef = x_coef.tolist()
    y_coef = y_coef.tolist()

    # Casadi variables to extract coefficients
    spline_x_coef = MX(DM(x_coef))
    spline_y_coef = MX(DM(y_coef))

    # Get spline index based on density i.e 3.0 and distance on track(s)
    density = 2.993
    idx = floor(fmod(theta, 13.28152)  * density)
    # Extract spline coefficien based on spline idx
    coef_xd = spline_x_coef[idx, :].T
    coef_yd = spline_y_coef[idx, :].T

    # Get desired x,y coordinate of center line of track from spline equations
    # xd = dot(coef_xd, (theta_hat - (idx * 0.334))**DM(range(4)))
     # yd = dot(coef_yd, (theta_hat - (idx * 0.334))**DM(range(4)))
    # x_grad = dot(coef_xd[1:4], (( theta_hat - (idx * 0.334))**DM(range(3)))*DM(range(1, 4)))
    # y_grad = dot(coef_yd[1:4], ((theta_hat - (idx * 0.334))**DM(range(3)))*DM(range(1, 4)))

    distance = fmod(theta, 13.28152) -  (idx * 0.334)
    xd = coef_xd[0] + (coef_xd[1] * distance) + (coef_xd[2] * (distance**2)) + (coef_xd[3] * (distance**3))
    yd = coef_yd[0] + (coef_yd[1] * distance) + (coef_yd[2] * (distance**2)) + (coef_yd[3] * (distance**3))
    grad_xd = coef_xd[1] + (2 * coef_xd[2] * distance) + (3 * coef_xd[3] * distance**2)
    grad_yd = coef_yd[1] + (2 * coef_yd[2] * distance) + (3 * coef_yd[3] * distance**2)
    phi_d = casadi.arctan2(grad_yd, grad_xd) + floor(theta / 13.28152) * 2 * pi

    # Casadi function input: predicted distance_on_track, output: desired x,y on center line of track
    get_xref = Function('get_xref', [theta], [xd], ['theta_hat'], ['xref'])
    get_yref = Function('get_yref', [theta], [yd], ['theta_hat'], ['yref'])
    get_phiref = Function('get_phiref', [theta], [phi_d], ['theta_hat'], ['phiref'])
    
    # cost
    eC = sin(get_phiref(theta)) * (xp - get_xref(theta)) - cos(get_phiref(theta)) * (yp - get_yref(theta))
    eL = -cos(get_phiref(theta)) * (xp -  get_xref(theta)) - sin(get_phiref(theta)) * (yp - get_yref(theta))

    c_eC = eC*eC*Q1
    c_eL = eL*eL*Q2
    c_theta = -q*theta
    c_dT = dT*dT*R1
    c_ddelta = ddelta*ddelta*R2
    c_dtheta = dtheta*dtheta*R3

    model.cost_expr_ext_cost = c_eC  + c_theta + c_dT + c_ddelta + c_dtheta

    # nonlinear track constraints
    # inner and outer track boundary given width of track
    # track_width = 0.85
    # r = track_width / 2
    # x_pos_out = xd + r * (-grad_yd)
    # y_pos_out = yd + r * grad_xd

    # x_pos_in = xd - r * (-grad_yd)
    # y_pos_in = yd - r * grad_xd

    # # Compute lower and upper track bounds

    # track_constraint_lower = (-grad_yd) * x_pos_in + grad_xd * y_pos_in 
    # track_constraint_upper = (-grad_yd) * x_pos_out + grad_xd * y_pos_out
    
    radius_sq = (xp - get_xref(theta))*(xp - get_xref(theta)) + (yp - get_yref(theta))*(yp - get_yref(theta))
    constraint.expr = vertcat(eL)

    params = types.SimpleNamespace()
    # params.xd = xref
    # params.yd = yref
    # params.grad_xd = grad_xref
    # params.grad_yd = grad_yref
    # params.phi_d = phiref
    # params.theta_hat = theta_hat
    params.Q1 = Q1
    params.Q2 = Q2
    params.R1 = R1
    params.R2 = R2
    params.R3 = R3
    params.q = q
    params.lr = lr
    params.lf = lf
    params.m = m
    params.I = I
    params.Df = Df
    params.Cf = Cf
    params.Bf = Bf
    params.Dr = Dr
    params.Cr = Cr
    params.Br = Br
    params.Cm1 = Cm1
    params.Cm2 = Cm2
    params.Cd = Cd
    params.Croll = Croll

    model.f_impl_expr = xdot - f_expl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    model.z = z
    model.p = p
    model.name = model_name
    model.params = params

    # return model, constraint
    return model, constraint