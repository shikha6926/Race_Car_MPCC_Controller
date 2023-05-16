import math
import numpy as np
from casadi import *
from typing import Dict

def pacejka_model_tracking_mpc(constraints: Dict[str, float]):

    # define structs
    model = types.SimpleNamespace()
    constraint = types.SimpleNamespace()
    model_name = "pacejka_model_tracking_mpc"

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
    model.ddelta_min = constraints["ddelta_min"]
    model.ddelta_max = constraints["ddelta_max"]
    model.dT_min = constraints["dT_min"]
    model.dT_max = constraints["dT_max"]


    """ Dynamics """
    # CasADi - states
    xp = SX.sym("xp")
    yp = SX.sym("yp")
    yaw = SX.sym("yaw")
    vx = SX.sym("vx")
    vy = SX.sym("vy")
    omega = SX.sym("omega")
    T = SX.sym("T")
    delta = SX.sym("delta")
    x = vertcat(xp, yp, yaw, vx, vy, omega, T, delta)

    # CasADi - input
    dT = SX.sym("dT")
    ddelta = SX.sym("ddelta")
    u = vertcat(dT, ddelta)

    # xdot
    xpdot = SX.sym("xpdot")
    ypdot = SX.sym("ypdot")
    yawdot = SX.sym("yawdot")
    vxdot = SX.sym("vxdot")
    vydot = SX.sym("vydot")
    omegadot = SX.sym("omegadot")
    xdot = vertcat(xpdot, ypdot, yawdot, vxdot, vydot,
                   omegadot, dT, ddelta)

    # algebraic variables
    z = vertcat([])

    # define params
    xd = SX.sym("xd")
    yd = SX.sym("yd")
    Q1 = SX.sym("Q1")
    Q2 = SX.sym("Q2")
    R1 = SX.sym("R1")
    R2 = SX.sym("R2")
    lr = SX.sym("lr")
    lf = SX.sym("lf")
    m = SX.sym("m")
    I = SX.sym("I")
    Df = SX.sym("Df")
    Cf = SX.sym("Cf")
    Bf = SX.sym("Bf")
    Dr = SX.sym("Dr")
    Cr = SX.sym("Cr")
    Br = SX.sym("Br")
    Cm1 = SX.sym("Cm1")
    Cm2 = SX.sym("Cm2")
    Cd = SX.sym("Cd")
    Croll = SX.sym("Croll")

    # parameters
    p = vertcat(xd, yd, Q1, Q2, R1,
                R2, lr, lf, m, I, Df, Cf, Bf, Dr, Cr, Br, Cm1, Cm2, Cd, Croll)


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
        ddelta
    )

    # cost
    eX = xp - xd
    eY = yp - yd

    c_eX = eX*eX*Q1
    c_eY = eY*eY*Q2
    c_dT = dT*dT*R1
    c_ddelta = ddelta*ddelta*R2

    model.cost_expr_ext_cost = c_eX + c_eY + c_dT+ c_ddelta

    # nonlinear track constraints
    # constraint.expr = vertcat(radius_sq) - not required at the moment

    params = types.SimpleNamespace()
    params.xd = xd
    params.yd = yd
    params.Q1 = Q1
    params.Q2 = Q2
    params.R1 = R1
    params.R2 = R2
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
