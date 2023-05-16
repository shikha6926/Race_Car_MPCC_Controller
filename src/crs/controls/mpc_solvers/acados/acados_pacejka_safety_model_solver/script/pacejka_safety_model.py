import math

from casadi import *
from typing import Dict


def safety_dynamic_model(constraints: Dict[str, float], track_constraints: Dict[str, float]):
    # define structs
    model = types.SimpleNamespace()
    constraint = types.SimpleNamespace()
    model_name = "safety_dynamic_model"
    

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

    # vehicle constant

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

    # Additional car parameters
    Car_width = SX.sym("Car_width") # Width of the car in m
    Car_overhang = SX.sym("Car_overhang") # Overhang of the car (distance from center of front wheel to boundary of car)

    # track width constraint
    xp_track = SX.sym("xp_track")
    yp_track = SX.sym("yp_track")
    yaw_track = SX.sym("yaw_track")

    # Final terminal constraint
    xp_e = SX.sym("xp_e")
    yp_e = SX.sym("yp_e")
    yaw_e = SX.sym("yaw_e")


    # parameters
    p = vertcat(lr, lf, m, I, Df, Cf, Bf, Dr, Cr, Br, Cm1, Cm2, Cd, Croll, Car_width, Car_overhang,
                xp_track, yp_track, yaw_track,
                xp_e, yp_e, yaw_e)
    # DYNAMICS


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
    )

    
    yaw_err_e = yaw - yaw_e
    y_err_e = -sin(yaw_e) * (xp - xp_e) + cos(yaw_e) * (yp - yp_e)


    y_err = -sin(yaw_track) * (xp - xp_track) + \
        cos(yaw_track) * (yp - yp_track)
    yaw_err = yaw - yaw_track
    
    lf_err = y_err + (lf + Car_overhang) * sin(yaw_err) + Car_width/2 * cos(yaw_err)
    rf_err = y_err + (lf + Car_overhang) * sin(yaw_err) - Car_width/2 * cos(yaw_err)
    
    constraint.expr = vertcat(lf_err, rf_err)
    constraint.expr_e = vertcat((xp - xp_e)**2 + (yp - yp_e)**2, vx)# + (yawp - yawp_e)**2

    # collect constants
    constants = types.SimpleNamespace()
    constants.track_width = track_constraints["track_width"]

    params = types.SimpleNamespace()
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

    # Car specificis for collision checks
    params.Car_width = Car_width
    params.Car_overhang = Car_overhang
   
    
    params.xp_track = xp_track
    params.yp_track = yp_track
    params.yaw_track = yaw_track
    params.xp_e = xp_e
    params.yp_e = yp_e
    params.yaw_e = yaw_e

    # Define rest of model
    model.x0 = np.zeros((8,))
    model.f_impl_expr = xdot - f_expl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    model.z = z
    model.p = p
    model.name = model_name
    model.params = params
    model.constants = constants
    
    # return model, constraint
    return model, constraint