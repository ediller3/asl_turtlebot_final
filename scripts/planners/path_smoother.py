import numpy as np
import scipy.interpolate

def compute_smoothed_traj(path, V_des, k, alpha, dt):
    """
    Fit cubic spline to a path and generate a resulting trajectory for our
    wheeled robot.

    Inputs:
        path (np.array [N,2]): Initial path
        V_des (float): Desired nominal velocity, used as a heuristic to assign nominal
            times to points in the initial path
        k (int): The degree of the spline fit.
            For this assignment, k should equal 3 (see documentation for
            scipy.interpolate.splrep)
        alpha (float): Smoothing parameter (see documentation for
            scipy.interpolate.splrep)
        dt (float): Timestep used in final smooth trajectory
    Outputs:
        t_smoothed (np.array [N]): Associated trajectory times
        traj_smoothed (np.array [N,7]): Smoothed trajectory
    Hint: Use splrep and splev from scipy.interpolate
    """
    assert(path and k > 2 and k < len(path))
    ########## Code starts here ##########
    time_nom = np.zeros(len(path))
    for i in range(1, len(path)):
        time_nom[i] =  time_nom[i - 1] + np.linalg.norm(np.array(path[i]) - np.array(path[i-1]))/V_des

    t_smoothed = np.arange(0, time_nom[-1], dt)

    x, y = list(map(list, zip(*path)))

    tck_x = scipy.interpolate.splrep(time_nom, x, s=alpha)
    tck_y = scipy.interpolate.splrep(time_nom, y, s=alpha)

    x_d = scipy.interpolate.splev(t_smoothed, tck_x) 
    y_d = scipy.interpolate.splev(t_smoothed, tck_y) 
    xd_d = scipy.interpolate.splev(t_smoothed, tck_x, der=1) 
    yd_d = scipy.interpolate.splev(t_smoothed, tck_y, der=1) 
    xdd_d = scipy.interpolate.splev(t_smoothed, tck_x, der=2) 
    ydd_d = scipy.interpolate.splev(t_smoothed, tck_y, der=2) 
    theta_d = np.arctan2(yd_d, xd_d)
    ########## Code ends here ##########
    traj_smoothed = np.stack([x_d, y_d, theta_d, xd_d, yd_d, xdd_d, ydd_d]).transpose()

    return t_smoothed, traj_smoothed
