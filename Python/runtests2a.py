import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import math

'''
*** BASIC HELPER FUNCTIONS ***
'''

def ECE569_NearZero(z):
    return abs(z) < 1e-6

def ECE569_Normalize(V):
    return V / np.linalg.norm(V)

'''
*** CHAPTER 3: RIGID-BODY MOTIONS ***
'''

def ECE569_RotInv(R):
    return np.array(R).T

def ECE569_VecToso3(omg):
    return np.array([[0,      -omg[2],  omg[1]],
                     [omg[2],       0, -omg[0]],
                     [-omg[1], omg[0],       0]])

def ECE569_so3ToVec(so3mat):
    return np.array([so3mat[2][1], so3mat[0][2], so3mat[1][0]])

def ECE569_AxisAng3(expc3):
    return (ECE569_Normalize(expc3), np.linalg.norm(expc3))

def ECE569_MatrixExp3(so3mat):
    omgtheta = ECE569_so3ToVec(so3mat)
    if ECE569_NearZero(np.linalg.norm(omgtheta)):
        return np.eye(3)
    else:
        theta = ECE569_AxisAng3(omgtheta)[1]
        omgmat = so3mat / theta
        return np.eye(3) + np.sin(theta) * omgmat \
               + (1 - np.cos(theta)) * np.dot(omgmat, omgmat)

def ECE569_MatrixLog3(R):
    acosinput = (np.trace(R) - 1) / 2.0
    if acosinput >= 1:
        return np.zeros((3, 3))
    elif acosinput <= -1:
        if not ECE569_NearZero(1 + R[2][2]):
            omg = (1.0 / np.sqrt(2 * (1 + R[2][2]))) \
                  * np.array([R[0][2], R[1][2], 1 + R[2][2]])
        elif not ECE569_NearZero(1 + R[1][1]):
            omg = (1.0 / np.sqrt(2 * (1 + R[1][1]))) \
                  * np.array([R[0][1], 1 + R[1][1], R[2][1]])
        else:
            omg = (1.0 / np.sqrt(2 * (1 + R[0][0]))) \
                  * np.array([1 + R[0][0], R[1][0], R[2][0]])
        return ECE569_VecToso3(np.pi * omg)
    else:
        theta = np.arccos(acosinput)
        return theta / 2.0 / np.sin(theta) * (R - np.array(R).T)

def ECE569_RpToTrans(R, p):
    return np.r_[np.c_[R, p], [[0, 0, 0, 1]]]

def ECE569_TransToRp(T):
    T = np.array(T)
    return T[0: 3, 0: 3], T[0: 3, 3]

def ECE569_TransInv(T):
    R, p = ECE569_TransToRp(T)
    Rt = np.array(R).T
    return np.r_[np.c_[Rt, -np.dot(Rt, p)], [[0, 0, 0, 1]]]

def ECE569_VecTose3(V):
    return np.array([[ 0,    -V[2],  V[1], V[3]],
                     [ V[2],  0,    -V[0], V[4]],
                     [-V[1],  V[0],  0,    V[5]],
                     [ 0,     0,     0,    0   ]])

def ECE569_se3ToVec(se3mat):
    return np.array([se3mat[2][1], se3mat[0][2], se3mat[1][0],
                     se3mat[0][3], se3mat[1][3], se3mat[2][3]])

def ECE569_Adjoint(T):
    R, p = ECE569_TransToRp(T)
    return np.r_[np.c_[R,                          np.zeros((3, 3))],
                 np.c_[np.dot(ECE569_VecToso3(p), R), R          ]]

def ECE569_MatrixExp6(se3mat):
    se3mat = np.array(se3mat)
    omgtheta = ECE569_so3ToVec(se3mat[0: 3, 0: 3])
    if ECE569_NearZero(np.linalg.norm(omgtheta)):
        return np.r_[np.c_[np.eye(3), se3mat[0: 3, 3]], [[0, 0, 0, 1]]]
    else:
        theta = ECE569_AxisAng3(omgtheta)[1]
        omgmat = se3mat[0: 3, 0: 3] / theta
        return np.r_[np.c_[
            ECE569_MatrixExp3(se3mat[0: 3, 0: 3]),
            np.dot(np.eye(3) * theta
                   + (1 - np.cos(theta)) * omgmat
                   + (theta - np.sin(theta)) * np.dot(omgmat, omgmat),
                   se3mat[0: 3, 3]) / theta],
            [[0, 0, 0, 1]]]

def ECE569_MatrixLog6(T):
    R, p = ECE569_TransToRp(T)
    omgmat = ECE569_MatrixLog3(R)
    if np.array_equal(omgmat, np.zeros((3, 3))):
        return np.r_[np.c_[np.zeros((3, 3)), p], [[0, 0, 0, 0]]]
    else:
        theta = np.arccos(np.clip((np.trace(R) - 1) / 2.0, -1, 1))
        if ECE569_NearZero(theta):
            return np.r_[np.c_[np.zeros((3, 3)), p], [[0, 0, 0, 0]]]
        omgmat_n = omgmat / theta   # normalized [omghat]
        # G_inv per MR eq 3.92, multiplied by theta as noted in lab spec
        G_inv = ((1.0 / theta) * np.eye(3)
                 - 0.5 * omgmat_n
                 + (1.0 / theta - 0.5 / np.tan(theta / 2.0))
                 * np.dot(omgmat_n, omgmat_n))
        v = theta * np.dot(G_inv, p)
        return np.r_[np.c_[omgmat, v], [[0, 0, 0, 0]]]


'''
*** CHAPTER 4: FORWARD KINEMATICS ***
'''

def ECE569_FKinBody(M, Blist, thetalist):
    T = np.array(M, dtype=float)
    for i in range(len(thetalist)):
        T = np.dot(T, ECE569_MatrixExp6(ECE569_VecTose3(
            np.array(Blist)[:, i] * thetalist[i])))
    return T

def ECE569_FKinSpace(M, Slist, thetalist):
    T = np.array(M, dtype=float)
    for i in range(len(thetalist) - 1, -1, -1):
        T = np.dot(ECE569_MatrixExp6(ECE569_VecTose3(
            np.array(Slist)[:, i] * thetalist[i])), T)
    return T

'''
*** CHAPTER 5: VELOCITY KINEMATICS AND STATICS***
'''

def ECE569_JacobianBody(Blist, thetalist):
    Jb = np.array(Blist).copy().astype(float)
    T = np.eye(4)
    for i in range(len(thetalist) - 2, -1, -1):
        T = np.dot(T, ECE569_MatrixExp6(ECE569_VecTose3(
            np.array(Blist)[:, i + 1] * -thetalist[i + 1])))
        Jb[:, i] = np.dot(ECE569_Adjoint(T), Blist[:, i])
    return Jb

'''
*** CHAPTER 6: INVERSE KINEMATICS ***
'''

def ECE569_IKinBody(Blist, M, T, thetalist0, eomg, ev):
    thetalist = np.array(thetalist0).copy()
    i = 0
    maxiterations = 20
    Vb = ECE569_se3ToVec(
            ECE569_MatrixLog6(
                np.dot(ECE569_TransInv(ECE569_FKinBody(M, Blist, thetalist)), T)))
    err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg \
          or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev
    while err and i < maxiterations:
        thetalist = thetalist + np.dot(np.linalg.pinv(
            ECE569_JacobianBody(Blist, thetalist)), Vb)
        i += 1
        Vb = ECE569_se3ToVec(
                ECE569_MatrixLog6(
                    np.dot(ECE569_TransInv(ECE569_FKinBody(M, Blist, thetalist)), T)))
        err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg \
              or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev
    return (thetalist, not err)

# the ECE569_normalized trapezoid function
def g(t, T, ta):
    if t < 0 or t > T:
        return 0
    if t < ta:
        return (T/(T-ta))* t/ta
    elif t > T - ta:
        return (T/(T-ta))*(T - t)/ta
    else:
        return (T/(T-ta))

def trapezoid(t, T, ta):
    return g(t, T, ta)

def main():

    ### Step 1: Trajectory Generation

    # Lissajous curve: x=A*sin(a*t), y=B*sin(b*t)
    # NOT a figure-8: using a=1, b=2 gives a figure-8, so we use a=3, b=2
    # which gives a closed curve with 3 x-lobes and 2 y-lobes
    # Period = 2*pi (lcm period for a=3,b=2 with gcd=1)
    A      = 0.08         # x-amplitude (m) — within ±0.16 m limit
    B      = 0.08         # y-amplitude (m) — within ±0.16 m limit
    a_coef = 3            # x-frequency  (NOT a figure-8: a=3,b=2)
    b_coef = 2            # y-frequency
    T      = 2 * np.pi    # period (both sin(3t)=0 and sin(2t)=0 at t=2pi)
    tfinal = 5.0          # trajectory duration (s)
    ta     = 0.5          # trapezoid ramp time (s)

    # Coarse grid for arc-length calculation
    N_coarse = 10000
    t_coarse = np.linspace(0, T, N_coarse)
    xd = A * np.sin(a_coef * t_coarse)
    yd = B * np.sin(b_coef * t_coarse)

    # calculate the arc length
    d = 0
    for i in range(1, len(t_coarse)):
        d += np.sqrt((xd[i] - xd[i-1])**2 + (yd[i] - yd[i-1])**2)

    # calculate average velocity
    c = d / tfinal
    print(f'Arc length = {d:.4f} m,  avg velocity c = {c:.4f} m/s  (limit 0.25 m/s)')

    # forward euler to calculate alpha
    dt = 0.002
    t = np.arange(0, tfinal, dt)
    alpha = np.zeros(len(t))
    for i in range(1, len(t)):
        xdot = A * a_coef * np.cos(a_coef * alpha[i-1])
        ydot = B * b_coef * np.cos(b_coef * alpha[i-1])
        speed = np.sqrt(xdot**2 + ydot**2)
        if speed < 1e-9:
            speed = 1e-9
        alpha[i] = alpha[i-1] + (c * g(t[i-1], tfinal, ta) / speed) * dt

    print(f'alpha(tf) = {alpha[-1]:.4f}  (should be ~T = {T:.4f})')

    # plot alpha vs t  (Figure 1.2)
    plt.figure()
    plt.plot(t, alpha, 'b-', label='alpha')
    plt.plot(t, np.ones(len(t)) * T, 'k--', label='T (period)')
    plt.xlabel('time (s)')
    plt.ylabel('alpha(t)')
    plt.title('Plot of alpha(t)')
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.savefig('/mnt/user-data/outputs/fig1b_alpha.png', dpi=150)
    plt.close()

    # rescale trajectory with alpha
    x = A * np.sin(a_coef * alpha)
    y = B * np.sin(b_coef * alpha)

    # calculate velocity
    xdot = np.diff(x) / dt
    ydot = np.diff(y) / dt
    v = np.sqrt(xdot**2 + ydot**2)

    # plot velocity vs t  (Figure 1.3)
    plt.figure()
    plt.plot(t[1:], v, 'b-', label='velocity')
    plt.plot(t[1:], np.ones(len(t[1:])) * c,    'k--', label='average velocity')
    plt.plot(t[1:], np.ones(len(t[1:])) * 0.25, 'r--', label='velocity limit')
    plt.xlabel('time (s)')
    plt.ylabel('velocity (m/s)')
    plt.title('Trajectory Velocity')
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.savefig('/mnt/user-data/outputs/fig1c_velocity.png', dpi=150)
    plt.close()

    # plot desired trajectory  (Figure 1.1)
    plt.figure()
    plt.plot(xd, yd, 'b-', linewidth=1.5)
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.title('Desired Lissajous Trajectory')
    plt.xlim(-0.18, 0.18); plt.ylim(-0.18, 0.18)
    plt.axhline( 0.16, color='r', linestyle='--', alpha=0.5, label='±0.16 m limit')
    plt.axhline(-0.16, color='r', linestyle='--', alpha=0.5)
    plt.axvline( 0.16, color='r', linestyle='--', alpha=0.5)
    plt.axvline(-0.16, color='r', linestyle='--', alpha=0.5)
    plt.gca().set_aspect('equal'); plt.grid(True); plt.legend()
    plt.tight_layout()
    plt.savefig('/mnt/user-data/outputs/fig1a_trajectory.png', dpi=150)
    plt.close()

    ### Step 2: Forward Kinematics
    L1 = 0.2435
    L2 = 0.2132
    W1 = 0.1311
    W2 = 0.0921
    H1 = 0.1519
    H2 = 0.0854

    M = np.array([[1, 0, 0, L1 + L2],
                  [0, 0, -1, -W1 - W2],
                  [0, 1, 0, H1 - H2],
                  [0, 0, 0, 1]])

    S1 = np.array([0, 0, 1, 0, 0, 0])
    S2 = np.array([0, -1, 0, H1, 0, 0])
    S3 = np.array([0, -1, 0, H1, 0, L1])
    S4 = np.array([0, -1, 0, H1, 0, L1 + L2])
    S5 = np.array([0, 0, -1, W1, L1+L2, 0])
    S6 = np.array([0, -1, 0, H1-H2, 0, L1+L2])
    S = np.array([S1, S2, S3, S4, S5, S6]).T

    B1 = np.linalg.inv(ECE569_Adjoint(M))@S1
    B2 = np.linalg.inv(ECE569_Adjoint(M))@S2
    B3 = np.linalg.inv(ECE569_Adjoint(M))@S3
    B4 = np.linalg.inv(ECE569_Adjoint(M))@S4
    B5 = np.linalg.inv(ECE569_Adjoint(M))@S5
    B6 = np.linalg.inv(ECE569_Adjoint(M))@S6
    B = np.array([B1, B2, B3, B4, B5, B6]).T

    theta0 = np.deg2rad(np.array([-51.0, -85.09, -125.84, -149.22, -51.0, 0.0]))

    T0_space = ECE569_FKinSpace(M, S, theta0)
    print(f'T0_space:\n{T0_space}')
    T0_body = ECE569_FKinBody(M, B, theta0)
    print(f'T0_body:\n{T0_body}')
    T0_diff = T0_space - T0_body
    print(f'T0_diff (should be ~0):\n{T0_diff}')
    T0 = T0_body

    # calculate Tsd for each time step:  Tsd(t) = T0 @ Td(t)
    Tsd = np.zeros((4, 4, len(t)))
    for i in range(len(t)):
        Td = np.array([[1, 0, 0, x[i]],
                       [0, 1, 0, y[i]],
                       [0, 0, 1, 0   ],
                       [0, 0, 0, 1   ]])
        Tsd[:, :, i] = np.dot(T0, Td)

    # plot p(t) in {s} frame  (Figure 2.4)
    xs = Tsd[0, 3, :]
    ys = Tsd[1, 3, :]
    zs = Tsd[2, 3, :]
    ax = plt.figure().add_subplot(projection='3d')
    ax.plot(xs, ys, zs, 'b-', label='p(t)')
    ax.plot([xs[0]], [ys[0]], [zs[0]], 'go', markersize=8, label='start')
    ax.plot([xs[-1]], [ys[-1]], [zs[-1]], 'rx', markersize=8,
            markeredgewidth=2, label='end')
    ax.set_aspect('equal')
    ax.set_title('Trajectory in {s} frame')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    ax.legend()
    plt.tight_layout()
    plt.savefig('/mnt/user-data/outputs/fig2d_trajectory_s.png', dpi=150)
    plt.close()

    ### Step 3: Inverse Kinematics

    thetaAll = np.zeros((6, len(t)))
    initialguess = theta0
    eomg = 1e-6
    ev   = 1e-6

    thetaSol, success = ECE569_IKinBody(B, M, Tsd[:,:,0], initialguess, eomg, ev)
    if not success:
        raise Exception(f'Failed to find a solution at index 0')
    thetaAll[:, 0] = thetaSol

    for i in range(1, len(t)):
        initialguess = thetaAll[:, i-1]          # warm-start
        thetaSol, success = ECE569_IKinBody(B, M, Tsd[:,:,i], initialguess, eomg, ev)
        if not success:
            print(f'Warning: IK did not converge at index {i}')
        thetaAll[:, i] = thetaSol
        if i % 500 == 0:
            print(f'  IK progress: {i}/{len(t)}')

    print('IK complete')

    # verify joint angles don't change much  (Figure 3.3)
    dj = np.diff(thetaAll, axis=1)
    plt.figure()
    plt.plot(t[1:], dj[0], 'b-', label='joint 1')
    plt.plot(t[1:], dj[1], 'g-', label='joint 2')
    plt.plot(t[1:], dj[2], 'r-', label='joint 3')
    plt.plot(t[1:], dj[3], 'c-', label='joint 4')
    plt.plot(t[1:], dj[4], 'm-', label='joint 5')
    plt.plot(t[1:], dj[5], 'y-', label='joint 6')
    plt.xlabel('t (seconds)')
    plt.ylabel('first order difference')
    plt.title('Joint angles first order difference')
    plt.legend(bbox_to_anchor=(1.05, 1.0), loc='upper left')
    plt.grid()
    plt.tight_layout()
    plt.savefig('/mnt/user-data/outputs/fig3c_dtheta.png', dpi=150)
    plt.close()
    print(f'Max |Δθ| = {np.abs(dj).max():.2e}')

    # verify trajectory via FK  (Figure 3.4)
    actual_Tsd = np.zeros((4, 4, len(t)))
    for i in range(len(t)):
        actual_Tsd[:,:,i] = ECE569_FKinBody(M, B, thetaAll[:,i])

    xs = actual_Tsd[0, 3, :]
    ys = actual_Tsd[1, 3, :]
    zs = actual_Tsd[2, 3, :]
    ax = plt.figure().add_subplot(projection='3d')
    ax.plot(xs, ys, zs, 'b-', label='p(t)')
    ax.plot([xs[0]], [ys[0]], [zs[0]], 'go', markersize=8, label='start')
    ax.plot([xs[-1]], [ys[-1]], [zs[-1]], 'rx', markersize=8,
            markeredgewidth=2, label='end')
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    ax.set_title('Verified Trajectory in {s} frame')
    ax.legend()
    plt.tight_layout()
    plt.savefig('/mnt/user-data/outputs/fig3d_verified.png', dpi=150)
    plt.close()

    # manipulability mu3  (Figure 3.5)
    mu3s = np.zeros(len(t))
    for i in range(len(t)):
        Jb = ECE569_JacobianBody(B, thetaAll[:, i])
        Jv = Jb[3:, :]                                     # last 3 rows
        mu3s[i] = np.sqrt(max(np.linalg.det(np.dot(Jv, Jv.T)), 0))

    plt.figure()
    plt.plot(t, mu3s, 'b-')
    plt.xlabel('t (seconds)')
    plt.ylabel(r'$\mu_3 = \sqrt{det(J_v J_v^\top)}$')
    plt.title('Manipulability')
    plt.grid()
    plt.tight_layout()
    plt.savefig('/mnt/user-data/outputs/fig3e_manipulability.png', dpi=150)
    plt.close()
    print(f'Min mu3 = {mu3s.min():.4f}  (must be > 0)')

    # save to csv — replace 'student' with your Purdue username
    led = np.ones_like(t)
    data = np.column_stack((t, thetaAll.T, led))
    np.savetxt('/mnt/user-data/outputs/student.csv', data, delimiter=',')
    print(f'CSV saved: {data.shape[0]} rows x {data.shape[1]} cols')
    print('Column order: time, j1, j2, j3, j4, j5, j6, led')


if __name__ == "__main__":
    main()
