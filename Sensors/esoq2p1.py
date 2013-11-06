import numpy as np
# Taken from https://raw.github.com/muzhig/ESOQ2/master/esoq2p1.py
# ESOQ2.1 - Attitude EStimate OPtimization (F. L. Markley, 7/15/99).
# Variation of Daniele Mortari's ESOQ2 (Paper AAS 97-167, AAS/AIAA
# Space Flight Mechanics Meeting, Huntsville, AL, February 10-12, 1997),
# with new singularity-avoidance and lambda_max computation logic.
#
# input: obs(3,n) - array of n observation unit vectors
# ref(3,n) - array of n reference unit vectors
# wt(n) - row array of n measurement weights
#
# The columns of obs and ref are assumed to be normalized.
# No assumption is made about the normalization of the weights.
#
# output: q(4) - optimal quaternion
#  loss - optimized value of Wahba's loss function

def esoq2p1(obs, ref, wt):
    lam = sum(wt)  # zeroth order approximation to lambda_max
    B = np.array([obs[0, :]*wt, obs[1, :]*wt, obs[2, :]*wt])
    B = B.dot(ref.T)
    trB = np.trace(B)
    diag = [B[0, 0], B[1, 1], B[2, 2], trB]

    # Optimal 180 deg rotation to avoid zero rotation angle singularity

    Bmin = min(diag)
    irot = diag.index(Bmin)

    if irot == 0:
        B[:, 1:3] *= -1
        trB = 2 * Bmin - trB
    elif irot == 1:
        B[:, 0] *= -1
        B[:, 2] *= -1
        trB = 2 * Bmin - trB
    elif irot == 2:
        B[:, 0:2] *= -1
        trB = 2 * Bmin - trB

    # Compute needed matrices and vectors
    S11 = 2 * B[0, 0]
    S23 = B[1, 2] + B[2, 1]
    S22 = 2 * B[1, 1]
    S31 = B[2, 0] + B[0, 2]
    S33 = 2 * B[2, 2]
    S12 = B[0, 1] + B[1, 0]
    z = np.array([B[1, 2] - B[2, 1], B[2, 0] - B[0, 2], B[0, 1] - B[1, 0]])
    z12 = z[0] * z[0]
    z22 = z[1] * z[1]
    z32 = z[2] * z[2]

    wt_len_eq_2 = max(wt.shape) == 2
    # max eigenvalue computation for two observation case
    if wt_len_eq_2:
        lam0 = lam
        trB2 = trB * trB
        Sz = np.array([[S11, S12, S31], [S12, S22, S23], [S31, S23, S33]]).dot(z)
        aa = trB2 - S22 * S33 + S23 * S23 - S11 * S33 + S31 * S31 - S22 * S11 + S12 * S12
        bb = trB2 + z12 + z22 + z32
        c2 = - aa - bb
        u = 2 * np.sqrt(aa * bb - Sz.T.dot(Sz))

        lam = (np.sqrt(u - c2) + np.sqrt(- u - c2)) / 2
        loss = lam0 - lam
    tml = trB - lam
    tpl = trB + lam


    M11 = tml * (S11 - tpl) - z12
    M23 = tml * S23 - z[1] * z[2]
    M22 = tml * (S22 - tpl) - z22
    M31 = tml * S31 - z[2] * z[0]
    M33 = tml * (S33 - tpl) - z32
    M12 = tml * S12 - z[0] * z[1]

    # Compute loss function and rotation axis
    e = np.array([M22 * M33 - M23 * M23, M11 * M33 - M31 * M31, M11 * M22 - M12 * M12])

    dummy = np.max(np.abs(e))

    if e[0] == dummy:
        e = np.array([e[0], M31 * M23 - M12 * M33, M12 * M23 - M31 * M22])
        imax = 0
    elif e[1] == dummy:
        e = np.array([M31 * M23 - M12 * M33, e[1], M12 * M31 - M11 * M23])
        imax = 1
    else:
        e = np.array([M12 * M23 - M31 * M22, M12 * M31 - M11 * M23, e[2]])
        imax = 2

    if not wt_len_eq_2:
        m1 = np.array([M11, M12, M31])
        m2 = np.array([M12, M22, M23])
        m3 = np.array([M31, M23, M33])
        n1 = np.array([(S11 - 2 * lam), S12, S31])
        n2 = np.array([S12, (S22 - 2 * lam), S23])
        n3 = np.array([S31, S23, (S33 - 2 * lam)])


        a = [m2, m3, m1][imax]
        b = [n3, n1, n2][imax]
        c = [m3, m1, m2][imax]
        d = [n2, n3, n1][imax]
        m = [m1, m2, m3][imax]
        n = [n1, n2, n3][imax]

        v = np.cross(a,b).T - np.cross(c, d).T

        loss = - (m.dot(e)) / (n.dot(e) + m.dot(v))
        tml = tml + loss
        e = e + loss * v
    # Quaternion computation in rotated frame
    q = np.hstack((tml * e, -z.T.dot(e)))

    q = q / np.linalg.norm(q)
    # Undo rotation to get quaternion in input frame
    if irot == 0:
        q = np.array([-q[0], q[3], -q[2], q[1]])
    elif irot == 1:
        q = np.array([-q[1], q[2], q[3], -q[0]])
    elif irot == 2:
        q = np.array([-q[2], -q[1], q[0], q[3]])

    return q, loss
