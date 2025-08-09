import numpy as np

def Rz(y):  return np.array([[np.cos(y),-np.sin(y),0],[np.sin(y),np.cos(y),0],[0,0,1]])
def Ry(p):  return np.array([[np.cos(p),0,np.sin(p)],[0,1,0],[-np.sin(p),0,np.cos(p)]])
def Rx(r):  return np.array([[1,0,0],[0,np.cos(r),-np.sin(r)],[0,np.sin(r),np.cos(r)]])

def leg_ik_with_body(roll, pitch, yaw,
                     foot_pos,        # (x,y,z) in BODY frame (see Case A); if in world, convert first
                     hip_offset,      # (x_i, y_i, z_i) in BODY frame
                     Lc, Lf, Lt,
                     flip_yaw=False):
    # 1) move foot target to hip origin
    xh, yh, zh = (np.array(foot_pos) - np.array(hip_offset))

    # 2) hip yaw
    th_c = np.arctan2(-yh, xh) if flip_yaw else np.arctan2(yh, xh)

    # 3) planar reduction
    r   = np.hypot(xh, yh) - Lc
    d   = np.hypot(r, zh)

    # 4) clamp for safety
    c_k = np.clip((Lf**2 + Lt**2 - d**2)/(2*Lf*Lt), -1.0, 1.0)
    c_a = np.clip((Lf**2 + d**2 - Lt**2)/(2*Lf*d),   -1.0, 1.0)

    beta   = np.arccos(c_k)
    alpha  = np.arccos(c_a)
    phi    = np.arctan2(zh, r)

    th_f = phi + alpha
    th_k = np.pi - beta
    return th_c, th_f, th_k

# If your input foot_pos is in WORLD, first do:
# pB = Rz(yaw)@Ry(pitch)@Rx(roll).T @ (pW - bodyOriginW)
# foot_pos = pB

