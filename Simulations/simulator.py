import mpmath as math
import numpy as np
import time
from vpython import *

ROD_SPACING = 0.085
ROD_RAD = 0.00635/2
ROD_LEN = 0.4
MIN_SPINE_LEN = 0.1

d = sin(np.pi/6) * ROD_SPACING / sin(np.pi/3)

def get_S():
    return (rod1.length + rod2.length + rod3.length)/3

def get_K():
    s1 = rod1.length
    s2 = rod2.length
    s3 = rod3.length

    return 2*sqrt(s1**2 + s2**2 + s3**2 - s1*s2 - s1*s3 - s2*s3)/d/(s1+s2+s3)

def get_P():
    s1 = rod1.length
    s2 = rod2.length
    s3 = rod3.length

    if s2 == s3:
        if s3+s2-2*s1 == 0:
            return None
        else:
            s2 = 0.999999*s2

    x = -sqrt(3)/3 * (s3+s2-2*s1) / (s2-s3)
    P = math.atan(1/x)

    if x < 0:
        P += math.pi
    if s2 < s3:
        P += math.pi

    return P

def get_ee_coords():
    S = get_S()
    K = get_K()
    P = get_P()

    x = 1/K * (1-cos(K*S))*cos(P)
    y = 1/K * (1-cos(K*S))*sin(P)
    z = sin(K*S)

    return (x,y,z)

def get_curve_pts(rod, P, K):
    s = rod.length

    R = 1/K

    pts = []

    for al0 in range(0, int(s*100)):
        al = al0/100
        angle = al/R

        x0 = al*sin(angle)
        y = al*cos(angle)

        x = x0*cos(-P)
        z = x0*sin(-P)

        pts.append(vec(x,y,z))

    return pts

rod1 = cylinder(
    pos=vec(-d,0,0),
    axis=vec(0,1,0),
    radius=ROD_RAD,
    length=ROD_LEN
)
curve1 = curve(radius=ROD_RAD, origin=vec(-d,0,0))

rod2 = cylinder(
    pos=vec(d*np.cos(np.pi/3),0,d*np.sin(np.pi/3)),
    axis=vec(0,1,0),
    radius=ROD_RAD,
    length=ROD_LEN
)
curve2 = curve(radius=ROD_RAD, origin=vec(d*np.cos(np.pi/3),0,d*np.sin(np.pi/3)))

rod3 = cylinder(
    pos=vec(d*np.cos(np.pi/3),0,-d*np.sin(np.pi/3)),
    axis=vec(0,1,0),
    radius=ROD_RAD,
    length=ROD_LEN
)
curve3 = curve(radius=ROD_RAD, origin=vec(d*np.cos(np.pi/3),0,-d*np.sin(np.pi/3)))

rods = [rod1, rod2, rod3]

selected_rod = rod1

def select_rod(x):
    global selected_rod
    x = int(x.text.split(" ")[1])
    selected_rod = rods[x-1]
    for i in range(3):
        if i == x-1:
            rods[i].color=color.cyan
        else:
            rods[i].color=color.white
    individual_length.value = selected_rod.length

radio1 = radio(bind=select_rod, checked=True, text="Rod 1", name="radios")
radio2 = radio(bind=select_rod, checked=False, text="Rod 2", name="radios")
radio3 = radio(bind=select_rod, checked=False, text="Rod 3", name="radios")

def change_length(x):
    global selected_rod
    selected_rod.length = x.value

individual_length = slider(min=MIN_SPINE_LEN, max=ROD_LEN, value=ROD_LEN, bind=change_length)

KILL = False
def kill(x):
    global KILL
    KILL = True

kill_btn = button(bind=kill, text="Stop")

while not KILL:
    P = get_P()
    K = get_K()
    if P is None or K == 0:
        rod1.opacity = 1
        rod2.opacity = 1
        rod3.opacity = 1
        curve1.clear()
        curve2.clear()
        curve3.clear()
    else:
        rod1.opacity = 0.1
        rod2.opacity = 0.1
        rod3.opacity = 0.1

        curve1.clear()
        curve1.append(get_curve_pts(rod1, P, K))
        curve2.clear()
        curve2.append(get_curve_pts(rod2, P, K))
        curve3.clear()
        curve3.append(get_curve_pts(rod3, P, K))

    time.sleep(0.01)