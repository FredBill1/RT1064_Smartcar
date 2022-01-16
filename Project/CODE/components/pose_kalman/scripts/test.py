import sympy
from sympy import sin, cos, diff
from sympy import *

x, y, w, dx, dy, dw = sympy.symbols("x y w dx dy dw")
x_, y_, w_, dx_, dy_, dw_ = sympy.symbols("x_ y_ w_ dx_ dy_ dw_")
dt = sympy.symbols("dt")

ux, uy, uw = sympy.symbols("ux uy uw")

dw_ = uw
w_ = w + (dw + uw) / 2 * dt

dx_ = ux * cos(w_) - uy * sin(w_)
dy_ = uy * cos(w_) + ux * sin(w_)

x_ = x + (dx + dx_) / 2 * dt
y_ = y + (dy + dy_) / 2 * dt

# x_ = x + dx_ * dt
# y_ = y + dx_ * dt


funcs = sympy.Matrix([x_, y_, w_, dx_, dy_, dw_])
args = sympy.Matrix([x, y, w, dx, dy, dw])
res: sympy.Matrix = funcs.jacobian(args)


Sin = sympy.symbols("Sin")
Cos = sympy.symbols("Cos")

# THETA = dt * (dw + uw) / 2 + w
Sin = sin(w_)
Cos = cos(w_)
# -dy_ = -(ux * Sin + uy * Cos)
# dx_ = ux * Cos - uy * Sin
dx_ = dx_
DT_2 = dt / 2
DT_22 = DT_2 * DT_2
DTY = DT_2 * -dy_
DTX = DT_2 * dx_
DTTY = DTY * DT_2
DTTX = DTX * DT_2
ddd = [
    1,
    0,
    DTY,
    DT_2,
    0,
    DTTY,
    0,
    1,
    DTX,
    0,
    DT_2,
    DTTX,
    0,
    0,
    1,
    0,
    0,
    DT_2,
    0,
    0,
    -dy_,
    0,
    0,
    DTY,
    0,
    0,
    dx_,
    0,
    0,
    DTX,
    0,
    0,
    0,
    0,
    0,
    0,
]
for a, b in zip(res, ddd):
    if a != b:
        print(sympy.simplify(a - b))
