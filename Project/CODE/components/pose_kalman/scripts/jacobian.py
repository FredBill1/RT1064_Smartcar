import sympy
from sympy import sin, cos, diff

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
res = sympy.simplify(res)
for v in res:
    print(v, end=", ")
