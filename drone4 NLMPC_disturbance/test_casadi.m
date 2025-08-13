import casadi.*

x = SX.sym('x');
y = SX.sym('y');
f = x^2 + y^2;
g = Function('g', {x, y}, {f});
disp(g(3, 4))
