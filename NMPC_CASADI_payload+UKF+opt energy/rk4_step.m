function [x_next_val] = rk4_step(x_current_val, u_current_val, dt, ode_fcn, varargin)
    % Simple RK4 integrator for simulation
    % x_current_val: current state (numeric)
    % u_current_val: current input (numeric)
    % dt: time step
    % ode_fcn: function handle for continuous-time ODE f(x, u, p1, p2, ...)
    % varargin: additional parameters for ode_fcn (e.g., mass, gravity)

    k1 = ode_fcn(x_current_val, u_current_val, varargin{:});
    k2 = ode_fcn(x_current_val + dt/2 * k1, u_current_val, varargin{:});
    k3 = ode_fcn(x_current_val + dt/2 * k2, u_current_val, varargin{:});
    k4 = ode_fcn(x_current_val + dt * k3, u_current_val, varargin{:});
    
    x_next_val = x_current_val + dt/6 * (k1 + 2*k2 + 2*k3 + k4);
end