% DIAGNOSTIC CODE - Jalankan ini untuk debug masalah

%% 1. Test Trajectory Function
fprintf('=== TESTING TRAJECTORY FUNCTION ===\n');
for t_test = [0, 1, 2, 3, 4, 5]
    ref = QuadrotorReferenceTrajectory4(t_test);
    fprintf('t=%.1f: Pos=[%.3f, %.3f, %.3f], Vel=[%.3f, %.3f, %.3f]\n', ...
            t_test, ref(1), ref(2), ref(3), ref(7), ref(8), ref(9));
end

%% 2. Test Model Dynamics
fprintf('\n=== TESTING MODEL DYNAMICS ===\n');
% Setup basic parameters (copy from main code)
m = 0.5; g = 9.81; l = 0.25;
Ixx = 4.85e-3; Iyy = 4.85e-3; Izz = 8.81e-3;
thrust_hover = m * g / 4;

% Test hover condition
test_state = [0; 0; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0]; % Hover at 1m
test_input = thrust_hover * ones(4, 1); % Equal thrust
fprintf('Hover test - Input: [%.3f, %.3f, %.3f, %.3f] N\n', test_input');

% Simplified dynamics test (without CasADi for quick check)
F_total = sum(test_input);
fprintf('Total thrust: %.3f N (should be ~%.3f N for hover)\n', F_total, m*g);

% Check if CasADi dynamics work
if exist('F_discrete', 'var')
    try
        next_state = full(F_discrete(test_state, test_input));
        state_change = next_state - test_state;
        fprintf('State change after 1 time step:\n');
        fprintf('  Position: [%.6f, %.6f, %.6f]\n', state_change(1:3)');
        fprintf('  Velocity: [%.6f, %.6f, %.6f]\n', state_change(7:9)');
        
        % Should be very small for hover condition
        if norm(state_change(1:3)) > 0.001
            fprintf('WARNING: Large position change during hover!\n');
        end
    catch ME
        fprintf('ERROR in dynamics function: %s\n', ME.message);
    end
else
    fprintf('F_discrete function not found - run main NMPC code first\n');
end

%% 3. Check Parameter Scaling
fprintf('\n=== CHECKING PARAMETER SCALING ===\n');
nx = 12; nu = 4;
Q_test = diag([500 500 300 50 50 50 100 100 100 10 10 10]);
R_test = diag([0.01 0.01 0.01 0.01]);

fprintf('Q matrix diagonal (position weights): %g, %g, %g\n', Q_test(1,1), Q_test(2,2), Q_test(3,3));
fprintf('R matrix diagonal (control weights): %g\n', R_test(1,1));

% Check if weights are balanced
pos_weight = mean([Q_test(1,1), Q_test(2,2), Q_test(3,3)]);
control_weight = R_test(1,1);
weight_ratio = pos_weight / control_weight;
fprintf('Position/Control weight ratio: %g\n', weight_ratio);

if weight_ratio > 10000
    fprintf('WARNING: Position weights might be too high compared to control weights!\n');
end

%% 4. Check Bounds
fprintf('\n=== CHECKING BOUNDS ===\n');
lb_thrust = 0.1 * thrust_hover;
ub_thrust = 3 * thrust_hover;
fprintf('Thrust bounds: [%.3f, %.3f] N per motor\n', lb_thrust, ub_thrust);
fprintf('Hover thrust: %.3f N per motor\n', thrust_hover);

if ub_thrust < 2 * thrust_hover
    fprintf('WARNING: Upper bound might be too low for maneuvering!\n');
end

%% 5. Simplified Trajectory Test
fprintf('\n=== TESTING SIMPLE TRAJECTORY ===\n');
% Test very simple trajectory manually
simple_traj = @(t) [0; 0; min(2, 0.5*t); 0; 0; 0; 0; 0; 0.5*(t<4); 0; 0; 0];

for t_test = [0, 1, 2, 3, 4, 5]
    traj = simple_traj(t_test);
    fprintf('t=%.1f: Simple traj = [%.3f, %.3f, %.3f]\n', t_test, traj(1), traj(2), traj(3));
end