function [ xdesired ] = QuadrotorReferenceTrajectory( t )
    % This function generates reference signal for nonlinear MPC controller
    % used in the quadrotor path following example.

    % Copyright 2019 The MathWorks, Inc.

    %#codegen
%     x = (6*sin(t/3) + 6) * (5/6); % Normalisasi dan skalasi
%     y = (-6*sin(t/3).*cos(t/3) + 6) * (5/6);
%     z = (6*cos(t/3) + 6) * (5/6);

    scaling_factor = (t); % Faktor skala bertahap dari 0 hingga 1

    x = (scaling_factor .* 6 .* sin(t/3) + 6) * (5/6); 
    y = (scaling_factor .* -6 .* sin(t/3).*cos(t/3) + 6) * (5/6);
    z = (scaling_factor .* 6 .* cos(t/3) + 6) * (5/6);
    phi = zeros(1,length(t)); % pitch
    theta = zeros(1,length(t)); % roll
    psi = zeros(1,length(t)); % yaw
    xdot = zeros(1,length(t)); % vx
    ydot = zeros(1,length(t)); % vy
    zdot = zeros(1,length(t)); % vz
    phidot = zeros(1,length(t)); %
    thetadot = zeros(1,length(t)); %
    psidot = zeros(1,length(t)); %

    xdesired = [x;y;z;phi;theta;psi;xdot;ydot;zdot;phidot;thetadot;psidot];
end


% function [xdesired] = QuadrotorReferenceTrajectory(t)
%     % Generates a reference signal for nonlinear MPC controller.
%     % Modified to create a landing trajectory like a SpaceX rocket.
%     
%     %#codegen
% 
%     % Trajectory equations
%     x = 6 * exp(-0.2 * t);     % Dekat horizontal
%     y = zeros(size(t));        % Tetap nol, drone tidak bergerak pada sumbu Y
%     z = max(6 - t, 0);         % Turun secara linear menuju Z=0
% 
%     % Angular and velocity components (set to zero for simplicity)
%     phi = zeros(size(t));
%     theta = zeros(size(t));
%     psi = zeros(size(t));
%     xdot = gradient(x, t);     % Kecepatan X
%     ydot = zeros(size(t));     % Kecepatan Y tetap nol
%     zdot = gradient(z, t);     % Kecepatan Z
% 
%     % Angular velocity components
%     phidot = zeros(size(t));
%     thetadot = zeros(size(t));
%     psidot = zeros(size(t));
% 
%     % Combine into the output vector
%     xdesired = [x; y; z; phi; theta; psi; xdot; ydot; zdot; phidot; thetadot; psidot];
% end

% 
% function [xdesired] = QuadrotorReferenceTrajectory(t)
%     %#codegen
% 
%     % Environment and settings for RRT*
%     start = [40 180 25 0.7 0.2 0 0.1];
%     goal = [150 33 35 0.3 0 0.1 0.6];
%     maxIterations = 2000;
%     stepSize = 2.0;
% 
%     % Generate path using RRT*
%     [xPath, yPath, zPath] = RRTStar3D(start, goal, maxIterations, stepSize);
% 
%     % Handle cases where path is empty or insufficient for interpolation
%     if isempty(xPath) || length(xPath) < 2
%         warning('RRT* failed or insufficient points. Using default path.');
%         xPath = linspace(start(1), goal(1), 10); % Default linear path
%         yPath = linspace(start(2), goal(2), 10);
%         zPath = linspace(start(3), goal(3), 10);
%     end
% 
%     % Generate time for the path
%     pathTime = linspace(0, max(t), length(xPath));
% 
%     % Interpolate x, y, z to match the size of t
%     x = interp1(pathTime, xPath, t, 'linear', 'extrap');
%     y = interp1(pathTime, yPath, t, 'linear', 'extrap');
%     z = interp1(pathTime, zPath, t, 'linear', 'extrap');
% 
%     % Set remaining variables (angles, velocities) to zero
%     phi = zeros(1, length(t));
%     theta = zeros(1, length(t));
%     psi = zeros(1, length(t));
%     xdot = zeros(1, length(t));
%     ydot = zeros(1, length(t));
%     zdot = zeros(1, length(t));
%     phidot = zeros(1, length(t));
%     thetadot = zeros(1, length(t));
%     psidot = zeros(1, length(t));
% 
%     % Combine into xdesired
%     xdesired = [x; y; z; phi; theta; psi; xdot; ydot; zdot; phidot; thetadot; psidot];
% end

