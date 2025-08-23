% This script plots the closed-loop responses of the nonlinear MPC
% controller used in the quadrotor path following example.

% Copyright 2019 The MathWorks, Inc.

% Plot the closed-loop response.
time = 0:Ts:Duration;
yreftot = QuadrotorReferenceTrajectory(time)';

% Plot the states.
fig = figure(1);      % Fokus pada figure 1
set(fig, 'Name', 'Drone Position Plot', 'NumberTitle', 'off'); % Beri nama pada window

subplot(3,4,1)
hold on
plot(time,xHistory(:,1))
plot(time,yreftot(:,1))
grid on
xlabel('time')
ylabel('x')
legend('actual','reference','Location','southeast')
title('Qruadrotor x position')

subplot(3,4,2)
hold on
plot(time,xHistory(:,2))
plot(time,yreftot(:,2))
grid on
xlabel('time')
ylabel('y')
legend('actual','reference','Location','southeast')
title('Qruadrotor y position')

subplot(3,4,3)
hold on
plot(time,xHistory(:,3))
plot(time,yreftot(:,3))
grid on
xlabel('time')
ylabel('z')
legend('actual','reference','Location','southeast')
title('Qruadrotor z position')

subplot(3,4,4)
hold on
plot(time,xHistory(:,4))
plot(time,yreftot(:,4))
grid on
xlabel('time')
ylabel('phi')
legend('actual','reference','Location','southeast')
title('Qruadrotor phi angle')

subplot(3,4,5)
hold on
plot(time,xHistory(:,5))
plot(time,yreftot(:,5))
grid on
xlabel('time')
ylabel('theta')
legend('actual','reference','Location','southeast')
title('Qruadrotor theta angle')

subplot(3,4,6)
hold on
plot(time,xHistory(:,6))
plot(time,yreftot(:,6))
grid on
xlabel('time')
ylabel('psi')
legend('actual','reference','Location','southeast')
title('Qruadrotor psi angle')

subplot(3,4,7)
hold on
plot(time,FxHistory)
plot(time,xHistory(:,1)-yreftot(:,1))
grid on
xlabel('time')
ylabel('psi')
legend('actual','reference','Location','southeast')
title('Qruadrotor x wind disturbance')

subplot(3,4,8)
hold on
plot(time,FyHistory)
plot(time,xHistory(:,2)-yreftot(:,2))
grid on
xlabel('time')
ylabel('psi')
legend('actual','reference','Location','southeast')
title('Qruadrotor y wind disturbance')

subplot(3,4,9)
hold on
plot(time,FzHistory)
plot(time,xHistory(:,3)-yreftot(:,3))
grid on
xlabel('time')
ylabel('psi')
legend('actual','reference','Location','southeast')
title('Qruadrotor z wind disturbance')

% subplot(3,4,10)
% hold on
% plot(time,xHistory)
% plot(time,x_est_history)
% grid on
% xlabel('time')
% ylabel('m/s')
% legend('reference','actual','Location','southeast')
% title('Qruadrotor noise vs EKF')

% Plot the manipulated variables.
fig = figure(2);      % Fokus pada figure 1
set(fig, 'Name', 'Drone Position Plot', 'NumberTitle', 'off'); % Beri nama pada window
subplot(2,2,1)
hold on
stairs(time,uHistory(:,1))
ylim([-0.5,12.5])
% plot(time,nloptions.MVTarget(2)*ones(1,length(time)))
plot(time,4.9*ones(1,length(time)))
grid on
xlabel('time')
legend('actual','reference')
title('Input 1')

subplot(2,2,2)
hold on
stairs(time,uHistory(:,2))
ylim([-0.5,12.5])
plot(time,4.9*ones(1,length(time)))
grid on
xlabel('time')
title('Input 2')
legend('actual','reference')

subplot(2,2,3)
hold on
stairs(time,uHistory(:,3))
ylim([-0.5,12.5])
plot(time,4.9*ones(1,length(time)))
grid on
xlabel('time')
title('Input 3')
legend('actual','reference')

subplot(2,2,4)
hold on
stairs(time,uHistory(:,4))
ylim([-0.5,12.5])
plot(time,4.9*ones(1,length(time)))
grid on
xlabel('time')
title('Input 4')
legend('actual','reference')

%fig=figure(3);
%plot(actuatorStatus);
%title('Aktuator Outputs dengan Simulasi Kegagalan');
%label('Waktu (step)');
%ylabel('Thrust Input');
%legend('Motor 1','Motor 2','Motor 3','Motor 4');

figure;
hold on;
grid on;

% Plot actual trajectory (X, Y, Z)
plot3(xHistory(:,1), xHistory(:,2), xHistory(:,3), 'b-', 'LineWidth', 1.5);

% Plot reference trajectory (Xref, Yref, Zref)
plot3(yreftot(:,1), yreftot(:,2), yreftot(:,3), 'r--', 'LineWidth', 1.5);

% Tambahkan titik start dan end
plot3(xHistory(1,1), xHistory(1,2), xHistory(1,3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g'); % Start
plot3(xHistory(end,1), xHistory(end,2), xHistory(end,3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % End

xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('Z Position (m)');
title('Quadrotor 3D Trajectory: Actual vs Reference');
legend('Actual Trajectory','Reference Trajectory','Start','End');
view(45,30); % sudut pandang biar jelas


