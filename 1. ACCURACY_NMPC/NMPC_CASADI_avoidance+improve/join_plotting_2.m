trajectory = 1;

obs_radius_val = 0.5; % semua radius sama
z_pos = 1.0;          % tinggi referensi (anggap obstacle menempel ground, drone terbang di atas)

% posisi obstacle (manual supaya jaraknya tidak terlalu dekat)
if trajectory == 1
    num_obs = 17; 
    obs_center = [ 6  6  6  8   12   12   15   15   16   18   18   22  23   24  26  29   30 ;   % x
                   9  6  3  1.8  4   6.8   6   1.5  8.2  6.2  2.8  2   4.8  8   4   2.2  7.0 ;   % y
                   z_pos*ones(1,num_obs) ];       % z

    obs_radius = obs_radius_val * ones(1,num_obs);
else
    num_obs = 2.0; 
    obs_center = [ 3.25 6.5;   % x
                   2.5 1.5;   % y
                   z_pos*ones(1,num_obs) ];       % z

    obs_radius = obs_radius_val * ones(1,num_obs);
end

% Load hasil simulasi
load('sim_single.mat','results'); results_single = results;
load('sim_multi.mat','results');  results_multi  = results;

% --- Time vector untuk state (N_sim+1) ---
t_single = (0:size(results_single.history_x,2)-1)*results_single.dt;
t_multi  = (0:size(results_multi.history_x,2)-1)*results_multi.dt;
x_ref = results_single.history_x_ref(1,:);
y_ref = results_single.history_x_ref(2,:);
z_ref = results_single.history_x_ref(3,:);

% --- Time vector untuk kontrol (N_sim) ---
t_u_single = (0:size(results_single.history_u,2)-1) * results_single.dt;
t_u_multi  = (0:size(results_multi.history_u,2)-1) * results_multi.dt;

% Plot the states.
fig = figure(1); % Fokus pada figure 1
set(fig, 'Name', 'Quadrotor Trajectory Comparison', 'NumberTitle', 'off');
clf(fig);

% --- Plot Posisi X ---
subplot(2,3,1);
hold on;
plot(t_single, results_single.history_x(1,:), 'b-', 'LineWidth', 1.5);
plot(t_multi, results_multi.history_x(1,:), 'g-', 'LineWidth', 1.5);
plot(t_single, results_single.history_x_ref(1,:), 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Position X (m)');
legend('Improvement APF X', 'Clasic APF X', 'Reference X', 'Location','best');
title('Quadrotor X Position');
hold off;

% --- Plot Posisi Y ---
subplot(2,3,2);
hold on;
plot(t_single, results_single.history_x(2,:), 'b-', 'LineWidth', 1.5);
plot(t_multi, results_multi.history_x(2,:), 'g-', 'LineWidth', 1.5);
plot(t_single, results_single.history_x_ref(2,:), 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Position Y (m)');
legend('Improvement APF Y', 'Clasic APF Y', 'Reference Y', 'Location','best');
title('Quadrotor Y Position');
hold off;

% --- Plot Posisi Z ---
subplot(2,3,3);
hold on;
plot(t_single, results_single.history_x(3,:), 'b-', 'LineWidth', 1.5);
plot(t_multi, results_multi.history_x(3,:), 'g-', 'LineWidth', 1.5);
plot(t_single, results_single.history_x_ref(3,:), 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Position Z (m)');
legend('Improvement APF Z', 'Clasic APF Z', 'Reference Z', 'Location','best');
title('Quadrotor Z Position');
hold off;

% --- Phi angle ---
subplot(2,3,4);
hold on;
plot(t_single, results_single.history_x(4,:), 'b-', 'LineWidth', 1.5);
plot(t_multi, results_multi.history_x(4,:), 'g-', 'LineWidth', 1.5);
plot(t_single, results_single.history_x_ref(4,:), 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Phi angle [rad]');
legend('Improvement APF', 'Clasic APF', 'Reference', 'Location','best');
title('Quadrotor Roll (Phi)');
hold off;

% --- Theta angle ---
subplot(2,3,5);
hold on;
plot(t_single, results_single.history_x(5,:), 'b-', 'LineWidth', 1.5);
plot(t_multi, results_multi.history_x(5,:), 'g-', 'LineWidth', 1.5);
plot(t_single, results_single.history_x_ref(5,:), 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Theta angle [rad]');
legend('Improvement APF', 'Clasic APF', 'Reference', 'Location','best');
title('Quadrotor Pitch (Theta)');
hold off;

% --- Psi angle ---
subplot(2,3,6);
hold on;
plot(t_single, results_single.history_x(6,:), 'b-', 'LineWidth', 1.5);
plot(t_multi, results_multi.history_x(6,:), 'g-', 'LineWidth', 1.5);
plot(t_single, results_single.history_x_ref(6,:), 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Psi angle [rad]');
legend('Improvement APF', 'Clasic APF', 'Reference', 'Location','best');
title('Quadrotor Yaw (Psi)');
hold off;

% Buat figure baru untuk perbandingan
figure;
hold on;
grid on;

% Plot input kontrol dari metode Single-NMPC
plot(t_u_single, results_single.history_u(1,:), 'b-', 'LineWidth', 1.5);
plot(t_u_single, results_single.history_u(2,:), 'r-', 'LineWidth', 1.5);
plot(t_u_single, results_single.history_u(3,:), 'g-', 'LineWidth', 1.5);
plot(t_u_single, results_single.history_u(4,:), 'm-', 'LineWidth', 1.5);

% Plot input kontrol dari metode Multi-NMPC dengan gaya garis putus-putus
plot(t_u_multi, results_multi.history_u(1,:), 'b--', 'LineWidth', 1.5);
plot(t_u_multi, results_multi.history_u(2,:), 'r--', 'LineWidth', 1.5);
plot(t_u_multi, results_multi.history_u(3,:), 'g--', 'LineWidth', 1.5);
plot(t_u_multi, results_multi.history_u(4,:), 'm--', 'LineWidth', 1.5);

xlabel('Time [s]');
ylabel('Control Input');
title('Comparison of Control Inputs');

% Buat legenda
legend('Improvement APF u1', 'Improvement APF u2', 'Improvement APF u3', 'Improvement APF u4', ...
       'Clasic APF u1', 'Clasic APF u2', 'Clasic APF u3', 'Clasic APF u4', ...
       'Location', 'best');

hold off;

% Asumsi: 'results_single' dan 'results_multi' telah dimuat
% Asumsi: 'obs_center', 'obs_radius', 'trajectory' telah didefinisikan

x_ref_single = results_single.history_x_ref(1, :);
y_ref_single = results_single.history_x_ref(2, :);
z_ref_single = results_single.history_x_ref(3, :);

x_actual_single = results_single.history_x(1, :);
y_actual_single = results_single.history_x(2, :);
z_actual_single = results_single.history_x(3, :);

x_actual_multi = results_multi.history_x(1, :);
y_actual_multi = results_multi.history_x(2, :);
z_actual_multi = results_multi.history_x(3, :);

figure; hold on; grid on; box on;

%% --- Plot Lintasan Reference dan Aktual ---
% Plot lintasan referensi (sama untuk kedua metode)
h_ref = plot3(x_ref_single, y_ref_single, z_ref_single, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Reference Path');

% Plot lintasan metode pertama (Single)
h_actual_single = plot3(x_actual_single, y_actual_single, z_actual_single, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Improvement APF');

% Plot lintasan metode kedua (Multi)
h_actual_multi = plot3(x_actual_multi, y_actual_multi, z_actual_multi, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Clasic APF Path');


%% --- Tandai Start & Finish ---
% Titik awal dan akhir untuk metode pertama
% h_start_act_single = plot3(x_actual_single(1), y_actual_single(1), z_actual_single(1), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 8, 'DisplayName', 'Start');
% h_finish_act_single = plot3(x_actual_single(end), y_actual_single(end), z_actual_single(end), 'bx', 'LineWidth', 2, 'MarkerSize', 10, 'DisplayName', 'Finish');

% % Titik awal dan akhir untuk metode kedua
% h_start_act_multi = plot3(x_actual_multi(1), y_actual_multi(1), z_actual_multi(1), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8, 'DisplayName', 'Start (Multi)');
% h_finish_act_multi = plot3(x_actual_multi(end), y_actual_multi(end), z_actual_multi(end), 'gx', 'LineWidth', 2, 'MarkerSize', 10, 'DisplayName', 'Finish (Multi)');

% Titik awal dan akhir referensi (hanya perlu satu)
h_start_ref = plot3(x_ref_single(1), y_ref_single(1), z_ref_single(1), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8, 'DisplayName', 'Start Reference');
h_finish_ref = plot3(x_ref_single(end), y_ref_single(end), z_ref_single(end), 'kx', 'LineWidth', 2, 'MarkerSize', 10, 'DisplayName', 'Finish Reference');

%% --- Drone Heading (quiver3) ---
% Panah untuk lintasan pertama
N_single = length(x_actual_single);
step_arrow_single = max(floor(N_single/20),1);
scale_arrow = 0.5;
for k = 1:step_arrow_single:N_single-1
    quiver3(x_actual_single(k), y_actual_single(k), z_actual_single(k), ...
            x_actual_single(k+1)-x_actual_single(k), ...
            y_actual_single(k+1)-y_actual_single(k), ...
            z_actual_single(k+1)-z_actual_single(k), ...
            scale_arrow, 'b', 'LineWidth', 1.5, 'MaxHeadSize', 2, 'HandleVisibility', 'off');
end

% Panah untuk lintasan kedua
N_multi = length(x_actual_multi);
step_arrow_multi = max(floor(N_multi/20),1);
for k = 1:step_arrow_multi:N_multi-1
    quiver3(x_actual_multi(k), y_actual_multi(k), z_actual_multi(k), ...
            x_actual_multi(k+1)-x_actual_multi(k), ...
            y_actual_multi(k+1)-y_actual_multi(k), ...
            z_actual_multi(k+1)-z_actual_multi(k), ...
            scale_arrow, 'g', 'LineWidth', 1.5, 'MaxHeadSize', 2, 'HandleVisibility', 'off');
end
h_arrow_single = plot3(nan, nan, nan, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Improvement APF Heading');
h_arrow_multi = plot3(nan, nan, nan, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Clasic APF Heading');

%% --- Plot Obstacles (Silinder Hijau Tua) ---
num_obs = size(obs_center,2);
if trajectory == 1
    obstacle_height = 8;
else
    obstacle_height = 2;%8;
end
safety_scale = 1.2; % safety margin

for j = 1:num_obs
    c = obs_center(:,j);
    r = obs_radius(j);

    [x_cyl, y_cyl, z_cyl] = cylinder(r,50);
    z_cyl = z_cyl * obstacle_height;
    x_cyl = x_cyl + c(1);
    y_cyl = y_cyl + c(2);

    surf(x_cyl, y_cyl, z_cyl, 'FaceAlpha', 0.5, 'FaceColor', [0.52 0.21 0.57], 'EdgeColor', 'none');

    [x_safe, y_safe, z_safe] = cylinder(r*safety_scale,50);
    z_safe = z_safe*obstacle_height;
    x_safe = x_safe + c(1);
    y_safe = y_safe + c(2);
    surf(x_safe, y_safe, z_safe, 'FaceAlpha', 0.1, 'FaceColor', 'r', 'EdgeColor', 'none', 'HandleVisibility', 'off');
end
h_obs = plot3(nan, nan, nan, 's', 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k', 'DisplayName', 'Obstacles');


%% --- Axis, Labels, View ---
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Drone Trajectory Comparison with NMPC-APF + Obstacles', 'FontSize', 12);
view(45,25);

% Atur batas sumbu agar konsisten untuk kedua plot
all_x = [x_ref_single, x_actual_single, x_actual_multi];
all_y = [y_ref_single, y_actual_single, y_actual_multi];
all_z = [z_ref_single, z_actual_single, z_actual_multi];

xlim([min(all_x)-1, max(all_x)+1]);
ylim([min(all_y)-1, max(all_y)+1]);
zlim([min(all_z)-1, max(all_z)+1]);

axis equal; grid on;

%% --- Legend (informatif) ---
legend([h_actual_single, h_actual_multi, h_ref, h_arrow_single, h_arrow_multi, h_obs, h_start_ref, h_finish_ref], 'Location','bestoutside');


figure; hold on; grid on; box on;

%% --- Plot Lintasan Referensi dan Aktual ---
h_ref = plot(x_ref, y_ref, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Reference Path'); % Jalur referensi
h_actual_single = plot(x_actual_single, y_actual_single, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Improvement APF Path'); % Jalur Single-NMPC
h_actual_multi = plot(x_actual_multi, y_actual_multi, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Clasic APF Path'); % Jalur Multi-NMPC

%% --- Obstacles (lingkaran 2D) ---
theta = linspace(0, 2*pi, 100);
colors = lines(size(obs_center, 2));
num_obs = size(obs_center, 2);
obs_radius = obs_radius_val * ones(1, num_obs);

for j = 1:num_obs
    x_circle = obs_center(1, j) + obs_radius(j) * cos(theta);
    y_circle = obs_center(2, j) + obs_radius(j) * sin(theta);
    fill(x_circle, y_circle, colors(j, :), 'FaceAlpha', 0.3, 'EdgeColor', 'k');
end
h_obs = plot(nan, nan, 'o', 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k', 'DisplayName', 'Obstacles');

%% --- Tandai Start & Finish ---
h_start_ref = plot(x_ref(1), y_ref(1), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k', 'DisplayName', 'Start Reference');
h_finish_ref = plot(x_ref(end), y_ref(end), 'kx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Finish Reference');
% 
% h_start_single = plot(x_actual_single(1), y_actual_single(1), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'DisplayName', 'Start Improvement APF');
% h_finish_single = plot(x_actual_single(end), y_actual_single(end), 'bx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Finish Improvement APF');

% h_start_multi = plot(x_actual_multi(1), y_actual_multi(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Start Multi-NMPC');
% h_finish_multi = plot(x_actual_multi(end), y_actual_multi(end), 'gx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Finish Multi-NMPC');

%% --- Drone Heading (quiver) ---
scale_arrow = 1;

% Quiver untuk metode single
N_single = length(x_actual_single);
step_arrow_single = max(floor(N_single / 20), 1);
for k = 1:step_arrow_single:N_single - 1
    quiver(x_actual_single(k), y_actual_single(k), ...
           x_actual_single(k+1) - x_actual_single(k), ...
           y_actual_single(k+1) - y_actual_single(k), ...
           scale_arrow, 'b', 'LineWidth', 1.5, 'MaxHeadSize', 3, 'HandleVisibility', 'off');
end

% Quiver untuk metode multi
N_multi = length(x_actual_multi);
step_arrow_multi = max(floor(N_multi / 20), 1);
for k = 1:step_arrow_multi:N_multi - 1
    quiver(x_actual_multi(k), y_actual_multi(k), ...
           x_actual_multi(k+1) - x_actual_multi(k), ...
           y_actual_multi(k+1) - y_actual_multi(k), ...
           scale_arrow, 'g', 'LineWidth', 1.5, 'MaxHeadSize', 3, 'HandleVisibility', 'off');
end
h_arrow_single = plot(nan, nan, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Improvement APF Heading');
h_arrow_multi = plot(nan, nan, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Clasic APF Heading');

%% --- Axis, Labels, Title ---
xlabel('X [m]');
ylabel('Y [m]');
title('Drone Trajectory (Top View) Comparison');
axis equal;
grid on;
box on;

%% --- Legend yang lebih informatif ---
legend([h_actual_single, h_actual_multi, h_ref, h_arrow_single, h_arrow_multi, h_start_ref, h_finish_ref, h_obs], 'Location', 'bestoutside');


% Asumsi: 'results_single', 'results_multi', 'obs_center', dan 'obs_radius' telah dimuat

%% --- Hitung metrik untuk Single-NMPC ---
N_actual_single = size(results_single.history_x, 2);
time_single = (0:N_actual_single-1) * results_single.dt;
d_min_log_single = zeros(1, N_actual_single);
for i = 1:N_actual_single
    p = results_single.history_x(1:3, i);
    d_all = vecnorm(p - obs_center, 2, 1) - obs_radius;
    d_min_log_single(i) = min(d_all);
end
v_act_norm_single = sqrt(sum(results_single.history_x(7:9, 1:N_actual_single).^2, 1));

%% --- Hitung metrik untuk Multi-NMPC ---
N_actual_multi = size(results_multi.history_x, 2);
time_multi = (0:N_actual_multi-1) * results_multi.dt;
d_min_log_multi = zeros(1, N_actual_multi);
for i = 1:N_actual_multi
    p = results_multi.history_x(1:3, i);
    d_all = vecnorm(p - obs_center, 2, 1) - obs_radius;
    d_min_log_multi(i) = min(d_all);
end
v_act_norm_multi = sqrt(sum(results_multi.history_x(7:9, 1:N_actual_multi).^2, 1));

%% --- Plot Perbandingan ---
figure('Name', 'Drone Distance & Speed Comparison', 'Color', 'w');
hold on; box on; grid on;

d_safe = 1.0; % Threshold aman [m]

% --- Plot Jarak Minimum (sumbu kiri) ---
yyaxis left
h_dist_single = plot(time_single, d_min_log_single, 'b-', 'LineWidth', 1.5);
h_dist_multi = plot(time_multi, d_min_log_multi, 'g--', 'LineWidth', 1.5);
ylabel('Minimum Distance to Obstacles [m]');

% Gabungkan data jarak untuk mendapatkan batas sumbu yang konsisten
all_dist = [d_min_log_single, d_min_log_multi];
ylim([0, max(all_dist) * 1.2]);
yline(d_safe, 'r-', 'LineWidth', 1.5, 'Label', 'Safe Distance', 'LabelHorizontalAlignment', 'right');

% --- Plot Kecepatan Total (sumbu kanan) ---
yyaxis right
h_speed_single = plot(time_single, v_act_norm_single, 'b:', 'LineWidth', 1.5);
h_speed_multi = plot(time_multi, v_act_norm_multi, 'g-.', 'LineWidth', 1.5);
ylabel('Drone Total Speed [m/s]');

% Gabungkan data kecepatan untuk mendapatkan batas sumbu yang konsisten
all_speed = [v_act_norm_single, v_act_norm_multi];
ylim([0, max(all_speed) * 1.2]);

% --- Labels & Title ---
xlabel('Time [s]');
title('Drone Minimum Distance to Obstacles and Total Speed','FontSize',12);

% --- Legend yang lebih informatif ---
legend([h_dist_single, h_dist_multi, h_speed_single, h_speed_multi], ...
       {'Distance (Improvement APF)', 'Distance (Clasic APF)', 'Speed (Improvement APF)', 'Speed (Clasic APF)'}, ...
       'Location', 'best');

set(gca, 'FontSize', 11);
grid on; box on;


%% --- Publikasi-ready Plotting Kecepatan Drone --- %%
% Asumsi: results_single dan results_multi telah dimuat
% dan memiliki history_x, history_x_ref, dan dt

% Dapatkan data dan waktu untuk kedua metode
t_single = (0:size(results_single.history_x, 2)-1) * results_single.dt;
t_multi = (0:size(results_multi.history_x, 2)-1) * results_multi.dt;

% Buat figure dengan ukuran yang disesuaikan
figure('Name','Drone Linear Velocity and Speed Comparison','Color','w', 'Position', [100 100 800 900]);

% Warna dan style
colors = {'b','r','g'};
style_ref = '--'; % Garis putus-putus untuk referensi
style_single = '-'; % Garis solid untuk Single-NMPC
style_multi = ':'; % Garis titik-titik untuk Multi-NMPC

%% --- VX, VY, VZ Subplots ---
components = {'v_x','v_y','v_z'};
for i = 1:3
    subplot(4,1,i);
    hold on;
    
    % Plot NMPC reference (sama untuk kedua metode)
    h_ref = plot(t_single, results_single.history_x_ref(6+i,:), [colors{i} style_ref], 'LineWidth', 1.5);
    
    % Plot Actual Single-NMPC
    h_single = plot(t_single, results_single.history_x(6+i,:), [colors{i} style_single], 'LineWidth', 1.5);
    
    % Plot Actual Multi-NMPC
    h_multi = plot(t_multi, results_multi.history_x(6+i,:), [colors{i} style_multi], 'LineWidth', 1.5);
    
    grid on; box on;
    xlabel('Time [s]');
    ylabel([components{i} ' [m/s]']);
    
    legend([h_ref, h_single, h_multi], 'NMPC reference', 'Actual (Improvement APF)', 'Actual (Clasic APF)', 'Location', 'best');
    title(['Quadrotor Linear Velocity in ', strrep(components{i},'_','-')]);
    hold off;
end

%% --- Total Speed Subplot ---
v_ref_norm = sqrt(sum(results_single.history_x_ref(7:9,:).^2,1));
v_single_norm = sqrt(sum(results_single.history_x(7:9,:).^2,1));
v_multi_norm = sqrt(sum(results_multi.history_x(7:9,:).^2,1));
 
subplot(4,1,4);
hold on;
plot(t_single, v_ref_norm, 'k--', 'LineWidth', 1.5);
plot(t_single, v_single_norm, 'b-', 'LineWidth', 1.5);
plot(t_multi, v_multi_norm, 'g:', 'LineWidth', 1.5);

grid on; box on;
xlabel('Time [s]');
ylabel('Speed [m/s]');
legend('NMPC reference speed', 'Actual speed (Improvement APF)', 'Actual speed (Clasic APF)', 'Location', 'best');
title('Comparison of Drone Total Linear Speed');
hold off;