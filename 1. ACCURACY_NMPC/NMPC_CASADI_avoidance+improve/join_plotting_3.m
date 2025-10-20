close all;
trajectory = 1;

obs_radius_val = 0.5; % semua radius sama
z_pos = 0.0;          % tinggi referensi (anggap obstacle menempel ground, drone terbang di atas)

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
load('2sim_single.mat','results'); results_single = results;
load('2sim_multi.mat','results');  results_multi  = results;
load('2sim_multi3.mat','results');  results_multi3  = results;

% --- Time vector untuk state (N_sim+1) ---
t_single = (0:size(results_single.history_x,2)-1)*results_single.dt;
t_multi  = (0:size(results_multi.history_x,2)-1)*results_multi.dt;
t_multi3  = (0:size(results_multi3.history_x,2)-1)*results_multi3.dt;
x_ref = results_single.history_x_ref(1,:);
y_ref = results_single.history_x_ref(2,:);
z_ref = results_single.history_x_ref(3,:);

% --- Time vector untuk kontrol (N_sim) ---
t_u_single = (0:size(results_single.history_u,2)-1) * results_single.dt;
t_u_multi  = (0:size(results_multi.history_u,2)-1) * results_multi.dt;
t_u_multi3  = (0:size(results_multi3.history_u,2)-1) * results_multi3.dt;

color1 = [0.00,0.30,0.60];
color2 = [0.80,0.40,0.00];
color3 = [0.30,0.50,0.30];

start = 1.2;
finish = 10.6;

%% =========================================================================
%  FIGURE 1: POSITIONS (px, py, pz)
%  =========================================================================
fig1 = figure(1); 
set(fig1, 'Name', 'Quadrotor Position Comparison', 'NumberTitle', 'off');
clf(fig1);

% Define plot indices for Position (1, 2, 3)
pos_indices = [1, 2, 3];
pos_labels = {'Position X (m)', 'Position Y (m)', 'Position Z (m)'};
pos_titles = {'Quadrotor X Position', 'Quadrotor Y Position', 'Quadrotor Z Position'};
ref_labels = {'Reference X', 'Reference Y', 'Reference Z'};

for i = 1:3
    idx = pos_indices(i); % 1, 2, or 3
    
    % KOREKSI: Menggunakan 3 baris, 1 kolom (Susunan Vertikal)
    subplot(3, 1, i);
    hold on;
    
    % Plot Actual Trajectories
    h_single = plot(t_single, results_single.history_x(idx,:), 'Color', color1, 'LineWidth', 1.5, 'DisplayName', 'Hybrid APF–NMPC');
    h_multi = plot(t_multi, results_multi.history_x(idx,:), 'Color', color2, 'LineWidth', 1.5, 'DisplayName', 'Cost-Augmented APF–NMPC');
    h_multi3 = plot(t_multi3, results_multi3.history_x(idx,:), 'Color', color3, 'LineWidth', 1.5, 'DisplayName', 'APF-Embedded Reference NMPC');
    
    % Plot Reference Trajectory
    h_ref = plot(t_single, results_single.history_x_ref(idx,:), 'k--', 'LineWidth', 1.5, 'DisplayName', ref_labels{i});
    
    grid on;
    ylabel(pos_labels{i});
    title(pos_titles{i});
    
    % Sumbu X hanya pada plot terakhir
    xlabel('Time (s)');
%     set(gca, 'XTickLabel', []); % Sembunyikan label X pada plot atas
    
    % Tambahkan legenda hanya pada plot pertama
    legend([h_single, h_multi, h_multi3, h_ref], 'Location','best', 'NumColumns', 2);

    hold off;
end


%% =========================================================================
%  FIGURE 2: EULER ANGLES (Phi, Theta, Psi)
%  =========================================================================
fig2 = figure(2); 
set(fig2, 'Name', 'Quadrotor Angle Comparison', 'NumberTitle', 'off');
clf(fig2);

% Define plot indices for Angles (4, 5, 6)
angle_indices = [4, 5, 6];
angle_labels = {'\phi (Roll) [deg]', '\theta (Pitch) [deg]', '\psi (Yaw) [deg]'};
angle_titles = {'Quadrotor Roll (\phi)', 'Quadrotor Pitch (\theta)', 'Quadrotor Yaw (\psi)'};

for i = 1:3
    idx = angle_indices(i); % 4, 5, or 6
    
    % KOREKSI: Menggunakan 3 baris, 1 kolom (Susunan Vertikal)
    subplot(3, 1, i);
    hold on;
    
    % Plot Actual Angles
    h_single = plot(t_single, results_single.history_x(idx,:), 'Color', color1, 'LineWidth', 1.5, 'DisplayName', 'Hybrid APF–NMPC');
    h_multi = plot(t_multi, results_multi.history_x(idx,:), 'Color', color2, 'LineWidth', 1.5, 'DisplayName', 'Cost-Augmented APF–NMPC');
    h_multi3 = plot(t_multi3, results_multi3.history_x(idx,:), 'Color', color3, 'LineWidth', 1.5, 'DisplayName', 'APF-Embedded Reference NMPC');
    
    % Plot Reference Angle
    h_ref = plot(t_single, results_single.history_x_ref(idx,:), 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
    
    grid on;
    ylabel(angle_labels{i});
    title(angle_titles{i});
    
    % Sumbu X hanya pada plot terakhir
    xlabel('Time [s]');
%     set(gca, 'XTickLabel', []); % Sembunyikan label X pada plot atas
    
    % Tambahkan legenda hanya pada plot pertama
    legend([h_single, h_multi, h_multi3, h_ref], 'Location','best', 'NumColumns', 2);

    hold off;
end

%% === Compute Average Thrust ===
T_avg_single = mean(results_single.history_u, 1);
T_avg_multi  = mean(results_multi.history_u, 1);
T_avg_multi3 = mean(results_multi3.history_u, 1);

%% === Compute Control Effort Metrics ===
J_u_single = sum(sum(results_single.history_u.^2, 1));
J_u_multi  = sum(sum(results_multi.history_u.^2, 1));
J_u_multi3 = sum(sum(results_multi3.history_u.^2, 1));

mean_u_single = mean(T_avg_single);
mean_u_multi  = mean(T_avg_multi);
mean_u_multi3 = mean(T_avg_multi3);

max_u_single = max(T_avg_single);
max_u_multi  = max(T_avg_multi);
max_u_multi3 = max(T_avg_multi3);

%% === PLOT 1: Average Thrust Comparison ===
figure('Name','Average Thrust Comparison','Color','w');
hold on; box on; grid on;

plot(t_u_single, T_avg_single, 'Color', color1, 'LineWidth', 1.8);
plot(t_u_multi,  T_avg_multi,  'Color', color2, 'LineWidth', 1.8);
plot(t_u_multi3, T_avg_multi3, 'Color', color3, 'LineWidth', 1.8);

xlabel('Time [s]');
ylabel('Average Thrust [N]');
title('Average Thrust Comparison between NMPC-APF Methods');

% (Optional) Mark obstacle avoidance window
% xline(start, '--', 'Avoidance Start', 'Color', [0.5 0.5 0.5], 'LabelHorizontalAlignment', 'center','LabelVerticalAlignment','top');
% xline(finish, '--', 'Avoidance End', 'Color', [0.5 0.5 0.5], 'LabelHorizontalAlignment', 'center','LabelVerticalAlignment','top');

legend({'Hybrid APF–NMPC', ...
        'Cost-Augmented APF–NMPC', ...
        'APF-Embedded Reference NMPC'}, ...
        'Location','best');
set(gca,'FontSize',11);

ylim([min([T_avg_single,T_avg_multi,T_avg_multi3])*0.9, ...
      max([T_avg_single,T_avg_multi,T_avg_multi3])*1.1]);

%% === PLOT 2: Thrust per Motor (Optional, use one representative motor) ===
figure('Name','Thrust Command (Motor 1)','Color','w');
hold on; box on; grid on;

plot(t_u_single, results_single.history_u(1,:), 'Color', color1, 'LineWidth', 1.5);
plot(t_u_multi,  results_multi.history_u(1,:),  'Color', color2, 'LineWidth', 1.5);
plot(t_u_multi3, results_multi3.history_u(1,:), 'Color', color3, 'LineWidth', 1.5);

xlabel('Time [s]');
ylabel('Thrust Motor 1 [N]');
title('Thrust Command Comparison (Representative Motor)');
legend({'Hybrid APF–NMPC', 'Cost-Augmented APF–NMPC', 'APF-Embedded Reference NMPC'}, 'Location','best');
set(gca,'FontSize',11);

% (Optional) Mark obstacle avoidance window
% xline(start, '--', 'Avoidance Start', 'Color', [0.5 0.5 0.5], 'LabelVerticalAlignment','bottom');
% xline(finish, '--', 'Avoidance End', 'Color', [0.5 0.5 0.5], 'LabelVerticalAlignment','bottom');

%% === TABLE: Control Effort Metrics ===
fprintf('\n=== CONTROL EFFORT METRICS ===\n');
fprintf('%-35s %-15s %-15s %-15s\n', 'Method', 'Σ||u||²', 'Mean Thrust [N]', 'Max Thrust [N]');
fprintf('---------------------------------------------------------------------------------\n');
fprintf('%-35s %-15.3f %-15.3f %-15.3f\n', 'Hybrid APF–NMPC', J_u_single, mean_u_single, max_u_single);
fprintf('%-35s %-15.3f %-15.3f %-15.3f\n', 'Cost-Augmented APF–NMPC',         J_u_multi,  mean_u_multi,  max_u_multi);
fprintf('%-35s %-15.3f %-15.3f %-15.3f\n', 'APF-Embedded Reference NMPC',        J_u_multi3, mean_u_multi3, max_u_multi3);

%% === (Optional) Convert metrics to table for plotting/export ===
ControlEffortTable = table( ...
    {'Hybrid APF–NMPC'; 'Cost-Augmented APF–NMPC'; 'APF-Embedded Reference NMPC'}, ...
    [J_u_single; J_u_multi; J_u_multi3], ...
    [mean_u_single; mean_u_multi; mean_u_multi3], ...
    [max_u_single; max_u_multi; max_u_multi3], ...
    'VariableNames', {'Method','TotalEffort','MeanThrust','MaxThrust'});

disp(ControlEffortTable);

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

x_actual_multi3 = results_multi3.history_x(1, :);
y_actual_multi3 = results_multi3.history_x(2, :);
z_actual_multi3 = results_multi3.history_x(3, :);

figure; hold on; grid on; box on;

%% --- Plot Lintasan Reference dan Aktual ---
% Plot lintasan referensi (sama untuk kedua metode)
h_ref = plot3(x_ref_single, y_ref_single, z_ref_single, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference Path');

% Plot lintasan metode pertama (Single)
h_actual_single = plot3(x_actual_single, y_actual_single, z_actual_single, 'Color', color1, 'LineWidth', 1.5, 'DisplayName', 'Hybrid APF–NMPC');

% Plot lintasan metode kedua (Multi)
h_actual_multi = plot3(x_actual_multi, y_actual_multi, z_actual_multi, 'Color', color2, 'LineWidth', 1.5, 'DisplayName', 'Cost-Augmented APF–NMPC');

% Plot lintasan metode kedua (Multi)
h_actual_multi3 = plot3(x_actual_multi3, y_actual_multi3, z_actual_multi3, 'Color', color3, 'LineWidth', 1.5, 'DisplayName', 'APF-Embedded Reference NMPC');


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
            scale_arrow, 'Color', color1, 'LineWidth', 1.5, 'MaxHeadSize', 2, 'HandleVisibility', 'off');
end

% Panah untuk lintasan kedua
N_multi = length(x_actual_multi);
step_arrow_multi = max(floor(N_multi/20),1);
for k = 1:step_arrow_multi:N_multi-1
    quiver3(x_actual_multi(k), y_actual_multi(k), z_actual_multi(k), ...
            x_actual_multi(k+1)-x_actual_multi(k), ...
            y_actual_multi(k+1)-y_actual_multi(k), ...
            z_actual_multi(k+1)-z_actual_multi(k), ...
            scale_arrow, 'Color', color2, 'LineWidth', 1.5, 'MaxHeadSize', 2, 'HandleVisibility', 'off');
end

% Panah untuk lintasan kedua
N_multi3 = length(x_actual_multi3);
step_arrow_multi3 = max(floor(N_multi3/20),1);
for k = 1:step_arrow_multi3:N_multi3-1
    quiver3(x_actual_multi3(k), y_actual_multi3(k), z_actual_multi3(k), ...
            x_actual_multi3(k+1)-x_actual_multi3(k), ...
            y_actual_multi3(k+1)-y_actual_multi3(k), ...
            z_actual_multi3(k+1)-z_actual_multi3(k), ...
            scale_arrow, 'Color', color3, 'LineWidth', 1.5, 'MaxHeadSize', 2, 'HandleVisibility', 'off');
end
h_arrow_single = plot3(nan, nan, nan, 'Color', color1, 'LineWidth', 1.5, 'DisplayName', 'Hybrid APF–NMPC');
h_arrow_multi = plot3(nan, nan, nan, 'Color', color2, 'LineWidth', 1.5, 'DisplayName', 'Cost-Augmented APF–NMPC');
h_arrow_multi3 = plot3(nan, nan, nan, 'Color', color3, 'LineWidth', 1.5, 'DisplayName', 'APF-Embedded Reference NMPC');

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
h_obs = plot3(nan, nan, nan, 'ko', 'MarkerFaceColor', 'm', 'MarkerEdgeColor', 'k', 'DisplayName', 'Obstacles');


%% --- Axis, Labels, View ---
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Drone Trajectory Comparison', 'FontSize', 12);
view(45,25);

% Atur batas sumbu agar konsisten untuk kedua plot
all_x = [x_ref_single, x_actual_single, x_actual_multi, x_actual_multi3];
all_y = [y_ref_single, y_actual_single, y_actual_multi, y_actual_multi3];
all_z = [z_ref_single, z_actual_single, z_actual_multi, z_actual_multi3];

xlim([min(all_x)-1, max(all_x)+1]);
ylim([min(all_y)-1, max(all_y)+1]);
zlim([min(all_z)-1, max(all_z)+1]);

axis equal; grid on;

%% --- Legend (informatif) ---
legend([h_actual_single, h_actual_multi, h_actual_multi3, h_ref, h_obs, h_start_ref, h_finish_ref], 'Location','bestoutside');


figure; hold on; grid on; box on;

%% --- Plot Lintasan Referensi dan Aktual ---
h_ref = plot(x_ref, y_ref, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference Path'); % Jalur referensi
h_actual_single = plot(x_actual_single, y_actual_single, 'Color', color1, 'LineWidth', 1.5, 'DisplayName', 'Hybrid APF–NMPC'); % Jalur Single-NMPC
h_actual_multi = plot(x_actual_multi, y_actual_multi, 'Color', color2, 'LineWidth', 1.5, 'DisplayName', 'Cost-Augmented APF–NMPC'); % Jalur Multi-NMPC
h_actual_multi3 = plot(x_actual_multi3, y_actual_multi3, 'Color', color3, 'LineWidth', 1.5, 'DisplayName', 'APF-Embedded Reference NMPC'); % Jalur Multi-NMPC

%% --- Obstacles (lingkaran 2D) ---
% --- Inisialisasi Parameter ---
theta = linspace(0, 2*pi, 100);
num_obs = size(obs_center, 2);
obs_radius = obs_radius_val * ones(1, num_obs); % Radius rintangan fisik

obstacle_color = [0.5 0.5 0.5]; % Warna abu-abu solid untuk semua rintangan
d_safe = 0.5; % Ganti dengan nilai d_safe aktual Anda

% --- Loop untuk Menggambar Setiap Rintangan ---
for j = 1:num_obs
    % 1. Gambar Rintangan Fisik (Warna Solid Tunggal)
    x_obs = obs_center(1, j) + obs_radius(j) * cos(theta);
    y_obs = obs_center(2, j) + obs_radius(j) * sin(theta);
    
    fill(x_obs, y_obs, obstacle_color, 'FaceAlpha', 0.6, 'EdgeColor', 'k', 'LineWidth', 1);
    
    % 2. Gambar Lingkaran Jarak Aman (Safe Distance Margin)
    R_safe_zone = obs_radius(j) + d_safe; % Radius Total: Fisik + Jarak Aman
    
    x_safe = obs_center(1, j) + R_safe_zone * cos(theta);
    y_safe = obs_center(2, j) + R_safe_zone * sin(theta);
    
    plot(x_safe, y_safe, 'k:', 'LineWidth', 1.2, 'HandleVisibility', 'off'); % Garis putus-putus hitam (dotted)
end

% --- Handle Legenda (untuk Rintangan dan Zona Aman) ---
% Tambahkan handle placeholder untuk legenda yang akurat
h_obs = plot(nan, nan, 's', 'MarkerFaceColor', obstacle_color, 'MarkerEdgeColor', 'k', 'DisplayName', 'Obstacles');
h_safe = plot(nan, nan, 'k:', 'LineWidth', 1.2, 'DisplayName', ['Safe zone (R_{safe} = ' num2str(d_safe) 'm)']);

% Hapus plot placeholder lama jika ada
% h_obs_old = plot(nan, nan, 'o', 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k', 'DisplayName', 'Obstacles');

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
           scale_arrow, 'Color', color1, 'LineWidth', 1.5, 'MaxHeadSize', 3, 'HandleVisibility', 'off');
end

% Quiver untuk metode multi
N_multi = length(x_actual_multi);
step_arrow_multi = max(floor(N_multi / 20), 1);
for k = 1:step_arrow_multi:N_multi - 1
    quiver(x_actual_multi(k), y_actual_multi(k), ...
           x_actual_multi(k+1) - x_actual_multi(k), ...
           y_actual_multi(k+1) - y_actual_multi(k), ...
           scale_arrow, 'Color', color2, 'LineWidth', 1.5, 'MaxHeadSize', 3, 'HandleVisibility', 'off');
end

N_multi3 = length(x_actual_multi3);
step_arrow_multi3 = max(floor(N_multi3 / 20), 1);
for k = 1:step_arrow_multi3:N_multi3 - 1
    quiver(x_actual_multi3(k), y_actual_multi3(k), ...
           x_actual_multi3(k+1) - x_actual_multi3(k), ...
           y_actual_multi3(k+1) - y_actual_multi3(k), ...
           scale_arrow, 'Color', color3, 'LineWidth', 1.5, 'MaxHeadSize', 3, 'HandleVisibility', 'off');
end
% h_arrow_single = plot(nan, nan, 'Color', color1, 'LineWidth', 1.5, 'DisplayName', 'Improvement APF Heading');
% h_arrow_multi = plot(nan, nan, 'Color', color3, 'LineWidth', 1.5, 'DisplayName', 'Clasic APF Heading');


h_arrow_single = plot(nan, nan, 'Color', color1, 'LineWidth', 1.5, 'DisplayName', 'Hybrid APF–NMPC');
h_arrow_multi = plot(nan, nan, 'Color', color2, 'LineWidth', 1.5, 'DisplayName', 'Cost-Augmented APF–NMPC');
h_arrow_multi3 = plot(nan, nan, 'Color', color3, 'LineWidth', 1.5, 'DisplayName', 'APF-Embedded Reference NMPC');

%% --- Axis, Labels, Title ---
xlabel('X [m]');
ylabel('Y [m]');
title('Drone Trajectory (Top View) Comparison');
axis equal;
grid on;
box on;

%% --- Legend yang lebih informatif ---
legend([h_actual_single, h_actual_multi, h_actual_multi3,h_ref, h_start_ref, h_finish_ref, h_obs], 'Location', 'northeast');


% Asumsi: 'results_single', 'results_multi', 'obs_center', dan 'obs_radius' telah dimuat

%% --- Hitung metrik untuk Single-NMPC ---
% N_single = size(results_single.history_x, 2);
% t_single = (0:N_single-1) * results_single.dt;
% d_min_log_single = zeros(1, N_single);
% for i = 1:N_single
%     p = results_single.history_x(1:3, i);
%     d_all = vecnorm(p - obs_center, 2, 1) - obs_radius;
%     d_min_log_single(i) = min(d_all);
% end

% Ambil hanya koordinat X dan Y dari pusat rintangan (Z diabaikan)
obs_center_2D = obs_center(1:2, :); 
obs_radius_2D = obs_radius; % Radius tetap sama

for i = 1:N_single
    % PENTING: Ambil hanya X dan Y dari posisi drone
    p_2D = results_single.history_x(1:2, i); 
    
    % Hitung jarak horizontal (2D) dari pusat drone ke pusat rintangan
    distance_to_center_2D = vecnorm(p_2D - obs_center_2D, 2, 1);
    
    % Hitung jarak minimum ke permukaan (2D)
    d_all_2D = distance_to_center_2D - obs_radius_2D;
    d_min_log_single(i) = min(d_all_2D);
end


v_act_norm_single = sqrt(sum(results_single.history_x(7:9, 1:N_single).^2, 1));

%% --- Hitung metrik untuk Multi-NMPC ---
% N_multi = size(results_multi.history_x, 2);
% t_multi = (0:N_multi-1) * results_multi.dt;
% d_min_log_multi = zeros(1, N_multi);
% for i = 1:N_multi
%     p = results_multi.history_x(1:3, i);
%     d_all = vecnorm(p - obs_center, 2, 1) - obs_radius;
%     d_min_log_multi(i) = min(d_all);
% end

obs_center_2D = obs_center(1:2, :); 
obs_radius_2D = obs_radius; % Radius tetap sama

for i = 1:N_multi
    % PENTING: Ambil hanya X dan Y dari posisi drone
    p_2D = results_multi.history_x(1:2, i); 
    
    % Hitung jarak horizontal (2D) dari pusat drone ke pusat rintangan
    distance_to_center_2D = vecnorm(p_2D - obs_center_2D, 2, 1);
    
    % Hitung jarak minimum ke permukaan (2D)
    d_all_2D = distance_to_center_2D - obs_radius_2D;
    d_min_log_multi(i) = min(d_all_2D);
end

v_act_norm_multi = sqrt(sum(results_multi.history_x(7:9, 1:N_multi).^2, 1));

% N_multi3 = size(results_multi3.history_x, 2);
% t_multi3 = (0:N_multi3-1) * results_multi3.dt;
% d_min_log_multi3 = zeros(1, N_multi3);
% for i = 1:N_multi3
%     p = results_multi3.history_x(1:3, i);
%     d_all = vecnorm(p - obs_center, 2, 1) - obs_radius;
%     d_min_log_multi3(i) = min(d_all);
% end

obs_center_2D = obs_center(1:2, :); 
obs_radius_2D = obs_radius; % Radius tetap sama

for i = 1:N_single
    % PENTING: Ambil hanya X dan Y dari posisi drone
    p_2D = results_multi3.history_x(1:2, i); 
    
    % Hitung jarak horizontal (2D) dari pusat drone ke pusat rintangan
    distance_to_center_2D = vecnorm(p_2D - obs_center_2D, 2, 1);
    
    % Hitung jarak minimum ke permukaan (2D)
    d_all_2D = distance_to_center_2D - obs_radius_2D;
    d_min_log_multi3(i) = min(d_all_2D);
end

v_act_norm_multi3 = sqrt(sum(results_multi3.history_x(7:9, 1:N_multi3).^2, 1));


%% --- Plot 1: Minimum Distance to Obstacles Comparison ---
figure('Name', 'Minimum Distance to Obstacles Comparison', 'Color', 'w');
hold on; 
box on; 
grid on;

% --- Plot Minimum Distance ---
% 't_single', 'd_min_log_single' likely refer to your Hybrid NMPC approach
h_dist_hybrid = plot(t_single, d_min_log_single, 'Color', color1, 'LineWidth', 1.5);

% 't_multi', 'd_min_log_multi' likely refer to the Cost-only APF benchmark
h_dist_cost_only = plot(t_multi, d_min_log_multi, 'Color', color2, 'LineWidth', 1.5);

% 't_multi3', 'd_min_log_multi3' likely refer to the Input-Bias APF benchmark
h_dist_input_bias = plot(t_multi3, d_min_log_multi3, 'Color', color3, 'LineWidth', 1.5);

ylabel('Minimum Distance to Obstacles [m]');
xlabel('Time [s]');
title('Comparison of Drone Minimum Distance to Obstacles');

% Combine distance data to set consistent axis limits
all_dist = [d_min_log_single, d_min_log_multi, d_min_log_multi3];
ylim([0, max(all_dist) * 1.2]);

% Safe Distance Threshold Line
h_safe_line = yline(0.5, 'k', 'LineWidth', 1.5, 'Label', 'safety boundary (d{safe})', 'LabelHorizontalAlignment', 'right', ...
    'LabelVerticalAlignment', 'bottom');

% Legend for Distance Plot
legend([h_dist_hybrid, h_dist_cost_only, h_dist_input_bias], ...
       {'Hybrid APF–NMPC', ...
        'Cost-Augmented APF–NMPC', ...
        'APF-Embedded Reference NMPC'}, ...
       'Location', 'northeast', 'FontSize', 10, 'Box', 'off');
   
% xline(start, '--', 'Start Avoidance', 'Color', [0.2 0.2 0.2]);
% xline(finish, '--', 'End Avoidance', 'Color', [0.2 0.2 0.2]);


set(gca, 'FontSize', 11);


%% --- Plot 2: Total Drone Speed Comparison ---
figure('Name', 'Total Drone Speed Comparison', 'Color', 'w');
hold on; 
box on; 
grid on;

% --- Plot Total Speed ---
h_speed_hybrid = plot(t_single, v_act_norm_single, 'Color', color1, 'LineWidth', 1.5);
h_speed_cost_only = plot(t_multi, v_act_norm_multi, 'Color', color2, 'LineWidth', 1.5);
h_speed_input_bias = plot(t_multi3, v_act_norm_multi3, 'Color', color3, 'LineWidth', 1.5);

ylabel('Drone Total Speed [m/s]');
xlabel('Time [s]');
title('Comparison of Drone Total Speed during Maneuver');

% Combine speed data to set consistent axis limits
all_speed = [v_act_norm_single, v_act_norm_multi, v_act_norm_multi3];
ylim([0, max(all_speed) * 1.2]);

% Legend for Speed Plot
legend([h_speed_hybrid, h_speed_cost_only, h_speed_input_bias], ...
       {'Hybrid APF–NMPC', 'Cost-Augmented APF–NMPC', 'APF-Embedded Reference NMPC'}, ...
       'Location', 'best');

% (Optional) Mark obstacle avoidance window
% xline(start, '--', 'Avoidance Start', 'Color', [0.5 0.5 0.5], 'LabelVerticalAlignment','top');
% xline(finish, '--', 'Avoidance End', 'Color', [0.5 0.5 0.5], 'LabelVerticalAlignment','top');

set(gca, 'FontSize', 11);


%% --- Publikasi-ready Plotting Kecepatan Drone --- %%
% Asumsi: results_single dan results_multi telah dimuat
% dan memiliki history_x, history_x_ref, dan dt

% Dapatkan data dan waktu untuk kedua metode
t_single = (0:size(results_single.history_x, 2)-1) * results_single.dt;
t_multi = (0:size(results_multi.history_x, 2)-1) * results_multi.dt;
t_multi3 = (0:size(results_multi3.history_x, 2)-1) * results_multi3.dt;

% Buat figure dengan ukuran yang disesuaikan
figure('Name','Drone Linear Velocity and Speed Comparison','Color','w', 'Position', [100 100 800 900]);

% Warna dan style
colors = {color1, color2, color3};
style_ref = '--'; % Garis putus-putus untuk referensi
style_single = '-'; % Garis solid untuk Single-NMPC
style_multi = '-'; % Garis titik-titik untuk Multi-NMPC
style_multi3 = '-'; % Garis titik-titik untuk Multi-NMPC

%% --- VX, VY, VZ Subplots ---
components = {'v_x','v_y','v_z'};
for i = 1:3
    subplot(4,1,i);
    hold on;
    
    % Plot NMPC reference (sama untuk kedua metode)
    h_ref = plot(t_single, results_single.history_x_ref(6+i,:), 'k', 'LineStyle', style_ref, 'LineWidth', 1.5);
    
    % --- Plot Actual Single-NMPC (Metode 1) ---
    h_single = plot(t_single, results_single.history_x(6+i,:), 'Color', colors{1}, 'LineStyle', style_single, 'LineWidth', 1.5);
    
    % --- Plot Actual Multi-NMPC (Metode 2) ---
    h_multi = plot(t_multi, results_multi.history_x(6+i,:), 'Color', colors{2}, 'LineStyle', style_multi, 'LineWidth', 1.5);
    
    % --- Plot Actual Metode Ketiga (Metode 3) ---
    h_multi3 = plot(t_multi3, results_multi3.history_x(6+i,:), 'Color', colors{3}, 'LineStyle', style_multi3, 'LineWidth', 1.5);
    
    grid on; box on;
    xlabel('Time [s]');
    ylabel([components{i} ' [m/s]']);
    
    legend([h_ref, h_single, h_multi, h_multi3], 'NMPC reference', 'Hybrid APF–NMPC', 'Cost-Augmented APF–NMPC', 'APF-Embedded Reference NMPC', 'Location', 'northeast');
    title(['Quadrotor Linear Velocity in ', strrep(components{i},'_','-')]);
    hold off;
end

%% --- Total Speed Subplot ---
v_ref_norm = sqrt(sum(results_single.history_x_ref(7:9,:).^2,1));
v_single_norm = sqrt(sum(results_single.history_x(7:9,:).^2,1));
v_multi_norm = sqrt(sum(results_multi.history_x(7:9,:).^2,1));
v_multi_norm3 = sqrt(sum(results_multi3.history_x(7:9,:).^2,1));
 
subplot(4,1,4);
hold on;
plot(t_single, v_ref_norm, 'k--', 'LineWidth', 1.5);
plot(t_single, v_single_norm, 'Color', color1, 'LineWidth', 1.5);
plot(t_multi, v_multi_norm, 'Color', color2, 'LineWidth', 1.5);
plot(t_multi3, v_multi_norm3, 'Color', color3, 'LineWidth', 1.5);

grid on; box on;
xlabel('Time [s]');
ylabel('Speed [m/s]');
legend('NMPC reference speed', 'Hybrid APF–NMPC', 'Cost-Augmented APF–NMPC', 'APF-Embedded Reference NMPC', 'Location', 'best');
title('Comparison of Drone Total Linear Speed');

hold off;



%% === Hitung Error untuk masing-masing metode ===
ref_traj = [x_ref; y_ref; z_ref];
act_single = results_single.history_x(1:3,:);
act_multi  = results_multi.history_x(1:3,:);
act_multi3 = results_multi3.history_x(1:3,:);

[RMSE_single, MAE_single, MAX_single] = computeTrackingError(ref_traj, act_single);
[RMSE_multi,  MAE_multi,  MAX_multi]  = computeTrackingError(ref_traj, act_multi);
[RMSE_multi3, MAE_multi3, MAX_multi3] = computeTrackingError(ref_traj, act_multi3);

%% === Hitung Violation untuk setiap metode ===
V_single = computeViolation(results_single, obs_center, obs_radius);
V_multi  = computeViolation(results_multi,  obs_center, obs_radius);
V_multi3 = computeViolation(results_multi3, obs_center, obs_radius);


%% === Buat Tabel Evaluasi ===
Method = {'Hybrid APF–NMPC'; 'Cost-Augmented APF–NMPC'; 'APF-Embedded Reference NMPC'};
RMSE = [RMSE_single; RMSE_multi; RMSE_multi3];
MAE  = [MAE_single; MAE_multi; MAE_multi3];
MAXE = [MAX_single; MAX_multi; MAX_multi3];
Violation = [V_single; V_multi; V_multi3];

T = table(Method, RMSE, MAE, MAXE, Violation);
disp('=== Performance Comparison ===');
disp(T);

%% === Visualisasi perbandingan Error & Violation ===
figure('Name','Error vs Violation Comparison','Color','w');
yyaxis left
bar(categorical(Method), RMSE, 0.4);
ylabel('RMSE [m]');
yyaxis right
plot(categorical(Method), Violation, 'ro-','LineWidth',2,'MarkerSize',8);
ylabel('Constraint Violation [m]');
title('Tracking Error and Safety Violation Comparison');
grid on;

%% === Fungsi perhitungan error tracking ===
function [RMSE, MAE, MAXE] = computeTrackingError(ref, actual)
    err = vecnorm(ref - actual, 2, 1);
    RMSE = sqrt(mean(err.^2));
    MAE  = mean(abs(err));
    MAXE = max(abs(err));
end

%% === Fungsi menghitung Constraint Violation (Obstacle) ===
function V = computeViolation(results, obs_center, obs_radius)
    d_safe = 0.5;   % batas aman (jarak pengaruh APF)
    num_steps = size(results.history_x,2);
    num_obs = size(obs_center,2);
    V = 0;
    for k = 1:num_steps
        p = results.history_x(1:2,k);
        min_dist = inf;
        for j = 1:num_obs
            c = obs_center(1:2,j);
            r = obs_radius(j);
            dist_surf = norm(p - c) - r; % jarak ke permukaan obstacle
            if dist_surf < min_dist
                min_dist = dist_surf;
            end
        end
        V = V + max(0, d_safe - min_dist);  % pelanggaran jika < d_safe
    end
    V = V / num_steps;
end