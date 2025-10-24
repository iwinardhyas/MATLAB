close all;

% Load hasil simulasi
load('adaptive.mat','results'); results_single = results;
load('fix.mat','results');  results_multi  = results;

% --- Time vector untuk state (N_sim+1) ---
t_single = (0:size(results_single.history_x,2)-1)*results_single.dt;
t_multi  = (0:size(results_multi.history_x,2)-1)*results_multi.dt;
x_ref = results_single.history_x_ref(1,:);
y_ref = results_single.history_x_ref(2,:);
z_ref = results_single.history_x_ref(3,:);

% --- Time vector untuk kontrol (N_sim) ---
t_u_single = (0:size(results_single.history_u,2)-1) * results_single.dt;
t_u_multi  = (0:size(results_multi.history_u,2)-1) * results_multi.dt;

color1 = [0.00,0.30,0.60];
color2 = [0.80,0.40,0.00];

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
    h_single = plot(t_single, results_single.history_x(idx,:), 'Color', color1, 'LineWidth', 1.5, 'DisplayName', 'Adaptive weight');
    h_multi = plot(t_multi, results_multi.history_x(idx,:), 'Color', color2, 'LineWidth', 1.5, 'DisplayName', 'Fixed weight');
    
    % Plot Reference Trajectory
    h_ref = plot(t_single, results_single.history_x_ref(idx,:), 'k--', 'LineWidth', 1.5, 'DisplayName', ref_labels{i});
    
    grid on;
    ylabel(pos_labels{i});
    title(pos_titles{i});
    
    % Sumbu X hanya pada plot terakhir
    xlabel('Time (s)');
%     set(gca, 'XTickLabel', []); % Sembunyikan label X pada plot atas
    
    % Tambahkan legenda hanya pada plot pertama
    legend([h_single, h_multi, h_ref], 'Location','best', 'NumColumns', 2);

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
    h_single = plot(t_single, results_single.history_x(idx,:), 'Color', color1, 'LineWidth', 1.5, 'DisplayName', 'Adaptive weight');
    h_multi = plot(t_multi, results_multi.history_x(idx,:), 'Color', color2, 'LineWidth', 1.5, 'DisplayName', 'Fixed weight');
    
    % Plot Reference Angle
    h_ref = plot(t_single, results_single.history_x_ref(idx,:), 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
    
    grid on;
    ylabel(angle_labels{i});
    title(angle_titles{i});
    
    % Sumbu X hanya pada plot terakhir
    xlabel('Time [s]');
%     set(gca, 'XTickLabel', []); % Sembunyikan label X pada plot atas
    
    % Tambahkan legenda hanya pada plot pertama
    legend([h_single, h_multi, h_ref], 'Location','best', 'NumColumns', 2);

    hold off;
end

%% === Compute Average Thrust ===
T_avg_single = mean(results_single.history_u, 1);
T_avg_multi  = mean(results_multi.history_u, 1);

%% === Compute Control Effort Metrics ===
J_u_single = sum(sum(results_single.history_u.^2, 1));
J_u_multi  = sum(sum(results_multi.history_u.^2, 1));

mean_u_single = mean(T_avg_single);
mean_u_multi  = mean(T_avg_multi);

max_u_single = max(T_avg_single);
max_u_multi  = max(T_avg_multi);

%% === PLOT 1: Average Thrust Comparison ===
figure('Name','Average Thrust Comparison','Color','w');
hold on; box on; grid on;

plot(t_u_single, T_avg_single, 'Color', color1, 'LineWidth', 1.8);
plot(t_u_multi,  T_avg_multi,  'Color', color2, 'LineWidth', 1.8);

xlabel('Time [s]');
ylabel('Average Thrust [N]');
title('Average Thrust Comparison between NMPC-APF Methods');

legend({'Adaptive weight', ...
        'Fixed weight'}, ...
        'Location','best');
set(gca,'FontSize',11);

ylim([min([T_avg_single,T_avg_multi])*0.9, ...
      max([T_avg_single,T_avg_multi])*1.1]);

%% === PLOT 2: Thrust per Motor (Optional, use one representative motor) ===
figure('Name','Thrust Command (Motor 1)','Color','w');
hold on; box on; grid on;

plot(t_u_single, results_single.history_u(1,:), 'Color', color1, 'LineWidth', 1.5);
plot(t_u_multi,  results_multi.history_u(1,:),  'Color', color2, 'LineWidth', 1.5);

xlabel('Time [s]');
ylabel('Thrust Motor 1 [N]');
title('Thrust Command Comparison (Representative Motor)');
legend({'Adaptive weight', 'Fixed weight'}, 'Location','best');
set(gca,'FontSize',11);

%% === TABLE: Control Effort Metrics ===
fprintf('\n=== CONTROL EFFORT METRICS ===\n');
fprintf('%-35s %-15s %-15s %-15s\n', 'Method', 'Σ||u||²', 'Mean Thrust [N]', 'Max Thrust [N]');
fprintf('---------------------------------------------------------------------------------\n');
fprintf('%-35s %-15.3f %-15.3f %-15.3f\n', 'Adaptive weight', J_u_single, mean_u_single, max_u_single);
fprintf('%-35s %-15.3f %-15.3f %-15.3f\n', 'Fixed weight',         J_u_multi,  mean_u_multi,  max_u_multi);

%% === (Optional) Convert metrics to table for plotting/export ===
ControlEffortTable = table( ...
    {'Adaptive weight'; 'Fixed weight'}, ...
    [J_u_single; J_u_multi], ...
    [mean_u_single; mean_u_multi], ...
    [max_u_single; max_u_multi], ...
    'VariableNames', {'Method','TotalEffort','MeanThrust','MaxThrust'});

disp(ControlEffortTable);

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
h_ref = plot3(x_ref_single, y_ref_single, z_ref_single, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference Path');

% Plot lintasan metode pertama (Single)
h_actual_single = plot3(x_actual_single, y_actual_single, z_actual_single, 'Color', color1, 'LineWidth', 1.5, 'DisplayName', 'Adaptive weight');

% Plot lintasan metode kedua (Multi)
h_actual_multi = plot3(x_actual_multi, y_actual_multi, z_actual_multi, 'Color', color2, 'LineWidth', 1.5, 'DisplayName', 'Fixed weight');

%% --- Tandai Start & Finish ---

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

xlabel('X [m]');
ylabel('Y [m]');
title('Drone Trajectory (Top View) Comparison');
axis equal;
grid on;
box on;

%% --- Legend yang lebih informatif ---
legend([h_actual_single, h_actual_multi,h_ref], 'Location', 'northeast');

h_arrow_single = plot3(nan, nan, nan, 'Color', color1, 'LineWidth', 1.5, 'DisplayName', 'Adaptive weight');
h_arrow_multi = plot3(nan, nan, nan, 'Color', color2, 'LineWidth', 1.5, 'DisplayName', 'Fixed weight');

% --- Time vector untuk state (N_sim+1) ---
t_single = (0:size(results_single.history_x,2)-1)*results_single.dt;
t_multi  = (0:size(results_multi.history_x,2)-1)*results_multi.dt;
x_ref = results_single.history_x_ref(1,:);
y_ref = results_single.history_x_ref(2,:);
z_ref = results_single.history_x_ref(3,:);

% --- Time vector untuk kontrol (N_sim) ---
t_u_single = (0:size(results_single.history_u,2)-1) * results_single.dt;
t_u_multi  = (0:size(results_multi.history_u,2)-1) * results_multi.dt;

color1 = [0.00,0.30,0.60];
color2 = [0.80,0.40,0.00];

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
    h_single = plot(t_single, results_single.history_x(idx,:), 'Color', color1, 'LineWidth', 1.5, 'DisplayName', 'Adaptive weight');
    h_multi = plot(t_multi, results_multi.history_x(idx,:), 'Color', color2, 'LineWidth', 1.5, 'DisplayName', 'Fixed weight');
    
    % Plot Reference Trajectory
    h_ref = plot(t_single, results_single.history_x_ref(idx,:), 'k--', 'LineWidth', 1.5, 'DisplayName', ref_labels{i});
    
    grid on;
    ylabel(pos_labels{i});
    title(pos_titles{i});
    
    % Sumbu X hanya pada plot terakhir
    xlabel('Time (s)');
%     set(gca, 'XTickLabel', []); % Sembunyikan label X pada plot atas
    
    % Tambahkan legenda hanya pada plot pertama
    legend([h_single, h_multi, h_ref], 'Location','best', 'NumColumns', 2);

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
    h_single = plot(t_single, results_single.history_x(idx,:), 'Color', color1, 'LineWidth', 1.5, 'DisplayName', 'Adaptive weight');
    h_multi = plot(t_multi, results_multi.history_x(idx,:), 'Color', color2, 'LineWidth', 1.5, 'DisplayName', 'Fixed weight');
    
    % Plot Reference Angle
    h_ref = plot(t_single, results_single.history_x_ref(idx,:), 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
    
    grid on;
    ylabel(angle_labels{i});
    title(angle_titles{i});
    
    % Sumbu X hanya pada plot terakhir
    xlabel('Time [s]');
%     set(gca, 'XTickLabel', []); % Sembunyikan label X pada plot atas
    
    % Tambahkan legenda hanya pada plot pertama
    legend([h_single, h_multi, h_ref], 'Location','best', 'NumColumns', 2);

    hold off;
end

%% === Compute Average Thrust ===
T_avg_single = mean(results_single.history_u, 1);
T_avg_multi  = mean(results_multi.history_u, 1);

%% === Compute Control Effort Metrics ===
J_u_single = sum(sum(results_single.history_u.^2, 1));
J_u_multi  = sum(sum(results_multi.history_u.^2, 1));

mean_u_single = mean(T_avg_single);
mean_u_multi  = mean(T_avg_multi);

max_u_single = max(T_avg_single);
max_u_multi  = max(T_avg_multi);