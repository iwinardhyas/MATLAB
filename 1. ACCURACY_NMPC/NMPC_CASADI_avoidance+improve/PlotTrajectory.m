% Definisikan vektor waktu yang sesuai dengan data simulasi
time_vector = 0:dt:T_sim; % Ini akan memiliki N_sim + 1 titik

% Plot the states.
fig = figure(1);       % Fokus pada figure 1
set(fig, 'Name', 'Drone Position Plot', 'NumberTitle', 'off'); % Beri nama pada window
clf(fig); % Bersihkan figure sebelum plotting baru

% Plot Posisi X
subplot(3,3,1); % Menggunakan 3 baris, 1 kolom, plot pertama
hold on;
plot(time_vector, history_x(1,:), 'b-', 'LineWidth', 1.5); % Actual X (baris 1)
plot(time_vector, history_x_ref(1,:), 'r--', 'LineWidth', 1.5); % Reference X (baris 1)
grid on;
xlabel('Time (s)');
ylabel('Position X (m)');
legend('Actual X', 'Reference X', 'Location','best');
title('Quadrotor X Position');
hold off;

% Plot Posisi Y
subplot(3,3,2); % Plot kedua
hold on;
plot(time_vector, history_x(2,:), 'b-', 'LineWidth', 1.5); % Actual Y (baris 2)
plot(time_vector, history_x_ref(2,:), 'r--', 'LineWidth', 1.5); % Reference Y (baris 2)
grid on;
xlabel('Time (s)');
ylabel('Position Y (m)');
legend('Actual Y', 'Reference Y', 'Location','best');
title('Quadrotor Y Position');
hold off;

% Plot Posisi Z
subplot(3,3,3); % Plot ketiga
hold on;
plot(time_vector, history_x(3,:), 'b-', 'LineWidth', 1.5); % Actual Z (baris 3)
plot(time_vector, history_x_ref(3,:), 'r--', 'LineWidth', 1.5); % Reference Z (baris 3)
grid on;
xlabel('Time (s)');
ylabel('Position Z (m)');
legend('Actual Z', 'Reference Z', 'Location','best');
title('Quadrotor Z Position');
hold off;
% 
% Phi angle
subplot(3,3,4); hold on;
plot(time_vector, history_x(4,:), 'b-', 'LineWidth', 1.5); 
plot(time_vector, history_x_ref(4,:), 'r--', 'LineWidth', 1.5); 
grid on;
xlabel('Time [s]');
ylabel('Phi angle [rad]');
legend('Actual', 'Reference', 'Location','best');
title('Quadrotor Roll (Phi)'); 
hold off;

% Theta angle
subplot(3,3,5); hold on;
plot(time_vector, history_x(5,:), 'b-', 'LineWidth', 1.5); 
plot(time_vector, history_x_ref(5,:), 'r--', 'LineWidth', 1.5); 
grid on;
xlabel('Time [s]');
ylabel('Theta angle [rad]');
legend('Actual', 'Reference', 'Location','best');
title('Quadrotor Pitch (Theta)'); 
hold off;

% Psi angle
subplot(3,3,6); hold on;
plot(time_vector, history_x(6,:), 'b-', 'LineWidth', 1.5); 
plot(time_vector, history_x_ref(6,:), 'r--', 'LineWidth', 1.5); 
grid on;
xlabel('Time [s]');
ylabel('Psi angle [rad]');
legend('Actual', 'Reference', 'Location','best');
title('Quadrotor Yaw (Psi)'); 
hold off;

% 
% Plot thrust
t = 0:dt:T_sim-dt;   % sumbu waktu
figure;
grid on;
plot(t, history_u');
xlabel('Time [s]');
ylabel('Control Input');
legend('u1','u2','u3','u4');   % sesuai nu
title('History of Control Inputs');
hold off;

% du = diff(history_u,1,2);                % ukuran = nu x (N_sim-1)
% t_du = 0:dt:(size(du,2)-1)*dt;           % panjang sama persis
% figure;
% plot(t_du, du');
% xlabel('Time [s]');
% ylabel('\Delta u');
% legend('du1','du2','du3','du4');
% title('Control Smoothness (Delta U) vs Time');
% grid on;




%% Plotting Hasil Lintasan

% Pastikan Anda memiliki data history dari simulasi Anda:
% history_x_ref: Matriks (12 x N_sim_total+1) yang menyimpan state referensi di setiap waktu
% history_x_actual: Matriks (12 x N_sim_total+1) yang menyimpan state aktual di setiap waktu

% Jika T_sim Anda adalah sebuah vektor waktu yang sesuai dengan kolom history_x_ref/actual
% Misalnya: T_sim = 0:dt_ukf:N_sim_total*dt_ukf;

% Ekstrak posisi (x,y,z) dari data history
x_ref = history_x_ref(1, :);
y_ref = history_x_ref(2, :);
z_ref = history_x_ref(3, :);

x_actual = history_x(1, :);
y_actual = history_x(2, :);
z_actual = history_x(3, :);


figure; hold on; grid on; box on;

%% --- Plot Lintasan Reference dan Aktual ---
h_ref   = plot3(x_ref, y_ref, z_ref, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Reference Path');
h_actual= plot3(x_actual, y_actual, z_actual, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Drone Path');

%% --- Tandai Start & Finish ---
h_start_ref = plot3(x_ref(1), y_ref(1), z_ref(1), 'bo','MarkerFaceColor','b','MarkerSize',8,'DisplayName','Start Reference');
h_start_act = plot3(x_actual(1), y_actual(1), z_actual(1), 'ro','MarkerFaceColor','r','MarkerSize',8,'DisplayName','Start Actual');
h_finish_ref= plot3(x_ref(end), y_ref(end), z_ref(end), 'bx','LineWidth',2,'MarkerSize',10,'DisplayName','Finish Reference');
h_finish_act= plot3(x_actual(end), y_actual(end), z_actual(end), 'rx','LineWidth',2,'MarkerSize',10,'DisplayName','Finish Actual');

%% --- Drone Heading (quiver3) ---
N = length(x_actual);
step_arrow = max(floor(N/20),1); % sekitar 20 panah
scale_arrow = 0.5;
for k = 1:step_arrow:N-1
    quiver3(x_actual(k), y_actual(k), z_actual(k), ...
            x_actual(k+1)-x_actual(k), ...
            y_actual(k+1)-y_actual(k), ...
            z_actual(k+1)-z_actual(k), ...
            scale_arrow,'m','LineWidth',1.5,'MaxHeadSize',2);
end
h_arrow = plot3(nan,nan,nan,'m-','LineWidth',1.5,'DisplayName','Drone Heading');

%% --- Plot Obstacles (Silinder Hijau Tua) ---
num_obs = size(obs_center,2);
obstacle_height = 2;%8;
safety_scale = 1.2; % safety margin

for j = 1:num_obs
    c = obs_center(:,j);
    r = obs_radius(j);

    [x_cyl, y_cyl, z_cyl] = cylinder(r,50); % 50 titik lingkaran
    z_cyl = z_cyl * obstacle_height;
    x_cyl = x_cyl + c(1);
    y_cyl = y_cyl + c(2);

    % Obstacle utama warna hijau tua
    surf(x_cyl, y_cyl, z_cyl, 'FaceAlpha',0.5, 'FaceColor',[0 0.5 0], 'EdgeColor','none');

    % Safety margin (transparan, merah tipis)
    [x_safe, y_safe, z_safe] = cylinder(r*safety_scale,50);
    z_safe = z_safe*obstacle_height;
    x_safe = x_safe + c(1);
    y_safe = y_safe + c(2);
    surf(x_safe, y_safe, z_safe, 'FaceAlpha',0.1,'FaceColor','r','EdgeColor','none','HandleVisibility','off');
end
h_obs = plot3(nan,nan,nan,'s','MarkerFaceColor','k','MarkerEdgeColor','k','DisplayName','Obstacles');

%% --- Axis, Labels, View ---
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Drone Trajectory with NMPC + Obstacles','FontSize',12);
view(45,25);

% Batasi sumbu Y mulai dari 0
ylim([0, max([y_ref y_actual])+1]);
xlim([min([x_ref x_actual])-1, max([x_ref x_actual])+1]);
zlim([0, max([z_ref z_actual])+1]);

axis equal; grid on;

%% --- Legend (informatif) ---
legend([h_actual h_ref h_obs h_arrow h_start_act h_finish_act h_start_ref h_finish_ref], 'Location','bestoutside');


figure; hold on; grid on;

%% --- Plot Drone Trajectory dan Reference (2D) ---
plot(history_x(1,:), history_x(2,:), 'b-', 'LineWidth', 1.5); % jalur drone
plot(history_x_ref(1,:), history_x_ref(2,:), 'r--', 'LineWidth', 1.5); % jalur referensi

%% --- Obstacles (lingkaran 2D sesuai obs_radius_val) ---
theta = linspace(0, 2*pi, 100); % lingkaran halus
colors = lines(size(obs_center,2));
obs_radius = obs_radius_val * ones(1, num_obs);

for j = 1:size(obs_center,2)
    x_circle = obs_center(1,j) + obs_radius(j)*cos(theta);
    y_circle = obs_center(2,j) + obs_radius(j)*sin(theta);
    fill(x_circle, y_circle, colors(j,:), 'FaceAlpha',0.3, 'EdgeColor','k');
end

plot(history_x(1,1), history_x(2,1), '^', 'MarkerSize',10,'MarkerFaceColor','g','DisplayName','Start');
plot(history_x(1,end), history_x(2,end), 'x', 'MarkerSize',10,'MarkerFaceColor','k','DisplayName','Finish');

%% --- Drone Heading (quiver) ---
N = length(history_x(1,:));
step_arrow = max(floor(N/20),1); % tampilkan sekitar 20 panah
scale_arrow = 1;
for k = 1:step_arrow:N-1
    quiver(history_x(1,k), history_x(2,k), ...
           history_x(1,k+1)-history_x(1,k), history_x(2,k+1)-history_x(2,k), ...
           scale_arrow, 'm', 'LineWidth',1.5, 'MaxHeadSize',3);
end

%% --- Axis, Labels, Title ---
xlabel('X [m]'); ylabel('Y [m]');
title('Drone Trajectory (Top View) NMPC+APF');

%% --- Legend yang lebih informatif ---
h_drone = plot(nan,nan,'b-','LineWidth',1.5,'DisplayName','Drone Path');
h_ref   = plot(nan,nan,'r--','LineWidth',1.5,'DisplayName','Reference Path');
h_obs   = plot(nan,nan,'o','MarkerFaceColor','k','MarkerEdgeColor','k','DisplayName','Obstacles'); % lingkaran
h_arrow = plot(nan,nan,'m-','LineWidth',1.5,'DisplayName','Drone Heading');
h_start = plot(nan,nan,'^','MarkerFaceColor','g','MarkerEdgeColor','g','DisplayName','Start');  % segitiga
h_finish= plot(nan,nan,'x','MarkerEdgeColor','k','LineWidth',2,'DisplayName','Finish');          % x

legend([h_drone h_ref h_obs h_arrow h_start h_finish],'Location','bestoutside');
axis equal; grid on; box on;



% d_min_log = zeros(1,N_sim);
% for i=1:N_sim
%     p = history_x(1:3,i);
%     d_all = vecnorm(p - obs_center,2,1) - obs_radius;
%     d_min_log(i) = min(d_all);
% end
% 
% figure; plot((0:N_sim-1)*dt, d_min_log, 'LineWidth',1.5);
% xlabel('Time [s]'); ylabel('Min distance to obstacle [m]');
% title('Jarak Minimum Drone ke Obstacle');
% grid on;

%% --- Publikasi-ready: Drone Minimum Distance & Speed --- %%
% Pastikan variabel berikut sudah ada:
% history_x      : state drone [12 x N_sim] (posisi, sudut, kecepatan)
% obs_center     : [3 x num_obs] pusat obstacle
% obs_radius     : [1 x num_obs] jari-jari obstacle
% dt             : timestep simulasi
% N_sim          : jumlah timestep

%% --- Publikasi-ready: Drone Minimum Distance & Speed --- %%
% Pastikan variabel berikut sudah ada:
% history_x      : state drone [12 x N_actual] (posisi, sudut, kecepatan)
% obs_center     : [3 x num_obs] pusat obstacle
% obs_radius     : [1 x num_obs] jari-jari obstacle
% dt             : timestep simulasi

% --- Jumlah timestep sesuai history_x ---
N_actual = size(history_x,2);
time = (0:N_actual-1)*dt;

%% --- Hitung jarak minimum ke obstacle ---
d_min_log = zeros(1,N_actual);
for i = 1:N_actual
    p = history_x(1:3,i);  % posisi drone (x,y,z)
    d_all = vecnorm(p - obs_center,2,1) - obs_radius; % jarak ke semua obstacle
    d_min_log(i) = min(d_all);  % jarak minimum
end

%% --- Hitung total speed drone ---
v_act_norm = sqrt(sum(history_x(7:9,1:N_actual).^2,1));  % magnitude linear velocity

%% --- Plot Publikasi-ready ---
figure('Name','Drone Distance to Obstacles and Speed','Color','w'); hold on; box on; grid on;

d_safe = 1.0; % Threshold aman [m]

% --- Plot jarak minimum (sumbu kiri) ---
yyaxis left
h_dist = plot(time, d_min_log, 'b-', 'LineWidth',1.5);
ylabel('Minimum Distance to Obstacles [m]');
ylim([0 max(d_min_log)*1.2]); % mulai dari 0 agar grafik padat
yline(d_safe,'r--','LineWidth',1.5,'Label','Safe Distance','LabelHorizontalAlignment','right');

% Highlight titik terlalu dekat obstacle
idx_warning = find(d_min_log < d_safe);
scatter(time(idx_warning), d_min_log(idx_warning), 50, 'ro','filled');

% --- Plot total speed (sumbu kanan) ---
yyaxis right
h_speed = plot(time, v_act_norm, 'k-','LineWidth',1.5);
ylabel('Drone Total Speed [m/s]');
ylim([0 max(v_act_norm)*1.2]);

% --- Labels & Title ---
xlabel('Time [s]');
title('Drone Minimum Distance to Obstacles and Total Speed','FontSize',12);

% --- Legend ---
legend([h_dist, h_speed], ...
       {'Minimum Distance d_{min}','Total Speed'}, ...
       'Location','best');

set(gca,'FontSize',11);
grid on; box on;



% Tambahkan baris ketiga (z) dengan nilai nol untuk plot 3D
% z_ref_history = zeros(1, size(v_ref_history, 2));
% 
% figure; plot3(v_ref_history(1,:), v_ref_history(2,:),z_ref_history);
% xlabel('v_x'); ylabel('v_y');
% title('Arah koreksi dari APF');
% grid on;

%% --- Publikasi-ready Plotting Kecepatan Drone --- %%
figure('Name','Drone Linear Velocity and Speed Comparison','Color','w');

% Warna dan style
colors = {'b','r','g'};
style_ref = '--';  % NMPC reference
style_act = '-';   % Actual velocity (after APF)

%% --- VX, VY, VZ Subplots ---
components = {'v_x','v_y','v_z'};
for i = 1:3
    subplot(4,1,i); hold on;
    
    % Plot NMPC reference
    plot(time_vector, history_x_ref(6+i,:), [colors{i} style_ref], 'LineWidth',1.5);
    
    % Plot Actual + APF
    plot(time_vector, history_x(6+i,:), [colors{i} style_act], 'LineWidth',1.5);
    
    grid on; box on;
    xlabel('Time [s]');
    ylabel([components{i} ' [m/s]']);
    
    legend('NMPC reference trajectory','Actual velocity (after APF)','Location','best');
    title(['Quadrotor Linear Velocity in ', strrep(components{i},'_','-')]);
end

%% --- Total Speed Subplot ---
v_ref_norm = sqrt(sum(history_x_ref(7:9,:).^2,1));  % magnitude NMPC
v_act_norm = sqrt(sum(history_x(7:9,:).^2,1));      % magnitude NMPC + APF

subplot(4,1,4); hold on;
plot(time_vector, v_ref_norm, 'k--','LineWidth',1.5);
plot(time_vector, v_act_norm, 'k-','LineWidth',1.5);

grid on; box on;
xlabel('Time [s]');
ylabel('Speed [m/s]');
legend('NMPC reference trajectory','Actual speed (after APF)','Location','best');
title('Comparison of Drone Total Linear Speed: NMPC Reference vs Actual (with APF)');

%% --- Overall figure adjustments ---
set(gcf,'Position',[100 100 800 900]);  % ukuran figure

% phi   = history_x(4,:)   % roll
% theta = history_x(5,:)   % pitch
% psi   = history_x(6,:)   % yaw

figure('Name','Drone Heading Angle','Color','w'); hold on; grid on; box on;

% NMPC reference
plot(time, history_x_ref(6,:), 'b--','LineWidth',1.5);

% Actual (dengan APF)
plot(time, history_x(6,:), 'r-','LineWidth',1.5);

xlabel('Time [s]');
ylabel('Heading Angle \psi [rad]');
title('Drone Heading (Yaw) Angle vs Time');
legend('NMPC reference','Actual (with APF)','Location','best');

psi_deg = rad2deg(history_x(6,:));
psi_ref_deg = rad2deg(history_x_ref(6,:));

figure('Name','Drone Heading on XY plane','Color','w'); hold on; grid on; axis equal;

quiver(history_x(1,:), history_x(2,:), cos(history_x(6,:)), sin(history_x(6,:)), 0.5, 'r');
plot(history_x(1,:), history_x(2,:), 'b-', 'LineWidth',1.5);

xlabel('X [m]');
ylabel('Y [m]');
title('Drone XY Trajectory with Heading Arrows');
legend('Heading','Trajectory','Location','best');
