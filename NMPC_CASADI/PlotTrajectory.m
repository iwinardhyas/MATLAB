% Definisikan vektor waktu yang sesuai dengan data simulasi
time_vector = 0:dt:T_sim; % Ini akan memiliki N_sim + 1 titik

% Plot the states.
fig = figure(1);       % Fokus pada figure 1
set(fig, 'Name', 'Drone Position Plot', 'NumberTitle', 'off'); % Beri nama pada window
clf(fig); % Bersihkan figure sebelum plotting baru

% Plot Posisi X
subplot(3,1,1); % Menggunakan 3 baris, 1 kolom, plot pertama
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
subplot(3,1,2); % Plot kedua
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
subplot(3,1,3); % Plot ketiga
hold on;
plot(time_vector, history_x(3,:), 'b-', 'LineWidth', 1.5); % Actual Z (baris 3)
plot(time_vector, history_x_ref(3,:), 'r--', 'LineWidth', 1.5); % Reference Z (baris 3)
grid on;
xlabel('Time (s)');
ylabel('Position Z (m)');
legend('Actual Z', 'Reference Z', 'Location','best');
title('Quadrotor Z Position');
hold off;

% Opsional: Plot 3D Trajectory
figure(2);
hold on;
plot3(history_x(1,:), history_x(2,:), history_x(3,:), 'b-', 'LineWidth', 2); % Actual 3D
plot3(history_x_ref(1,:), history_x_ref(2,:), history_x_ref(3,:), 'r--', 'LineWidth', 1.5); % Reference 3D
grid on;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
legend('Actual Trajectory', 'Reference Trajectory', 'Location', 'best');
title('Quadrotor 3D Trajectory');
view(3); % Set view to 3D
axis equal; % Penting untuk menjaga proporsi
hold off;