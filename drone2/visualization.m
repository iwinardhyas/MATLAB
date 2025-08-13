function visualization(sim_data, r)
    % VISUALIZATION_3D - Visualisasi hasil simulasi dalam 3D

    % Plot lintasan drone
    figure;
    plot3(sim_data.Output(:, 1), sim_data.Output(:, 2), sim_data.Output(:, 3), ...
        'b', 'LineWidth', 1.5, 'DisplayName', 'Lintasan Drone');
    hold on;

    % Plot target (setpoint) dalam 3D
    plot3(r(1), r(2), r(3), 'ro', 'MarkerSize', 15, 'DisplayName', 'Target');

    % Tambahkan grid dan properti plot
    grid on;
    xlabel('Posisi X (meter)');
    ylabel('Posisi Y (meter)');
    zlabel('Posisi Z (meter)');
    title('Visualisasi Pergerakan Drone dalam 3D');
    legend('show');
    axis equal;

    % Tambahkan anotasi lintasan
%     for k = 1:10:length(sim_data.Time)
%         text(sim_data.Output(k, 1), sim_data.Output(k, 2), sim_data.Output(k, 3), ...
%             sprintf('t=%.1fs', sim_data.Time(k)), 'FontSize', 8);
%     end
    text(sim_data.Output(1, 1), sim_data.Output(1, 2), sim_data.Output(1, 3), ...
    sprintf('Awal:\nPos: [%.2f, %.2f, %.2f]\nKecepatan: [%.2f, %.2f, %.2f]\nWaktu: %.2fs', ...
    sim_data.Output(1, 1), sim_data.Output(1, 2), sim_data.Output(1, 3), ...
    sim_data.Output(1, 4), sim_data.Output(1, 5), sim_data.Output(1, 6), sim_data.Time(1)), ...
    'Color', 'green', 'FontSize', 8, 'FontWeight', 'bold');

    idx_end = size(sim_data.Output, 1); % Indeks akhir
    text(sim_data.Output(idx_end, 1), sim_data.Output(idx_end, 2), sim_data.Output(idx_end, 3), ...
    sprintf('Akhir:\nPos: [%.2f, %.2f, %.2f]\nKecepatan: [%.2f, %.2f, %.2f]\nWaktu: %.2fs', ...
    sim_data.Output(idx_end, 1), sim_data.Output(idx_end, 2), sim_data.Output(idx_end, 3), ...
    sim_data.Output(idx_end, 4), sim_data.Output(idx_end, 5), sim_data.Output(idx_end, 6), sim_data.Time(idx_end)), ...
    'Color', 'red', 'FontSize', 8, 'FontWeight', 'bold');
    hold off;
    
    % Grafik posisi dengan target
    figure;
    plot(sim_data.Time, sim_data.Output(:, 1), 'b', 'DisplayName', 'X Position');
    hold on;
    plot(sim_data.Time, repmat(r(1), size(sim_data.Time)), '--r', 'DisplayName', 'Target X');
    legend show;
    xlabel('Time (s)');
    ylabel('Position');
    title('X Position vs Target');

    % Error plot
    error = sim_data.Output(:, 1) - r(1);
    figure;
    plot(sim_data.Time, error, 'k', 'DisplayName', 'Error X');
    xlabel('Time (s)');
    ylabel('Error');
    title('Position Error');
end

