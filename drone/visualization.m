function visualization(sim_data, r)
    % VISUALIZATION_3D - Visualisasi hasil simulasi dalam 3D

    % Plot lintasan drone
    figure;
    plot3(sim_data.Output(:, 1), sim_data.Output(:, 2), sim_data.Output(:, 3), ...
        'b', 'LineWidth', 1.5, 'DisplayName', 'Lintasan Drone');
    hold on;

    % Plot target (setpoint) dalam 3D
    plot3(r(1), r(2), r(3), 'ro', 'MarkerSize', 8, 'DisplayName', 'Target');

    % Tambahkan grid dan properti plot
    grid on;
    xlabel('Posisi X (meter)');
    ylabel('Posisi Y (meter)');
    zlabel('Posisi Z (meter)');
    title('Visualisasi Pergerakan Drone dalam 3D');
    legend('show');
    axis equal;

    % Tambahkan anotasi lintasan
    for k = 1:10:length(sim_data.Time)
        text(sim_data.Output(k, 1), sim_data.Output(k, 2), sim_data.Output(k, 3), ...
            sprintf('t=%.1fs', sim_data.Time(k)), 'FontSize', 8);
    end

    hold off;
end

