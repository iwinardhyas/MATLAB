function y = quad_measurement_model(x)
    % [pos, ori, vel]
    y = [x(1:3); x(4:6); x(7:9)];
end
