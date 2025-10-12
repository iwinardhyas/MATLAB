function goal = adaptive_reference(current_state, goal, obstacles, threshold_speed)
    % Adaptive re-targeting for NMPC when drone becomes unstable
    % current_state: [x; y; z; vx; vy; vz; ...]
    % goal: [x_goal; y_goal; z_goal]
    % obstacles: Nx3 matrix [x_obs, y_obs, z_obs]
    % threshold_speed: trigger when drone too fast (unstable)

    persistent prev_goal re_target_timer
    if isempty(prev_goal)
        prev_goal = goal;
        re_target_timer = 0;
    end

    % --- Extract velocity and position ---
    vx = current_state(4);
    vy = current_state(5);
    vz = current_state(6);
    v_norm = sqrt(vx^2 + vy^2 + vz^2);

    pos = current_state(1:3);

    % --- Check if drone exceeds threshold speed ---
    if v_norm > threshold_speed
        re_target_timer = re_target_timer + 1;

        % If unstable for more than 5 consecutive loops → retarget
        if re_target_timer > 5
            % Find nearest obstacle
            dists = vecnorm(obstacles - pos', 2, 2);
            [~, idx_min] = min(dists);

            obs_near = obstacles(idx_min, :);
            dir_away = (pos' - obs_near) / norm(pos' - obs_near + 1e-6);

            % Shift goal slightly away from obstacle
            safe_offset = 3; % meters away
            goal = pos' + dir_away * safe_offset;

            fprintf('[AdaptiveRef] Retargeted goal to avoid instability.\n');
            re_target_timer = 0;
        end
    else
        re_target_timer = max(0, re_target_timer - 1);
    end

    prev_goal = goal;
end
