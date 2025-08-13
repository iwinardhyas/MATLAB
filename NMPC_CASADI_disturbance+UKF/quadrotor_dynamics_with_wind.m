function xdot = quadrotor_dynamics_with_wind_ukf(x, u, g_val, l_val, Ixx_val, Iyy_val, Izz_val)
    % x is 15x1: [px;py;pz; phi;theta;psi; vx;vy;vz; p;q;r; Wx;Wy;Wz]
    m = 0.5; C_d = 1.0;

    % unpack
    phi = x(4); theta = x(5); psi = x(6);
    vx = x(7); vy = x(8); vz = x(9);
    p = x(10); q = x(11); r = x(12);
    Wx = x(13); Wy = x(14); Wz = x(15);

    % Rotation
    R_b_i = rotz(psi) * roty(theta) * rotx(phi);

    % thrust
    F_total = sum(u);
    thrust_body = [0; 0; F_total];
    thrust_inertial = R_b_i * thrust_body;

    % relative velocity & drag (in inertial)
    v_rel = [vx; vy; vz] - [Wx; Wy; Wz];
    drag_force = -C_d * v_rel;

    % translational accelerations
    ax = (thrust_inertial(1) + drag_force(1)) / m;
    ay = (thrust_inertial(2) + drag_force(2)) / m;
    az = (thrust_inertial(3) + drag_force(3)) / m - g_val;

    % rotational torques -> angular accelerations
    tau_phi = l_val * (u(2) - u(4));
    tau_theta = l_val * (u(3) - u(1));
    tau_psi = 0.005 * (u(1) - u(2) + u(3) - u(4));

    p_dot = (tau_phi + (Iyy_val - Izz_val) * q * r) / Ixx_val;
    q_dot = (tau_theta + (Izz_val - Ixx_val) * p * r) / Iyy_val;
    r_dot = (tau_psi + (Ixx_val - Iyy_val) * p * q) / Izz_val;

    % kinematics
    phi_dot = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
    theta_dot = q*cos(phi) - r*sin(phi);
    psi_dot = q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta);

    % wind dynamics: random-walk / slow-varying (tune via Q)
    Wx_dot = 0;
    Wy_dot = 0;
    Wz_dot = 0;

    xdot = [vx; vy; vz;
            phi_dot; theta_dot; psi_dot;
            ax; ay; az;
            p_dot; q_dot; r_dot;
            Wx_dot; Wy_dot; Wz_dot];
end


function R = rotation_matrix(phi, theta, psi)
    Rz = [cos(psi), -sin(psi), 0;
          sin(psi),  cos(psi), 0;
                0,        0, 1];
    Ry = [cos(theta), 0, sin(theta);
                   0, 1,         0;
         -sin(theta), 0, cos(theta)];
    Rx = [1,        0,         0;
          0, cos(phi), -sin(phi);
          0, sin(phi),  cos(phi)];
    R = Rz * Ry * Rx;
end
