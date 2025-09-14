%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DEMO: NMPC + Dijkstra + Trajectory Smoothing for Quadrotor (Ready-to-Run)
% -------------------------------------------------------------------------
% What this script does
% 1) Builds a 2D occupancy grid with obstacles.
% 2) Finds shortest path (x,y) using Dijkstra.
% 3) Smooths the path (shortcut + PCHIP) and time-parameterizes it.
% 4) Generates a continuous reference trajectory (x,y,z,yaw and derivatives).
% 5) Solves NMPC (CasADi) to track the trajectory with a 12D quadrotor model
%    and inputs [T, tau_x, tau_y, tau_z]. Discretization: RK4.
% 6) Plots results.
%
% Notes
% - This is a research-grade scaffold with standard formulations and 
%   conservative defaults. You can swap the 2D planner with 3D and keep 
%   the rest intact.
% - Requires: MATLAB R2021a+ and CasADi (e.g., casadi-3.6.5) on path.
% - No special toolboxes are required (smoothing uses PCHIP).
% - You can plug this into your existing function
%       xdesired = QuadrotorReferenceTrajectory1(t, mass)
%   by replacing the reference source with the generated trajectory here.
% - For clarity, everything is in one file; helper functions are at the end.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% clear; clc; close all;

function [ref_fun, ref_data] = QuadrotorReferenceNMPC(start_xy, goal_xy, mass, r_drone, r_safe)
% QuadrotorReferenceNMPC
% Menghasilkan fungsi reference trajectory untuk NMPC quadrotor
% Input:
%   start_xy : [x, y] posisi awal (m)
%   goal_xy  : [x, y] posisi tujuan (m)
%   mass     : massa quadrotor (kg)
%
% Output:
%   ref_fun  : handle function @(t) -> [x,y,z,yaw,...] di waktu t
%   ref_data : struct berisi seluruh array reference (t, x, y, ...)

    %% -------------------- ENVIRONMENT -------------------- %%
    map_size   = [50, 35];   % m
    resolution = 0.5;        % m per cell

    obstacles = [
        % --- Halangan besar di tengah ---
        18, 10, 8, 12;     % obstacle kotak besar di tengah
        28, 15, 6, 10;     % obstacle kotak sedang

        % --- Halangan di dekat pinggir bawah ---
        5,  2,  8, 6;      % obstacle bawah kiri
        40, 1,  8, 7;      % obstacle bawah kanan

        % --- Halangan di dekat pinggir atas ---
        2,  28, 6, 5;      % obstacle atas kiri
        30, 27, 12, 6;     % obstacle atas kanan

        % --- Halangan vertikal memotong lintasan ---
        12, 5,  3, 18;     % dinding vertikal kiri
        22, 8,  3, 15;     % dinding vertikal tengah
        38, 10,  3, 20;     % dinding vertikal kanan

        % --- Halangan kecil random (gangguan tambahan) ---
        8,  18, 3, 3;      % obstacle kecil
        16, 25, 4, 3;      % obstacle kecil
        25, 5,  3, 3;      % obstacle kecil
        35, 18, 4, 4;      % obstacle kecil
        45, 10, 3, 3;      % obstacle kecil dekat kanan
        
        0, 1, 50, 1;     % bawah
        0, 34, 50, 1;    % atas
        0, 1, 1, 35;     % kiri
        49, 0, 1, 35;     % kanan
    ];

    % Reference altitude & yaw
    z_ref_const   = 2.0;   % m
    psi_ref_const = 0.0;   % rad

    % Time law params
    v_max    = 3.0;   % m/s
    a_max    = 2.0;   % m/s^2
    min_dt   = 0.05;  % s

    rng(5); % reproducible

    %% ----------------------- OCCUPANCY GRID ------------------------ %%
    [G, x_coords, y_coords] = buildOccupancyGrid(map_size, resolution, obstacles, r_drone, r_safe);
    start_idx = world2grid(start_xy, x_coords, y_coords);
    goal_idx  = world2grid(goal_xy,  x_coords, y_coords);


    %% ---------------------------- DIJKSTRA -------------------------- %%
    path_idx = dijkstraGrid(G, start_idx, goal_idx);
    if isempty(path_idx)
        error('No path found by Dijkstra. Adjust obstacles or start/goal.');
    end
    path_xy = grid2world(path_idx, x_coords, y_coords);

    %% ------------------- SMOOTH & TIME-SCALE ------------------------ %%
%     [s_ref, ref_xy] = smoothAndParametrize(path_xy);
    [s_ref, ref_xy] = smoothAndParametrize(path_xy, G, x_coords, y_coords, resolution);
    [t_ref, xy_ref, dxy_ref, ddxy_ref] = timeScaleReference(s_ref, ref_xy, v_max, a_max, min_dt);

    %% ------------------- BUILD FULL REFERENCE ----------------------- %%
    ref_data = struct();
    ref_data.t    = t_ref;
    ref_data.x    = xy_ref(:,1);
    ref_data.y    = xy_ref(:,2);
    ref_data.z    = z_ref_const*ones(size(t_ref));
    ref_data.phi     = zeros(size(t_ref)); % roll
    ref_data.theta   = zeros(size(t_ref)); % pitch
    ref_data.psi  = psi_ref_const*ones(size(t_ref));
    ref_data.vx   = dxy_ref(:,1);
    ref_data.vy   = dxy_ref(:,2);
    ref_data.vz   = zeros(size(t_ref));
    ref_data.phidot  = zeros(size(t_ref));
    ref_data.thetadot= zeros(size(t_ref));
    ref_data.psidot  = zeros(size(t_ref));
    ref_data.psir = zeros(size(t_ref));
    ref_data.ax   = ddxy_ref(:,1);
    ref_data.ay   = ddxy_ref(:,2);
    ref_data.az   = zeros(size(t_ref));
    ref_data.psi2 = zeros(size(t_ref));

    % Function handle untuk query mirip QuadrotorReferenceTrajectory1
    ref_fun = @(t) referenceQuery(t, ref_data);
%     xdesired = [ref_data.x;ref_data.y;ref_data.z;ref_data.phi;ref_data.theta;ref_data.psi;ref_data.vx;ref_data.vy;ref_data.vz;ref_data.phidot;ref_data.thetadot;ref_data.psidot];


    figure;
    imagesc(x_coords, y_coords, G'); set(gca,'YDir','normal'); axis equal tight;
    colormap(gray);
    hold on;
    plot(start_xy(1), start_xy(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    plot(goal_xy(1), goal_xy(2), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
    plot(path_xy(:,1), path_xy(:,2), 'r--', 'LineWidth', 2); % jalur asli
    plot(ref_xy(:,1),  ref_xy(:,2),  'b-', 'LineWidth', 2); % jalur halus
    legend('Start','Goal','Dijkstra Path','Smoothed Path');
    title('Path Planning: Dijkstra vs Smoothed Path');
    xlabel('X (m)'); ylabel('Y (m)');
    
    speed = sqrt(dxy_ref(:,1).^2 + dxy_ref(:,2).^2);
    figure;
    plot(t_ref, speed, 'b-', 'LineWidth', 2);
    grid on;
    xlabel('Time (s)'); ylabel('Speed (m/s)');
    title('Speed Profile After Time Scaling');
    
    accel = sqrt(ddxy_ref(:,1).^2 + ddxy_ref(:,2).^2);
    figure;
    plot(t_ref, accel, 'r-', 'LineWidth', 2);
    grid on;
    xlabel('Time (s)'); ylabel('Acceleration (m/s^2)');
    title('Acceleration Profile After Time Scaling');

end


%% ------------------- PLOTS ------------------------------------------ %%
% figure('Name','Occupancy + Path'); hold on; axis equal; grid on;
% imagesc(x_coords, y_coords, ~G'); set(gca,'YDir','normal'); colormap(gray);
% plot(path_xy(:,1), path_xy(:,2), 'r.-', 'LineWidth', 1.5, 'DisplayName','Dijkstra path');
% plot(ref.x, ref.y, 'b-', 'LineWidth', 1.5, 'DisplayName','Smoothed ref');
% plot(start_xy(1), start_xy(2),'go','MarkerFaceColor','g');
% plot(goal_xy(1),  goal_xy(2), 'mo','MarkerFaceColor','m');
% legend; title('Map (white = free), Dijkstra path & Smoothed Trajectory'); xlabel('x [m]'); ylabel('y [m]');
% 
% figure('Name','3D Trajectory'); plot3(ref.x, ref.y, ref.z, 'b-'); hold on; grid on; axis equal;
% plot3(log.x(:,1), log.x(:,2), log.x(:,3), 'r-'); xlabel('x'); ylabel('y'); zlabel('z');
% legend('Reference','NMPC State'); title('3D Trajectory Tracking');
% 
% figure('Name','States Over Time');
% subplot(3,1,1); plot(log.t, log.x(:,1), log.t, log.xd(:,1)); ylabel('x [m]'); grid on; legend('x','x_d');
% subplot(3,1,2); plot(log.t, log.x(:,2), log.t, log.xd(:,2)); ylabel('y [m]'); grid on; legend('y','y_d');
% subplot(3,1,3); plot(log.t, log.x(:,3), log.t, log.xd(:,3)); ylabel('z [m]'); grid on; legend('z','z_d'); xlabel('t [s]');
% 
% figure('Name','Inputs');
% subplot(4,1,1); plot(log.t, log.u(:,1)); grid on; ylabel('T [N]');
% subplot(4,1,2); plot(log.t, log.u(:,2)); grid on; ylabel('Mx [Nm]');
% subplot(4,1,3); plot(log.t, log.u(:,3)); grid on; ylabel('My [Nm]');
% subplot(4,1,4); plot(log.t, log.u(:,4)); grid on; ylabel('Mz [Nm]'); xlabel('t [s]');
% 
% disp('Done. You can now replace your QuadrotorReferenceTrajectory1 with ref_fun.');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                           HELPER FUNCTIONS                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [G, x_coords, y_coords] = buildOccupancyGrid(map_size, res, obstacles, r_drone, r_safe)
    nx = ceil(map_size(1)/res)+1; ny = ceil(map_size(2)/res)+1;
    x_coords = linspace(0, map_size(1), nx);
    y_coords = linspace(0, map_size(2), ny);
    G = false(nx, ny); % false = free, true = occupied (note transpose when plotting)
    inflate = r_drone + r_safe;   % total margin
    for i=1:size(obstacles,1)
        x0=obstacles(i,1); y0=obstacles(i,2); w=obstacles(i,3); h=obstacles(i,4);

        xi_min = max(1, round((x0 - inflate)/res)+1);
        xi_max = min(nx, round((x0 + w + inflate)/res)+1);

        yi_min = max(1, round((y0 - inflate)/res)+1);
        yi_max = min(ny, round((y0 + h + inflate)/res)+1);

        G(xi_min:xi_max, yi_min:yi_max) = true;
    end
end

function ij = world2grid(p_xy, x_coords, y_coords)
    [~, ix] = min(abs(x_coords - p_xy(1)));
    [~, iy] = min(abs(y_coords - p_xy(2)));
    ij = [ix, iy];
end

function P = grid2world(idx_list, x_coords, y_coords)
    % pastikan index bertipe integer dan >= 1
    idx_list = round(idx_list);  
    if any(idx_list(:) < 1)
        error('grid2world: Index < 1 ditemukan: %s', mat2str(idx_list));
    end
    
    P = zeros(size(idx_list,1), 2);
    for k = 1:size(idx_list,1)
        P(k,1) = x_coords(idx_list(k,1));
        P(k,2) = y_coords(idx_list(k,2));
    end
end


function path = dijkstraGrid(G, start_idx, goal_idx)
    % 4-connected Dijkstra on binary grid G (true=occupied). Returns list of indices [ix,iy].
    sz = size(G);
    start = sub2ind(sz, start_idx(1), start_idx(2));
    goal  = sub2ind(sz, goal_idx(1),  goal_idx(2));
    N = prod(sz);
    dist = inf(N,1); prev = zeros(N,1,'uint32'); visited = false(N,1);
    dist(start)=0;

    % Min-heap via simple linear scan (ok for medium grids)
    while true
        % pick unvisited with minimal dist
        [~, u] = min(dist + visited*1e12);
        if isinf(dist(u)) || u==goal; break; end
        visited(u)=true;
        [ix,iy] = ind2sub(sz, u);
        nbrs = [ix-1,iy; ix+1,iy; ix,iy-1; ix,iy+1];
        for j=1:4
            x=nbrs(j,1); y=nbrs(j,2);
            if x>=1 && x<=sz(1) && y>=1 && y<=sz(2) && ~G(x,y)
                v = sub2ind(sz, x,y);
                if visited(v); continue; end
                alt = dist(u) + 1; % unit cost per cell
                if alt < dist(v)
                    dist(v)=alt; prev(v)=u;
                end
            end
        end
    end

    if isinf(dist(goal))
        path = [];
        return;
    end
    % reconstruct
    P = uint32([]); u = goal;
    while u~=0
        P(end+1) = u; %#ok<AGROW>
        if u==start; break; end
        u = prev(u);
    end
    P = fliplr(P);
    path = zeros(numel(P),2);
    for k=1:numel(P)
        [ix,iy] = ind2sub(sz, P(k));
        path(k,:) = [ix,iy];
    end
end

function [s_ref, ref_xy] = smoothAndParametrize(path_xy, G, x_coords, y_coords, res)
    smoothed = shortcutSmooth(path_xy, G, x_coords, y_coords);

    % 2) Interpolasi spline untuk membuat halus
    t = 1:size(smoothed,1);
    tt = linspace(1, size(smoothed,1), 5*size(smoothed,1)); % 5x lebih rapat
    x_smooth = interp1(t, smoothed(:,1), tt, 'pchip');
    y_smooth = interp1(t, smoothed(:,2), tt, 'pchip');

    % 3) Hitung panjang lintasan kumulatif (arc length)
    ds = sqrt(diff(x_smooth).^2 + diff(y_smooth).^2);
    s_ref = [0; cumsum(ds(:))];

    ref_xy = [x_smooth(:), y_smooth(:)];
end

function P2 = shortcutSmooth(P, iters, G, x_coords, y_coords, res, inflate)
    if size(P,1)<=2, P2 = P; return; end
    P2 = P;
    for k = 1:iters
        if size(P2,1) <= 2, break; end
        i = randi([1, size(P2,1)-2]);
        j = randi([i+2, size(P2,1)]);
        % cek apakah segmen (P2(i)->P2(j)) bebas obstacle
        if isFreeLine(P2(i,:), P2(j,:), G, x_coords, y_coords, res, inflate)
            P2 = [P2(1:i,:); P2(j,:); P2(j+1:end,:)]; %#ok<AGROW>
        end
    end
end

function ok = isFreeLine(pA, pB, G, x_coords, y_coords, res, inflate)
    % Sampling rapat sepanjang garis dan cek occupancy
    L = norm(pB - pA);
    if L < 1e-6, ok = true; return; end
    % step: setengah sel
    step = max(res*0.5, 0.1);
    n    = max(2, ceil(L/step));
    t    = linspace(0,1,n)';
    pts  = pA.*(1-t) + pB.*t;

    % inflasi obstacle dalam satuan sel
    dilate_cells = max(0, round(inflate/res));

    sz = size(G);
    for r = 1:n
        % world -> nearest grid index
        [~, ix] = min(abs(x_coords - pts(r,1)));
        [~, iy] = min(abs(y_coords - pts(r,2)));
        % cek sekitar (inflasi)
        xlo = max(1, ix - dilate_cells); xhi = min(sz(1), ix + dilate_cells);
        ylo = max(1, iy - dilate_cells); yhi = min(sz(2), iy + dilate_cells);
        if any(G(xlo:xhi, ylo:yhi), 'all')
            ok = false; return;
        end
    end
    ok = true;
end


function [t_ref, xy_ref, dxy_ref, ddxy_ref] = timeScaleReference(s_ref, ref_xy, v_max, a_max, min_dt)
    % s_ref : jarak kumulatif (1D)
    % ref_xy: Nx2
    % v_max, a_max: batas kecepatan & percepatan
    % min_dt: sampling min

    % --- Time scaling based on trapezoidal velocity profile ---
    total_s = s_ref(end);
    t_accel = v_max / a_max;
    s_accel = 0.5 * a_max * t_accel^2;

    if total_s < 2*s_accel
        t_accel = sqrt(total_s / a_max);
        t_cruise = 0;
    else
        t_cruise = (total_s - 2*s_accel) / v_max;
    end

    total_time = 2*t_accel + t_cruise;

    % Sampling
    t_ref = (0:min_dt:total_time)';  

    % Parametric s(t)
    s_t = zeros(size(t_ref));
    for i = 1:length(t_ref)
        t = t_ref(i);
        if t <= t_accel
            s_t(i) = 0.5 * a_max * t^2;
        elseif t <= t_accel + t_cruise
            s_t(i) = s_accel + v_max * (t - t_accel);
        else
            td = t - (t_accel + t_cruise);
            s_t(i) = s_accel + v_max * t_cruise + v_max * td - 0.5 * a_max * td^2;
        end
    end

    % Interpolasi posisi X,Y
    xy_ref = interp1(s_ref, ref_xy, s_t, 'pchip');

    % Hitung turunan per kolom
    dx_ref = gradient(xy_ref(:,1), t_ref);
    dy_ref = gradient(xy_ref(:,2), t_ref);
    dxy_ref = [dx_ref, dy_ref];


    % Turunan kedua
    ddx_ref = gradient(dx_ref, t_ref);
    ddy_ref = gradient(dy_ref, t_ref);
    ddxy_ref = [ddx_ref, ddy_ref];

end


% function r = referenceQuery(t, ref)
%     % zero-order hold at ends
%     if t <= ref.t(1)
%         idx = 1;
%     elseif t >= ref.t(end)
%         idx = length(ref.t);
%     else
%         idx = find(ref.t>=t,1,'first');
%     end
%     r = [ref.x(idx); ref.y(idx); ref.z(idx); ref.psi(idx); ...
%          ref.vx(idx); ref.vy(idx); ref.vz(idx); ref.psir(idx); ...
%          ref.ax(idx); ref.ay(idx); ref.az(idx); ref.psi2(idx); 1];
% end

function x_ref = referenceQuery(t, ref)
    % Pastikan ref.t ada untuk interpolasi
    if ~isfield(ref, 't')
        error('ref_data.t tidak ditemukan. Tambahkan t_ref saat membuat ref_data.');
    end

    % Clamp waktu ke range
    if t <= ref.t(1)
        idx = 1;
    elseif t >= ref.t(end)
        idx = length(ref.t);
    else
        idx = [];
    end

    % Interpolasi semua variabel sesuai waktu
    if isempty(idx)
        x     = interp1(ref.t, ref.x, t, 'linear');
        y     = interp1(ref.t, ref.y, t, 'linear');
        z     = interp1(ref.t, ref.z, t, 'linear');
        phi   = interp1(ref.t, ref.phi, t, 'linear');
        theta = interp1(ref.t, ref.theta, t, 'linear');
        psi   = interp1(ref.t, ref.psi, t, 'linear');
        vx    = interp1(ref.t, ref.vx, t, 'linear');
        vy    = interp1(ref.t, ref.vy, t, 'linear');
        vz    = interp1(ref.t, ref.vz, t, 'linear');
        phidot    = interp1(ref.t, ref.phidot, t, 'linear');
        thetadot  = interp1(ref.t, ref.thetadot, t, 'linear');
        psidot    = interp1(ref.t, ref.psidot, t, 'linear');
    else
        % Kalau t di luar range, pakai nilai ujung
        x = ref.x(idx); y = ref.y(idx); z = ref.z(idx);
        phi = ref.phi(idx); theta = ref.theta(idx); psi = ref.psi(idx);
        vx = ref.vx(idx); vy = ref.vy(idx); vz = ref.vz(idx);
        phidot = ref.phidot(idx); thetadot = ref.thetadot(idx); psidot = ref.psidot(idx);
    end

    % Output persis 12 elemen sesuai model NMPC
    x_ref = [x; y; z; phi; theta; psi; vx; vy; vz; phidot; thetadot; psidot];
end