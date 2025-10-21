function animation = drone_Animation2(t,x,y,z,roll,pitch,yaw, num_obs,z_pos,obs_center,obs_radius)
% Animation for QuadCopter (Modified: obstacle changes color when hit)

%% Parameters
D2R = pi/180;
b   = 0.6; a = b/3; H = 0.06; H_m = H+H/2; r_p = b/4;
ro = 45*D2R;
Ri = [cos(ro) -sin(ro) 0; sin(ro) cos(ro) 0; 0 0 1];
base_co = [-a/2 a/2 a/2 -a/2; -a/2 -a/2 a/2 a/2; 0 0 0 0];
base = Ri*base_co;

to = linspace(0, 2*pi);
xp = r_p*cos(to); yp = r_p*sin(to); zp = zeros(1,length(to));

%% Figure setup
fig1 = figure('pos', [0 50 800 600]);
hg = gca; view(68,53); grid on; axis equal;
margin = 1;
xlim([min(x)-margin max(x)+margin]);
ylim([min(y)-margin max(y)+margin]);
zlim([max(0,min(z)-margin) max(z)+margin]);
title('Drone Animation'); xlabel('X[m]'); ylabel('Y[m]'); zlabel('Z[m]');
hold(gca, 'on');

%% Plot obstacles (store handle)
h_obs = gobjects(1, num_obs);
for j = 1:num_obs
    c = obs_center(:,j);
    r = obs_radius(j);
    h = z_pos;
    [Xc, Yc, Zc] = cylinder(r, 50);
    Zc = Zc * h; Xc = Xc + c(1); Yc = Yc + c(2);
    h_obs(j) = surf(Xc, Yc, Zc, 'FaceColor',[0.5 0.5 0.5], ...
                    'FaceAlpha',0.5, 'EdgeColor','none');
end

%% Drone body
drone(1) = patch([base(1,:)],[base(2,:)],[base(3,:)],'r');
drone(2) = patch([base(1,:)],[base(2,:)],[base(3,:)+H],'r');
alpha(drone(1:2),0.7);
[xcyl, ycyl, zcyl] = cylinder([H/2 H/2]);
drone(3) = surface(b*zcyl-b/2,ycyl,xcyl+H/2,'facecolor','b');
drone(4) = surface(ycyl,b*zcyl-b/2,xcyl+H/2,'facecolor','b');
alpha(drone(3:4),0.6);
drone(5) = surface(xcyl+b/2,ycyl,H_m*zcyl+H/2,'facecolor','r');
drone(6) = surface(xcyl-b/2,ycyl,H_m*zcyl+H/2,'facecolor','r');
drone(7) = surface(xcyl,ycyl+b/2,H_m*zcyl+H/2,'facecolor','r');
drone(8) = surface(xcyl,ycyl-b/2,H_m*zcyl+H/2,'facecolor','r');
alpha(drone(5:8),0.7);
drone(9)  = patch(xp+b/2,yp,zp+(H_m+H/2),'c','LineWidth',0.5);
drone(10) = patch(xp-b/2,yp,zp+(H_m+H/2),'c','LineWidth',0.5);
drone(11) = patch(xp,yp+b/2,zp+(H_m+H/2),'p','LineWidth',0.5);
drone(12) = patch(xp,yp-b/2,zp+(H_m+H/2),'p','LineWidth',0.5);
alpha(drone(9:12),0.3);

%% Group object
combinedobject = hgtransform('parent',hg);
set(drone,'parent',combinedobject);

%% Video Writer
videoFile = 'drone_animation.mp4';
v = VideoWriter(videoFile,'MPEG-4');
v.FrameRate = 30;
open(v);

%% Animation loop
for i = 1:length(x)
    plot3(x(1:i),y(1:i),z(1:i),'b:','LineWidth',1.5);

    translation = makehgtform('translate',[x(i) y(i) z(i)]);
    rotation1 = makehgtform('xrotate',(pi/180)*roll(i));
    rotation2 = makehgtform('yrotate',(pi/180)*pitch(i));
    rotation3 = makehgtform('zrotate',(pi/180)*yaw(i));
    set(combinedobject,'matrix',translation*rotation3*rotation2*rotation1);

    % --- Deteksi tabrakan ---
    for j = 1:num_obs
        c = obs_center(:,j);
        r = obs_radius(j);
        h = z_pos;

        % Jarak horizontal drone ke pusat obstacle
        dist_xy = sqrt((x(i)-c(1))^2 + (y(i)-c(2))^2);

        % Jika drone berada di area obstacle
        if dist_xy <= r && z(i) <= h
            set(h_obs(j), 'FaceColor',[1 0 0]); % ubah jadi merah
        else
            % opsional: kembalikan ke warna abu-abu
            set(h_obs(j), 'FaceColor',[0.5 0.5 0.5]);
        end
    end

    drawnow;
    frame = getframe(fig1);
    writeVideo(v, frame);
end

close(v);
disp(['Video disimpan sebagai: ' videoFile]);
end
