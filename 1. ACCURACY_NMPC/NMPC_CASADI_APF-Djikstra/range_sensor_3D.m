function d = range_sensor_3D(pos, obstacles, sensor)
% pos        : [3x1] posisi drone
% obstacles  : struct array dengan fields .center (3x1), .radius
% sensor     : struct dengan .dir (3x1), .max_range
%
% return     : jarak ke obstacle terdekat dalam arah sensor

dir = sensor.dir / norm(sensor.dir);
d = sensor.max_range; % default (tidak ada tabrakan)

for i = 1:length(obstacles)
    c = obstacles(i).center;
    r = obstacles(i).radius;
    
    % vektor dari drone ke pusat obstacle
    oc = c - pos;
    
    % solusi intersection ray-sphere
    b = dot(oc, dir);
    discriminant = b^2 - (dot(oc,oc) - r^2);
    
    if discriminant >= 0
        t = b - sqrt(discriminant); % titik terdekat di depan
        if t > 0 && t < d
            d = min(d, t);
        end
    end
end
end
