classdef PID_Controller2 < handle
    % PID_Controller  Simple PID controller class converted from Python
    %
    % Usage:
    %   pid = PID_Controller(Kp, Kd, Ki, Ki_sat, dt);
    %   des_acc = pid.control_update(pos_error, vel_error);
    %
    % Kp, Kd, Ki, Ki_sat are vectors [3x1] (or [1x3]) for x,y,z
    % pos_error, vel_error should be column vectors [3x1] (or row vectors).
    
    properties
        Kp
        Kd
        Ki
        Ki_sat
        dt
        int     % integrator state [3x1]
    end
    
    methods
        function obj = PID_Controller(Kp, Kd, Ki, Ki_sat, dt)
            % Constructor
            obj.Kp = Kp(:);    % force column
            obj.Kd = Kd(:);
            obj.Ki = Ki(:);
            obj.Ki_sat = Ki_sat(:);
            obj.dt = dt;
            obj.int = zeros(3,1);
        end
        
        function des_acc = control_update(obj, pos_error, vel_error)
            % control_update  compute desired acceleration (3x1)
            %
            % pos_error, vel_error: vectors (3x1 or 1x3)
            
            pos_error = pos_error(:);
            vel_error = vel_error(:);
            
            % Update integral
            obj.int = obj.int + pos_error * obj.dt;
            
            % Anti-windup / saturate integrator per-element while preserving sign
            % If Ki_sat element is zero or negative, no saturation applied for that element.
            for i = 1:3
                sat_val = obj.Ki_sat(i);
                if sat_val > 0
                    mag = abs(obj.int(i));
                    if mag > sat_val
                        obj.int(i) = sign(obj.int(i)) * sat_val;
                    end
                end
            end
            
            % Compute desired acceleration (element-wise)
            % des_acc = Kp.*pos_error + Ki.*int + Kd.*vel_error
            des_acc = obj.Kp .* pos_error + obj.Ki .* obj.int + obj.Kd .* vel_error;
        end
        
        function reset_integrator(obj)
            % reset_integrator  reset integrator state to zero
            obj.int = zeros(3,1);
        end
    end
end
