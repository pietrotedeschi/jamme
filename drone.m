%% Leveraging jamming to help drones complete their mission
%% Authors: Pietro Tedeschi, Gabriele Oligeri and Roberto Di Pietro
%% Baseline scenario:
%  clc; clear all; d = drone(); d = d.setwinddir(-pi/2); d = d.set_wind_strength(0); d.target_pos = [8,11]; d.mov = [0.1 0]; d = d.fly(); d.show()

classdef drone
    properties
        mSNR                  % Minimum Signal to Noise Ratio
        DmSNR                 % Distance at which the drone start the evasion
        idx_ev                % Time slot at which the evasion starts
        init_pos              % Drone initial position
        mov                   % Drone step. Polar coordinates in the drone's reference system.
        target_pos            % Position of the taret
        jam_pos               % Jammer position
        jam_est               % Jammer estimated position
        path                  % Path of the drone
        Pr                    % Log of all the received signal strength
        PrTg                  % Receiver Power Target
        drift                 % Drift component (polar coordinates)
        stepdrift             % Step value belonged to drift
        JAM_EVASION = false;  % Estimating the position of the jammer
        JAM = false;          % Jammer Position Found
        angin                 % Angle used to go towards target
        de                    % Distance Jammer-Target
        dt                    % Distance Drone-Target
        ptD                   % Distance beetween Driftted Drone and Jammer
        ptND                  % Matrix: (x,y) to reach the target, distance, power received
        PrND                  % Support variable to store the ideal path
        pathND                % Support variable to store the ideal path
        ldt                   % Last distance between last path and target
        u                     % Compensation
        speed                 % Path coordinates inside the jamming area (last part of path)
        u_prev                % Previous Compensation
        kc                    % Gain
        rec_dir               % Direction for the recognition path
        ejradius              % Expected Jamming radius
        TARGETAPPROACH        % Approaching the target
        err                   % Difference between Pr and SNR used in the controller
        winddir               % Wind direction: vector tail at the jammer position
        windstrength          % Wind strength (vector)
        entrypoint            % Absolute coordinates of the entry point in the jamming area
        dronePosEstJam        % Position of the drone after having estimated the jammer position
        xstrength             % Wind Strength X component
        strength              % Wind Intensity
        first_maneuver        % First Maneuver coordinates
        pid_hist              % Log of the controller variables
        pos_jam_est_duration  % Time slots to estimate the jamming position
    end
    
    properties(Constant)
        f = 1.57542e9;      % Frequency
        Gr = 0;             % RX antenna gain (dB)
        Gt = 0;             % TX antenna gain (dB)
        Pt = 20;            % Transmission power (dB)
    end
    
    methods
        
        function obj = drone()
            obj.mSNR            = -30;
            obj.DmSNR           = 0;
            obj.init_pos        = [9, 0];
            obj.mov             = [0.1, 0];     %% 0 means straight forward, pi/2 left, -pi/2 right
            obj.jam_pos         = [10, 10];
            obj.path            = [];
            obj.target_pos      = [8, 10];
            obj.u               = NaN;
            obj.u_prev          = 0;
            obj.kc              = -20;
            obj.rec_dir         = 1;
            obj.ejradius        = -1;
            obj.TARGETAPPROACH  = 1;
            obj.entrypoint      = [NaN, NaN];
            obj.dronePosEstJam  = [NaN, NaN];
            obj.strength        = -1;
            obj.xstrength       = [];
            
            %% Compute the expected jammer radius
            obj.ejradius = 1 / (10^((obj.mSNR - obj.Pt) / 20) * 4 * pi / (physconst('lightspeed') / obj.f));
            
            obj.winddir = 0;
            obj.windstrength = [];
            obj.first_maneuver = [NaN, NaN];
            obj.pid_hist = [];
            obj.speed = [];
            obj.pos_jam_est_duration = -1;
        end
        
        function Pr = friis(obj, d)
            c = physconst('lightspeed');
            lambda = c / obj.f;
            Pr = obj.Pt + obj.Gt + obj.Gr + 20*log10(lambda./(4*pi*d));
        end

        function d = friisInv(obj, Pr)
            c = physconst('lightspeed');
            lambda = c / obj.f;
            d = 1 / (4*pi / lambda * 10^((Pr - obj.Pt - obj.Gt - obj.Gr)/20));
        end
        
        function obj = controller(obj, e, kc, j)
            %% Parameters: obj, Powers' difference, Gain, index, compensation
            %% Closed loop linear controller
            % Velocity Form
            Tc      =   1;
            kp      =   0.6*kc;
            ki      =   2*kp/Tc;
            kd      =   kp*Tc/8;
            ud      =   0;
            up      =   0;
           
            ui = ki*(e(end));

            if (j > 1)
                up = kp*(e(end)-e(end-1));
            end           
                       
            if (j > 2)
                ud = kd*(e(end) - 2*e(end-1) + e(end-2));
            end

            obj.u = ui + up + ud;            
            %obj.u = obj.u_prev + up + ui + ud;
            %obj.u_prev = obj.u;
        end
        
        function obj = controller2(obj, e, kc, j)
            %% Parameters: obj, Powers' difference, Gain, index, compensation
            %% Closed loop linear controller
            Tc      =   1;
            kp      =   0.6*kc;
            ki      =   2*kp/Tc;
            kd      =   kp*Tc/8;
            ud      =   0;
           
            up = kp*(e(end));
            
            ui = ki*(sum(e));
                       
            if (j > 1)
                ud = kd*(e(end) - e(end-1));
            end

            obj.u = ui + up + ud;
        end
        
        function obj = estimate_jammer_position(obj, i)
            %% If there are at least 20 RSS samples
            if obj.idx_ev > 0 && i - obj.idx_ev > 3
                par = CircleFitByPratt(obj.path(obj.idx_ev:end, :));
                %% Est. x and y, and radius
                obj.jam_est = [obj.jam_est; par];
                %% If there are at least 10 estimations of the center
                if length(obj.jam_est) > 10
                    if std(obj.jam_est(end-10:end, 3)) < 0.01
                        obj.JAM = true;
                        %% What we have to do after locating the jammer
                        % Distance between Jammer position and Target
                        obj.de = pdist([obj.target_pos; round(obj.jam_est(end,[1 2]))], 'euclidean');
                        obj.PrTg = friis(obj, obj.de);
                        
                        %% Compute the entrance angle, P1 = drone path, P2 = target
                        %obj.angin = atan2(obj.target_pos(2) - obj.path(end,2), obj.target_pos(1) - obj.path(end,1));
                    end
                end
            end
        end
        
        function obj = pos_jam_estimation(obj)
            i = 0; h = 1; 
            %% Compute jammer position, and fly around the jam area border
            while ~obj.JAM           
                %% Compute the RSS from the jammer
                d = pdist([obj.path(end, :); obj.jam_pos], 'euclidean');
                obj.Pr = [obj.Pr; [d, friis(obj, d)]];
                
                %% Update obj.mov according to the RSS
                if obj.JAM_EVASION                                                           
                    e = obj.Pr(:, 2) - obj.mSNR;
                    obj = controller(obj, e, 1, h);
                    new_yaw = obj.mov(end, 2) + obj.u;
                    obj.mov = [obj.mov; [obj.mov(end, 1), new_yaw]];

                    h = h + 1;
                    
                    obj.mov = [obj.mov; [obj.mov(end, 1), obj.mov(end, 2)]];
                    obj = estimate_jammer_position(obj, i);
                end
                
                %% First maneuver when jamming is detected
                if obj.Pr(end, 2) > obj.mSNR && ~obj.JAM_EVASION
                    obj.JAM_EVASION = true;
                    obj.DmSNR = d;
                    obj.idx_ev = i;
                    obj.mov = [obj.mov; [obj.mov(1, 1), obj.rec_dir * pi/2]]; %% First maneuver after detecting jamming (rotate left)
                    obj.first_maneuver = obj.path(end, :);
                end
                
                %% The drone makes a step in the mov(end, :) direction
                movXY = [obj.mov(end, 1) * cos(pi/2 + obj.mov(end, 2)), obj.mov(end, 1) * sin(pi/2 + obj.mov(end, 2))];
                obj.path = [obj.path; [obj.path(end, :) +  movXY]];
                
                i = i + 1;
            end
            obj.pos_jam_est_duration = i;
        end
        
        function obj = reach_entry_point(obj, dir)
            i = 1;
            while pdist([obj.path(end, :); obj.entrypoint], 'euclidean') > obj.mov(1, 1) 
                d = pdist([obj.path(end, :); obj.jam_pos], 'euclidean');
                obj.Pr = [obj.Pr; [d, friis(obj, d)]];

                %% Update obj.mov according to the RSS
                e = obj.Pr(:, 2) - obj.mSNR;                    
                obj = controller(obj, -dir*e, 1, i);
                new_yaw = obj.mov(end, 2) + obj.u;
                obj.mov = [obj.mov; [obj.mov(end, 1), new_yaw]];
                
                if i== 1 && dir > 0
                    obj.mov = [obj.mov; [obj.mov(end, 1), obj.mov(end, 2) + pi]];
                else
                    obj.mov = [obj.mov; [obj.mov(end, 1), obj.mov(end, 2)]];
                end
                
                %% The drone makes a step in the mov(end, :) direction
                movXY = [obj.mov(end, 1) * cos(pi/2 + obj.mov(end, 2)), obj.mov(end, 1) * sin(pi/2 + obj.mov(end, 2))];
                obj.path = [obj.path; [obj.path(end, :) +  movXY]];

                i = i + 1;
            end
            if pdist([obj.path(end, :); obj.entrypoint], 'euclidean') <= obj.mov(1, 1) 
                obj.path = [obj.path; obj.entrypoint];
            end
        end
        
        
        function obj = fly(obj)
            obj.path = obj.init_pos;
            obj = obj.pos_jam_estimation();
            obj.dronePosEstJam = obj.path(end, :);
            
            %% If there is no wind entrypoint = actual position
            if length(find(obj.strength)) > 0
                %% Compute the entry point in the jamming area
                if pdist([obj.target_pos; obj.jam_pos], 'euclidean') > 0.5
                    if abs(obj.winddir) ~= pi/2                    
                        q = obj.target_pos(2) - obj.target_pos(1) * tan(obj.winddir);
                        [xout,yout] = linecirc(tan(obj.winddir), q, obj.jam_pos(1), obj.jam_pos(2), obj.ejradius);

                        if pdist([obj.target_pos; [xout(1), yout(1)]], 'euclidean') > pdist([obj.target_pos; [xout(2), yout(2)]], 'euclidean')
                            obj.entrypoint = [xout(2), yout(2)];
                        else
                            obj.entrypoint = [xout(1), yout(1)];
                        end
                    else
                        obj.entrypoint = [obj.target_pos(1), obj.jam_pos(2) + sin(acos((obj.target_pos(1) - obj.jam_pos(1)) / obj.ejradius) * sign(obj.target_pos(2) - obj.jam_pos(2))) * obj.ejradius];
                    end
                       
                else
                    obj.entrypoint = obj.jam_pos + [obj.ejradius * cos(obj.winddir + pi), obj.ejradius * sin(obj.winddir + pi)];
                end

                %% Compute the line equation between the drone and the jammer
                p = polyfit([obj.path(end, 1), obj.jam_pos(1)], [obj.path(end, 2), obj.jam_pos(2)], 1);
                x = linspace(0, 100, 100); fx = polyval(p, x);            

                %% Direction to reach the entry point: >0 back, <0 moving fwd
                dir = ((obj.entrypoint(1) - obj.path(end, 1)) * (fx(end) - obj.path(end, 2)) - (obj.entrypoint(2) - obj.path(end, 2)) * (x(end) - obj.path(end, 1)));                        
                obj = obj.reach_entry_point(sign(dir));
            else
                obj.entrypoint = obj.dronePosEstJam;
                %obj.winddir = atan2(obj.target_pos(2) - obj.entrypoint(2), obj.target_pos(1) - obj.entrypoint(1));
            end
                        
            %% Compute all the possible coordinates and powers that we expect to see on the line from the starting point to the end point
            %% Compute the power we expect to see on the path to follow towards the target
            %% Path to the target without drift and angin computation
            obj.PrND = obj.Pr;
            obj.pathND = obj.path;
            obj.dt = pdist([obj.target_pos; obj.path(end,[1 2])], 'euclidean');
            
            obj.angin = atan2(obj.target_pos(2) - obj.path(end,2), obj.target_pos(1) - obj.path(end,1));
            
            while obj.dt >= obj.mov(1, 1) * 2
                d = pdist([obj.pathND(end, :); obj.jam_pos], 'euclidean');
                obj.PrND = [obj.PrND; [d, friis(obj, d)]];
                
                %% Compute the entrance angle, P1 = drone path, P2 = target
                movXY2 = [obj.mov(end, 1) * cos(obj.angin), obj.mov(end, 1) * sin(obj.angin)];
                obj.pathND = [obj.pathND; [obj.pathND(end, :) +  movXY2]];
                obj.ptND = [obj.ptND; [d friis(obj, d)]];
                obj.dt = pdist([obj.target_pos; obj.pathND(end,[1 2])], 'euclidean');
            end
          
            
           %% Expected power at a certain distance between the entry point and the drone
           xePr = linspace(0, pdist([obj.target_pos; obj.entrypoint], 'euclidean'), 1000);
           expectedPr = friis(obj,  xePr);          
           expectedPrattarget = friis(obj, pdist([obj.target_pos; obj.jam_est(end, 1:2)], 'euclidean'));
           
           i = 1; dP = []; e = [];
           
           outJammingArea = true;
           RefSpeed = obj.mov(1, 1);
           while (obj.Pr(end, 2) - obj.Pr(end-1, 2) > 0 && obj.Pr(end, 2) < expectedPrattarget) || outJammingArea
                outJammingArea = false;

                dDroneEntry = pdist([obj.path(end, :); obj.entrypoint], 'euclidean');

                speed = obj.friisInv(obj.Pr(end-1, 2)) - obj.friisInv(obj.Pr(end, 2));
                
                if i > 2
                    e = [e; RefSpeed - speed];
                    obj = controller2(obj, e, 0.6, i);  
                else
                    e = [e; 0];
                    obj.u = 0;                    
                end

                % Drone movement in the universal reference system
                xd = (obj.mov(end, 1) + obj.u) * cos(obj.angin);
                yd = (obj.mov(end, 1) + obj.u) * sin(obj.angin);
                if length(find(obj.strength)) == 0
                    x = xd;
                    y = yd;
                    wx = 0; wy =0;
                else
                    wx = obj.strength(max(find(obj.xstrength <= dDroneEntry))) * cos(obj.winddir);
                    wy = obj.strength(max(find(obj.xstrength <= dDroneEntry))) * sin(obj.winddir);
                    x = xd + wx;
                    y = yd + wy;
                end                

                % Logging
                movXY   = [x, y];
                
                try
                    newpos = obj.path(end, :) +  movXY;
                catch
                    disp();
                end

                %% Drone position log in the universal reference system
                obj.path = [obj.path; newpos];              
                d = pdist([obj.path(end, :); obj.jam_pos], 'euclidean');
                obj.ptD = [obj.ptD; [d, friis(obj, d)]];
                obj.Pr = [obj.Pr; [d, friis(obj, d)]];                
                obj.speed = [obj.speed; sqrt(x^2 + y^2)];                
                obj.pid_hist = [obj.pid_hist; [speed, RefSpeed-speed, sqrt(wx^2 + wy^2), obj.u, sqrt(x^2 + y^2)]];
                
                i = i + 1;
           end
                      
        end
        
       
        function show(obj)
            hold on
            grid on
            f1 = figure(1);
            plot(obj.jam_pos(:,1), obj.jam_pos(:,2), 'or', 'MarkerSize', 5, 'MarkerFaceColor', 'r');
            plot(obj.jam_est(end, 1), obj.jam_est(end, 2), '*g');
            plot(obj.target_pos(:,1), obj.target_pos(:,2), 'xk', 'MarkerSize', 15, 'MarkerFaceColor', 'k', 'LineWidth', 2)
            plot(obj.path(:,1), obj.path(:,2), 'o-k')          
            pos = [obj.jam_est(end,[1 2]) - obj.DmSNR, 2*obj.DmSNR, 2*obj.DmSNR]; rectangle('Position', pos, 'Curvature', [1 1], 'EdgeColor', 'r')
            
            %% Print the wind vector
            [xw, yw] = pol2cart(obj.winddir, 2);
            plot([obj.jam_pos(1), obj.jam_pos(1) + xw], [obj.jam_pos(2), obj.jam_pos(2) + yw], '-g', 'LineWidth', 2);
            
            plot(obj.entrypoint(1), obj.entrypoint(2), 'or')
        end
                
        function obj = set_wind_strength(obj, a)
           %% Precompute wind intensity as a function of the distance to the entry point
           obj.xstrength = linspace(0, 2*obj.ejradius, 1000); 
           %% SIN wave
           %obj.strength = sin((2*pi) / obj.ejradius  * obj.xstrength) * a;
           
           %% STEP wind
           x = size(obj.xstrength, 2);
           obj.strength =  [ones(x, 1) * a];
           obj.strength(1:100) = zeros(100, 1);
        end
        
        function obj = setwinddir(obj, alpha)          
            obj.winddir = alpha;
        end
    end
end
