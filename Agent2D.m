classdef Agent2D < handle
    %UNTITLED7 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        pos = [0, 0]
        yaw = 0
        vmax = 0.8
        wmax = 5
        shape = [10, -7.5; 13, 0; 10, 7.5; -10, 7.5; -10, -7.5; 10, -7.5] * 1e-2
        color = [0 0 1]
        dt = 1/5
        hb = []
        hc = []
        camidx = 1
        
        etc
        enable = true
        
        plan
        t
        vop
        
        dist_tol = 0.1
    end
    
    methods
        
        function obj = Agent2D(varargin)
            % assign custom options
            for k = 1 : 2 : (nargin)
                switch varargin{k}
                    case properties(obj)
                        obj.(varargin{k}) = varargin{k+1};
                        
                    otherwise
                        warning(['Unknown input parameter: ' varargin{k}]);
                end
            end
            obj.etc = struct('hplot', [], 'rvector', [0, 0], 'ptarget', [0, 0], ...
                'cs', zeros(3,2));
        end
        
        function set_plan(obj, wpoint, perf)
            % wpoint: way points of the plan, Nx2
            % perf: [0, 100] speed performance

%             wpoint = [obj.pos; wpoint];
            s = [0; cumsum(sqrt(sum(diff(wpoint).^2, 2)))];
            s = s + linspace(0, 1e-5, length(s))';
            ss = (s(1) : obj.vmax*obj.dt : s(end))';
            obj.plan = interp1(s, wpoint, ss);
            
            obj.vop = obj.vmax * perf / 100;
        end
        
        
        function [u, DV, dyaw] = line_follow(obj)
            
            % find closest point to plan
            [dist, idx] = min( sqrt(sum( (obj.pos - obj.plan).^2, 2 ) ) );
            if idx == 1 && size(obj.plan,1) > 1
                dd = obj.plan(idx+1,:) - obj.plan(idx,:);
                dist = 0;
                
            elseif idx ~= size(obj.plan,1)
                dd = obj.plan(idx+1,:) - obj.plan(idx,:);
                pts = [obj.pos; obj.plan(idx+[0,1],:); obj.pos];
                sig = sum(pts(1:end-1,1).*pts(2:end,2) - pts(2:end,1).*pts(1:end-1,2)); % side of line
                dist = dist * sign(sig);

            else
                dd = obj.plan(end,:) - obj.pos;
                dist = 0;
            end
            
            K = [0.5, 5, 1];
            dyaw = -min(max(K(1) * dist, -pi/2), pi/2) + atan2(dd(2), dd(1)); % desired yaw
            w = K(2) * asin(sin(dyaw - obj.yaw));
            if sqrt(sum((obj.plan(end,1:2) - obj.pos).^2, 2)) > obj.dist_tol
                v = max(obj.vop - 0.01*abs(w), 0);
            else
                v = 0;
            end

            u = [v, w];
            DV = K(3) * w;
        end
        
        
        function [u, DV, dyaw] = line_follow2(obj)
            
            % find closest point to plan
            mindist = Inf;
            for k = 1 : size(obj.plan, 1)-1
                d = obj.plan(k+1,1:2) - obj.plan(k,1:2);
                n = [-d(2), d(1)];
                n = n / sqrt(n * n');
                
                alf = [d', n'] \ (obj.pos - obj.plan(k,1:2))';
                if alf(1) > 0 && alf(1) < 1 && abs(alf(2)) < mindist
                    mindist = abs(alf(2));
                    y = alf(2);
                    dd = d;
                end 
            end
            
            if isinf(mindist)
                [y, idx] = min( sum( (obj.pos - obj.plan(:,1:2)).^2, 2 ) );
                if idx ~= size(obj.plan,1)
                    pts = [obj.pos; obj.plan(idx+[0,1],1:2); obj.pos];
                    A = sum(pts(1:end-1,1).*pts(2:end,2) - pts(2:end,1).*pts(1:end-1,2));
                    y = sign(A) * y;
                    dd = obj.plan(idx+1,1:2) - obj.plan(idx,1:2);
                else % past destination
                    dd = obj.plan(end,1:2) - obj.pos;
                    y = 0;

                end
            end
            
            K = [2, 3, 1];
            dyaw = -min(max(K(1) * y, -pi/2), pi/2) + atan2(dd(2), dd(1));
            w = K(2) * (dyaw - obj.yaw);
            if sqrt(sum((obj.plan(end,1:2) - obj.pos).^2, 2)) > obj.dist_tol
                v = obj.vop;
            else
                v = 0;
            end
            
            u = [v, w];
            DV = K(3) * w;
        end
        
        
        function u = pctrl(obj, tpos)
            % u: [linear vel; angular vel] m/s, rad/s
            
            epos = (tpos - obj.pos) * obj.Rz(obj.yaw); % position error on car frame
            erot = atan2(epos(2), epos(1)); 

            zerr = [epos(1), erot];
            if sqrt( epos * epos' ) > 15e-2 % if error is large
                u = [   70.5, 0
                    0, 2.5] * zerr';
            else % if error is small
                u = 10*[   2, 0
                    0, 1] * zerr';
            end
        end
        
        function correct(obj)
            % correct position using camera, if exist
            if ~isempty(obj.hc)
                [tracked, pos0, yaw0] = Agent2D.read_mocap(obj.hc, obj.camidx);
                if tracked
                    obj.pos = pos0;
                    obj.yaw = yaw0;
                end
            end
        end
        
        function predict(obj, v, w)
            if abs(w) > 1e-2
                rad = v/w; % radius of turn
                pos2 = obj.pos + rad*[sin(w*obj.dt), 1-cos(w*obj.dt)] * obj.Rz(obj.yaw)';
            else
                pos2 = obj.pos + [v*obj.dt, 0] * obj.Rz(obj.yaw)';
            end
            rot2 = obj.yaw + w*obj.dt;

            obj.pos = pos2;
            obj.yaw = rot2;
        end
        
        function pos0 = step(obj, varargin)
            % agent.step(): follows pre-assigned plan
            % agent.step(posd): moves towards posd

            pos0 = obj.pos;
            obj.correct()
            
%             % compute repulsive force to avoid collision
%             [mindist, rf] = obj.map.collide(obj.pos);
%             mindist = max(mindist, 0);
%             rf = 1e-3 * rf / abs(1e-3 + mindist.^3);
%             
%             A = 0.3/(1/0.1 - 1); % at 0.3 m, weight is 0.1
%             weight = A/(mindist + A);
% %             weight = 0;
%             tpos = weight * (obj.pos + 5*rf * obj.vmax * obj.dt) + (1-weight) * tpos;
%             
%             obj.rvector = rf;

%             % retrieve from plan
%             if nargin == 1
%                 obj.t = obj.t + obj.dt;
%                 if size(obj.plan, 1) >= 2 && obj.t < obj.plan(end,3)
%                     
%                     tpos = interp1(obj.plan(:,3), obj.plan(:,1:2), obj.t);
%                 else
% %                     warning('no pre-assigned plan')
%                     tpos = obj.pos;
%                 end
%             else
%                 tpos = varargin{1};
%             end
%             
%             obj.etc.ptarget = tpos;
            
            % differential drive control
%             u = pctrl(obj, tpos);
            if ~isempty(obj.plan) && obj.enable
                u = line_follow(obj);
            else
                u = [0, 0];
            end
            
            v = u(1);
            w = u(2);
            
            if abs(w) > obj.wmax
                w = obj.wmax * sign(w);
            end
            if abs(v) + (obj.vmax/obj.wmax)*abs(w) > obj.vmax
                v = sign(v) * (obj.vmax - (obj.vmax/obj.wmax)*abs(w));
            end

            % predict new pose
            obj.predict(v, w);
            
            % update motors, if exist
            if ~isempty(obj.hb)
                k1 = obj.vmax/10;
                k2 = obj.wmax/10;
                volt = [k1, k1; k2, -k2] \ [v; w];

                if max(abs(volt)) > 5
                    volt = volt * 5 / max(abs(volt));
                end
                fprintf(obj.hb, '%dr %dr\n', (round(volt*100/5)));
            end
        end
        
        function plot(obj)
            shape1 = obj.shape * obj.Rz(obj.yaw)' + obj.pos;
            if isempty(obj.etc.hplot) || ~isvalid(obj.etc.hplot(1))
%                 obj.map.plot()
                obj.etc.hplot(1) = fill(shape1(:,1), shape1(:,2), obj.color, ...
                    'FaceAlpha', 0.8, 'LineStyle', 'none');
%                 obj.etc.hplot(2) = quiver(0, 0, 0, 0);
%                 obj.etc.hplot(3) = plot(0, 0, 'x', 'markersize', 20, 'linewidth', 2);
%                 obj.etc.hplot(4) = plot(0, 0, '--');
%                 obj.etc.hplot(5) = plot(0, 0);
%                 obj.etc.hplot(6) = plot(0, 0, 'o');
                
                obj.etc.hplot = handle(obj.etc.hplot);
%                 set(obj.etc.hplot,'Parent',obj.map.hax)
                
            else
            set(obj.etc.hplot(1), 'Vertices', shape1)
%             set(obj.hplot(2), 'XData', obj.pos(1), 'YData', obj.pos(2), 'UData', obj.rvector(1), 'VData', obj.rvector(2), 'Color', [1 0 0])
%             set(obj.etc.hplot(3), 'XData', obj.etc.ptarget(1), 'YData', obj.etc.ptarget(2))
            if ~isempty(obj.plan)
%                 set(obj.etc.hplot(4), 'XData', obj.plan(:,1), 'YData', obj.plan(:,2))
            end
%             set(obj.etc.hplot(5), 'XData', obj.etc.cs(:,1), 'YData', obj.etc.cs(:,2))
%             set(obj.etc.hplot(6), 'XData', obj.etc.cs(1,1), 'YData', obj.etc.cs(1,2))
            end
        end
        

    end
    
    methods (Static)
        function R = Rz(q)
            R = [cos(q), -sin(q)
                sin(q), cos(q)]; 
        end
        
        function d = dist(p1, p2)
            d = sqrt(sum((p1 - p2).^2));
        end
        
        function [tracked, pos, rot] = read_mocap(hc, bidx)
            % bidx: body index

            f = hc.getFrame();
            rbody = f.RigidBody(bidx);
            tracked = rbody.Tracked;

            if tracked
                q0 = [cosd(-90/2), sind(-90/2), 0, 0];
                quat = double([rbody.qw, rbody.qx, rbody.qy, rbody.qz]);
                quat = quatmultiply(quatmultiply(quatinv(q0), quat), (q0));
                trans = double([rbody.x, rbody.y, rbody.z]);
                trans = quatrotate(q0, trans);

                pos = [trans(1), trans(2)];
                rot = quat2angle((quat), 'ZYX');
            else
                pos = [0, 0];
                rot = [0];
            end
        end
    end
    
end

