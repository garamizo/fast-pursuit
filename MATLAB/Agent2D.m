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
        hb = 0
        hc = 0
        camidx = 1
        
        etc
        enable = true
        
        plan
        t
        vop
        
        dist_tol = 0.1
        ctrl
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

            obj.ctrl.DesiredLinearVelocity = obj.vmax * perf / 100;
            obj.ctrl.set(wpoint);
        end
        
        
        function correct(obj)
            % correct position using camera, if exist
            [tracked, pos0, yaw0] = Agent2D.read_mocap(obj.hc, obj.camidx);
            if tracked
                obj.pos = pos0;
                obj.yaw = yaw0;
            end
        end
        
        function predict(obj, twist)
            
            pose = [obj.pos, obj.yaw] + twist * Rz(obj.yaw)' * obj.dt;
            obj.pos = pose(1:2);
            obj.yaw = pose(3);
        end
        
        function pos0 = step(obj)
            % agent.step(): follows pre-assigned plan
            % agent.step(posd): moves towards posd

            pos0 = obj.pos;
            try
                obj.correct()
            catch ME
                if ~strcmp(ME.identifier, 'FastPursuit:Natnet_is_closed')
                    rethrow(ME)
                end
            end

            twist = obj.ctrl.step([obj.pos, obj.yaw]);
            obj.predict(twist);
            
            volt = obj.ctrl.motorization(twist);
            try
            fprintf(obj.hb, '%.1f %.1f\n', volt);
            catch
            end
            
%             % update motors, if exist
%             k1 = obj.vmax/10;
%             k2 = obj.wmax/10;
%             volt = [k1, k1; k2, -k2] \ [v; w];
%             if max(abs(volt)) > 5
%                 volt = volt * 5 / max(abs(volt));
%             end
%             try
%                 fprintf(obj.hb, '%dr %dr\n', (round(volt*100/5)));
%             catch
%             end
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

            assert(isa(hc, 'natnet'), 'FastPursuit:Natnet_is_closed', ...
                'Natnet was not initialized or was closed')
                
            f = hc.getFrame();
            rbody = f.RigidBody(bidx);
            tracked = rbody.Tracked;

            if tracked
%                 q0 = [cosd(90/2), sind(90/2), 0, 0];
                q0 = dcm2quat(Rz(pi) * Rx(pi/2));
                quat = double([rbody.qw, rbody.qx, rbody.qy, rbody.qz]);
                quat = quatmultiply(quatmultiply(quatinv(q0), quat), (q0));
                trans = double([rbody.x, rbody.y, rbody.z]);
                trans = quatrotate(q0, trans);
                
%                 mocap2pursuit = @(pos) quatrotate(quatinv(xg(4:7)), pos) - xg(1:3);

                pos = [trans(1), trans(2)];
                rot = quat2angle((quat), 'ZYX');
            else
                pos = [0, 0];
                rot = [0];
            end
        end
    end
    
end

