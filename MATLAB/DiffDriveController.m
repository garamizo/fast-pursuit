classdef DiffDriveController < handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Waypoints
        DesiredLinearVelocity = 0.1
        MaxAngularVelocity = 6
        MaxLinearVelocity = 0.7
        K = 4
        lookAhead = 0.3
        Tolerance = 0.2
        
        path
        pathd
        h
        
        
    end
    
    methods
        
        function obj = HolonomicController(varargin)
            % assign custom options
            for k = 1 : 2 : (nargin)
                switch varargin{k}
                    case properties(obj)
                        obj.(varargin{k}) = varargin{k+1};
                        
                    otherwise
                        warning(['Unknown input parameter: ' varargin{k}]);
                end
            end
        end
        
        function set(obj, wp)
            s = [0; cumsum(sqrt(sum(diff(wp).^2, 2)))];
            ss = linspace(s(1), s(end), 100)';
            obj.Waypoints = wp;
            obj.path = interp1(s, wp, ss, 'pchip', 'extrap');
            obj.pathd = [   obj.path(2,:) - obj.path(1,:)
                            obj.path(3:end,:) - obj.path(1:end-2,:)
                            obj.path(end,:) - obj.path(end-1,:)     ];
            obj.pathd = obj.pathd ./ sqrt(sum(obj.pathd.^2, 2));
        end
        
        function twist = step(obj, pose)
            % pose: robot pose in global frame
            % twist: desired robot twist in robot frame
            p = pose(1:2);
            q = pose(3);
            
            if size(obj.path,1) == 0 || sqrt(sum((p - obj.path(end,:)).^2)) < obj.Tolerance
                twist = [0, 0, 0];
                return
            end
            
            [~, idx] = min( sqrt(sum((obj.path - p).^2, 2)) );
            heading = obj.pathd(idx,:) * Agent2D.Rz(q);
            pRP = (obj.path(idx,:) - p) * Agent2D.Rz(q);
            vdir = obj.lookAhead * heading + (1 - obj.lookAhead) * pRP;
            
            v = [obj.DesiredLinearVelocity, 0];
            w = max(min(obj.K * atan2(vdir(2), vdir(1)), obj.MaxAngularVelocity), -obj.MaxAngularVelocity);
            twist = [v, w];
            
%             obj.motorization(twist)
        end
        
        function volt = motorization(obj, twist)
            dvolt = twist(3) * 200 / obj.MaxAngularVelocity;
            svolt = twist(1) * 200 / obj.MaxLinearVelocity;
            
            if abs(dvolt) + svolt > 200
                svolt = 200 - abs(dvolt);
            end
            volt = [svolt/2 + dvolt/2, svolt/2 - dvolt/2];
        end
        
        function plot(obj)
            try
                set(obj.h(1), 'XData', obj.Waypoints(:,1), 'YData', obj.Waypoints(:,2))
                set(obj.h(2), 'XData', obj.path(:,1), 'YData', obj.path(:,2))
            catch
                obj.h(1) = plot(obj.Waypoints(:,1), obj.Waypoints(:,2), 'o');
                obj.h(2) = plot(obj.path(:,1), obj.path(:,2), ':');
            end
            
        end
    end
    
end

