classdef HolonomicController < handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Waypoints
        DesiredLinearVelocity = 1
        MaxAngularVelocity = 10
        LookaheadDistance = 0.9
        Tolerance = 0.1
        
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
            obj.path = interp1(s, wp, ss, 'linear', 'extrap');
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
            radi = obj.path(idx,:) - p;
            if radi * radi' > 1e-5
                radi = radi / sqrt(radi * radi');
            end
            tang = obj.pathd(idx,:);
            
            v = radi * (1 - obj.LookaheadDistance) + tang * obj.LookaheadDistance;
            v = v * obj.DesiredLinearVelocity / sqrt(v * v');
            w = 0;
            twist = [v, w] * Rz(q);
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

