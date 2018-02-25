classdef HolonomicController
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Waypoints
        DesiredLinearVelocity = 1
        MaxAngularVelocity = 10
        LookaheadDistance = 0.5
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
            obj.WaiPoints = interp1(s, wp, ss, 'linear', 'extrap');
        end
        
        function twist = step(obj, pose)
            % pose: robot pose in global frame
            % twist: desired robot twist in robot frame
            p = pose(1:2);
            q = pose(3);
            
            [~, idx] = min( sqrt(sum((obj.Waypoints - p).^2, 2)) );
            radi = obj.Waypoints(idx,:) - p;
            radi = radi / sqrt(radi * radi');
            try
                tang = mean(diff(obj.Waypoints(idx-1:idx+1,:)));
            catch ME
                throw(ME)
            end
            tang = tang / sqrt(tang * tang');
            v = radi * (1 - obj.LookaheadDistance) + tang * obj.LookaheadDistance;
            v = v * obj.DesiredLinearVelocity / sqrt(v * v');
            w = 0;
            twist = [v, w] * Rz(q);
        end
    end
    
end

