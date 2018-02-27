classdef Planner2D_fast < handle
    %UNTITLED14 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        p
        p2
        e
        m
        
        tp = struct()
        te
        
        gpos
        tg
        
        x0
        nx0 = 10
        
        tracks = repmat(struct('pos', [0, 0], 'cost', 0, 'count', 0), [0, 1])
        maxtracks = 5
        
        opt
        h
        etc
        
        problem
        ms
        
        dist_tol % length resolution
    end
    
    methods
        function obj = Planner2D_fast(varargin)
            
            
            for k = 1 : 2 : (nargin)
                switch varargin{k}
                    case properties(obj)
                        obj.(varargin{k}) = varargin{k+1};
                        
                    otherwise
                        warning(['Unknown input parameter: ' varargin{k}]);
                end
            end
            
            obj.dist_tol = min(diff(obj.m.lims(1:2)), diff(obj.m.lims(3:4))) / 20;
            
            obj.tg = grow_tree(obj.m, obj.gpos);
            obj.x0 = rand(obj.nx0,2) .* (obj.m.lims([2 4]) - obj.m.lims([1 3])) + obj.m.lims([1 3]);
            
            obj.tp = grow_tree(obj.m, obj.gpos);
            
%             obj.opt = optimoptions(@fmincon,'Algorithm','interior-point', 'display', 'off');
%             obj.opt = optimoptions(obj.opt, ...
%                 'SpecifyObjectiveGradient', true, ...
%                 'SpecifyConstraintGradient',true, ...
%                 'MaxIterations', 10, ...
%                 'MaxFunctionEvaluations', 50, ...
%                 'StepTolerance', obj.dist_tol / 100, ...
%                 'TypicalX', [diff(obj.m.lims(1:2)), diff(obj.m.lims(3:4))] / 2, ...
%                 'ObjectiveLimit', 0, ...
%                 'OptimalityTolerance', obj.dist_tol / 10, ...
%                 'ConstraintTolerance', obj.e.dt / 10, ...
%                 'HessianFcn', @(x,lambda) hessianfcn(obj, x, lambda));
            
            obj.opt = optimoptions(@fmincon,'Algorithm','sqp', 'display', 'off');
            obj.opt = optimoptions(obj.opt, ...
                        'SpecifyObjectiveGradient', true, ...
                        'SpecifyConstraintGradient', true, ...
                        'MaxIterations', 10, ...
                        'MaxFunctionEvaluations', 50, ...
                        'StepTolerance', obj.dist_tol / 100, ...
                        'TypicalX', [diff(obj.m.lims(1:2)), diff(obj.m.lims(3:4))] / 2, ...
                        'ObjectiveLimit', 0, ...
                        'OptimalityTolerance', obj.dist_tol / 10, ...
                        'ConstraintTolerance', obj.e.dt / 10, ...
                        'HessianApproximation', 'lbfgs');
            
        end
        
  
        function [x, fval] = step(obj)

            % create trees
            for k = 1 : length(obj.p)
                [tr, free] = grow_tree(obj.m, obj.p(k).pos);
                if free
                    obj.tp(k) = tr;
                end
            end
            
            [tr, free] = grow_tree(obj.m, obj.e.pos);
            if free || ~free
                obj.te = tr;
            end
            
            [tr, free] = grow_tree(obj.m, obj.gpos);
            if free
                obj.tg = tr;
            end
            
            % search N random points
            N = 5;
            pos0 = [obj.x0 + 0.01*randn(size(obj.x0,1), 2)
                rand(N-size(obj.x0,1),2) .* (obj.m.lims([2 4]) - obj.m.lims([1 3])) + obj.m.lims([1 3])];
            nsols = 0;
            for k = 1 : size(pos0, 1)
                try
                    [pos(nsols+1,:), fval(nsols+1,:)] = search_intercept(obj, pos0(k,:));
                    xout(k,:) = pos(nsols+1,:);
                    nsols = nsols + 1;
                    
                catch ME
                    if ~strcmp(ME.identifier, 'FastPursuit:No_solution') && ...
                       ~strcmp(ME.identifier, 'optimlib:sqpInterface:UsrObjUndefAtX0') && ...
                       ~strcmp(ME.identifier, 'optim:barrier:UsrObjUndefAtX0') && ...
                       ~strcmp(ME.identifier, 'optim:sqpInterface:UsrObjUndefAtX0')
                        rethrow(ME)
                    end
                    xout(k,:) = NaN(1, 2);
                end
            end
            
            if nsols > 0
                obj.etc = struct('pos', pos0, 'x', xout);

                % remove duplicates
                % uidx points to the unique points
                [~, ~, uidx] = unique(round(pos / (5*obj.dist_tol)) * 5 *obj.dist_tol, 'rows');
                for k = 1 : max(uidx)
                    fpos(k,:) = mean(pos(uidx==k,:), 1);
                    ffval(k,1) = mean(fval(uidx==k), 1);
                end
                pos = fpos;
                fval = ffval;
            else
                pos = zeros(0, 2);
                fval = zeros(0, 1);
            end
            
            % find lost tracks
            costOfNonAssignment = 2;
            costMatrix = zeros(length(obj.tracks), size(pos, 1));
            for k1 = 1 : length(obj.tracks)
                for k2 = 1 : size(pos, 1)
                    costMatrix(k1,k2) = sqrt(sum((obj.tracks(k1).pos - pos(k2,:)).^2));
                end
            end
            [assignments,unassignedTracks,unassignedDetections] = ...
                assignDetectionsToTracks( costMatrix,costOfNonAssignment);
            for k = 1 : size(assignments,1)
                obj.tracks(assignments(k,1)).pos = pos(assignments(k,2),:);
                obj.tracks(assignments(k,1)).cost = fval(assignments(k,2));
                obj.tracks(assignments(k,1)).count = 0;
            end
            for k = 1 : size(unassignedTracks,1)
                obj.tracks(unassignedTracks(k)).count = obj.tracks(unassignedTracks(k)).count + 1;
            end
            rows = [obj.tracks.count] <= 5;  % remove tracks gone for 5 loops
            obj.tracks = obj.tracks(rows);
            for k = 1 : size(unassignedDetections,1)
                obj.tracks(end+1) = struct('pos', pos(unassignedDetections(k),:), ...
                    'cost', fval(unassignedDetections(k)), 'count', 0); 
            end
            
            % assign best track for each pursuer
            if length(obj.tracks) > 1
                [~, iscost] = sort([obj.tracks.cost]);
                iscost((length(obj.p)+1):end) = [];
                for k1 = 1 : length(iscost) % loop track
                    for k2 = 1 : length(obj.p) % loop pursuer
                        costmat(k1,k2) = 2 * obj.tracks(iscost(k1)).cost + ...
                            find_path(obj.m, obj.tp(k2), obj.tracks(iscost(k1)).pos) / obj.p(k2).vmax;
                    end
                end
                asstracks = assignDetectionsToTracks(costmat, 1e5);
                asstracks = iscost(asstracks(:,1));
                asstracks(end+1:length(obj.p)) = iscost(1);
            else
                asstracks = ones(length(obj.p), 1);
            end
            
            % update initial guess for next step
            obj.x0 = cat(1, obj.tracks(:).pos);
            fval = cat(1, obj.tracks(:).cost);
            
%             asstracks
            x = cat(1, obj.tracks(asstracks).pos);
        end
        
        function [f, gradf] = objfungrad(obj, x)
            % path distance to target
            [f, ~, gradf] = find_path(obj.m, obj.tg, x);
            gradf = -gradf;
        end

        function [c, ceq, DC, DCeq, idx] = confungrad(obj, x)
            % (path distance from 
            [ce, ~, ve] = find_path(obj.m, obj.te, x);
            
            cpnorm = Inf;
            vpnorm = [1, 0];
            idx = 1;
            for k = 1 : length(obj.tp)
                [cpp, ~, vpp] = find_path(obj.m, obj.tp(k), x);
                
                if cpp/obj.p(k).vmax < cpnorm
                    cpnorm = cpp / obj.p(k).vmax;
                    vpnorm = vpp / obj.p(k).vmax;
                    idx = k;
                end
            end
            
            ceq = [];
            c = ce/obj.e.vmax - cpnorm;
            DCeq = [];
            DC = -(ve/obj.e.vmax - vpnorm)';
        end
        
        function [pos, fval, exitflag, count] = search_intercept(obj, pos0)

            [pos, fval, exitflag, output] = fmincon(@(x) objfungrad(obj,x), ...
                pos0,[],[],[],[],obj.m.lims([1 3]), obj.m.lims([2 4]), @(x)confungrad(obj,x), obj.opt);
            count = [output.funcCount, output.iterations];
            
            assert(exitflag > 0, 'FastPursuit:No_solution', ...
                        'negative exitflag')
        end
        
        function [Hout, H, Hp, He] = hessianfcn(obj, x, lambda)
            x = x';
            
            [~, k] = find_path(obj.m, obj.tg, x);
            [~, ke] = find_path(obj.m, obj.te, x);

            r = x - obj.tg.pos(k,:); % (x0,y0) - (x,y)
            rn = sqrt(r * r'); % scalar r
            H = (eye(2)*rn^2 - r' * r) / rn^3;
            
            imin = 1;  % pursuer closest to x
            cmin = Inf;  % minimum cost for pursuer imin
            kpi = 1;
            for k = 1 : length(obj.p)
                [c, kp] = find_path(obj.m, obj.tp, x);
                if c < cmin
                    imin = k;
                    cmin = c;
                    kpi = kp;
                end
            end
                
            r = x - obj.tp(imin).pos(kpi,:); % (x0,y0) - (x,y)
            rn = sqrt(r * r'); % scalar r
            Hp = (eye(2)*rn^2 - r' * r) / rn^3;
            
            r = x - obj.te.pos(ke,:); % (x0,y0) - (x,y)
            rn = sqrt(r * r'); % scalar r
            He = (eye(2)*rn^2 - r' * r) / rn^3;

            Hout = (H - lambda.ineqnonlin*(Hp - He));
%             lambda
        end
            
        function plot(obj)
            if isempty(obj.h) || ~isvalid(obj.h(1))
                plot(obj.m)
                
                
%                 obj.h = plot(obj.x0(:,1), obj.x0(:,2), 'xr');
                obj.h = plot([0, 0], [0, 0], '.-', 'color', [0.8 0.8 0.6], 'markersize', 12);  % x0 -> x
                obj.h(4) = scatter(zeros(2,1), zeros(2,1), [1e-5;1e-5], [1, 0.5, 0], '+', 'linewidth', 1.5);
                obj.h(2) = scatter(zeros(2,1), zeros(2,1), [1e-5;1e-5], [1, 0.5, 0], 'linewidth', 1.5);
                obj.h(3) = scatter(obj.gpos(1), obj.gpos(2), 400, [0, 0.8, 0], 'filled', 'MarkerFaceAlpha', 0.3);
                
                ds = min(diff(obj.m.lims(1:2)), diff(obj.m.lims(3:4))) / 50;
                x = obj.m.lims(1) : ds : obj.m.lims(2);
                y = obj.m.lims(3) : ds : obj.m.lims(4);
                [xg, yg] = meshgrid(x, y);
                c = zeros(length(y), length(x));
                for k1 = 1 : length(x)
                    for k2 = 1 : length(y)
                        c(k2,k1) = find_path(obj.m, obj.tg, [x(k1), y(k2)]);
                    end
                end
                contour(xg, yg, c);
            end
            
            for k = 1 : length(obj.p)
            plot(obj.p(k))
            end
            plot(obj.e)
%             set(obj.h(1), 'XData', obj.x0(:,1), 'YData', obj.x0(:,2))
            
            if ~isempty(obj.etc)
                % plot initial guess joined to local solution
                pp = cat(3, obj.etc.pos, obj.etc.x, NaN(size(obj.etc.pos)));
                pp = reshape(permute(pp, [3 1 2]), [3*size(obj.etc.pos,1), 2, 1]);
                set(obj.h(1), 'XData', pp(:,1), 'YData', pp(:,2))
            end
            
            
            sol = cat(1, obj.tracks(:).pos);
            fval = cat(1, obj.tracks(:).cost);
            if ~isempty(sol)
                set(obj.h(2), 'XData', sol(:,1), 'YData', sol(:,2), 'SizeData', 0.2*(150 + 1500*mean(fval)./(1+fval)))
                [~,idx] = min(fval);
                set(obj.h(4), 'XData', sol(idx,1), 'YData', sol(idx,2), 'SizeData', 0.2*(150 + 1500*mean(fval(idx))./(1+fval(idx))))
            end
            set(obj.h(3), 'XData', obj.gpos(1), 'YData', obj.gpos(2));
            
        end
        
    end
    
end

