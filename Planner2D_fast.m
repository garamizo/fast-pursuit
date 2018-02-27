classdef Planner2D_fast < handle
    %UNTITLED14 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        p
        e
        m
        
        tp
        te
        
        gpos
        tg
        
        x0
        nx0 = 4
        
        tracks = repmat(struct('pos', [0, 0], 'cost', 0, 'count', 0), [0, 1])
        maxtracks = 5
        
        opt
        h
        etc
        
        problem
        ms
        
        dist_tol = 0.01 % length resolution
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
            
            obj.tg = grow_tree(obj.m, obj.gpos);
            obj.x0 = rand(15,2) .* (obj.m.lims([2 4]) - obj.m.lims([1 3])) + obj.m.lims([1 3]);
            
%             obj.opt = optimoptions(@fmincon,'Algorithm','interior-point', 'display', 'off');
%             obj.opt = optimoptions(obj.opt,'SpecifyObjectiveGradient',true,'SpecifyConstraintGradient',true, ...
%                 'MaxIterations', 100, 'MaxFunctionEvaluations', 500, 'StepTolerance', 0.1e-3, ...
%                 'TypicalX', [10 10], 'ObjectiveLimit', 0, 'OptimalityTolerance', 100e-3, ...
%                 'ConstraintTolerance', 100e-3, 'HessianFcn', @(x,lambda) hessianfcn(obj, x, lambda));
            
            obj.opt = optimoptions(@fmincon,'Algorithm','sqp', 'display', 'off');
            obj.opt = optimoptions(obj.opt,'SpecifyObjectiveGradient',true,'SpecifyConstraintGradient',true, ...
                'MaxIterations', 100, 'MaxFunctionEvaluations', 500, 'StepTolerance', 5e-3, ...
                'TypicalX', [10 10], 'ObjectiveLimit', 0, 'OptimalityTolerance', obj.dist_tol/10, ...
                'ConstraintTolerance', obj.dist_tol/10, 'HessianApproximation', 'lbfgs');
            
        end
        
  
        function [x, fval] = step(obj)

            % create trees
            [tr, free] = grow_tree(obj.m, obj.p.pos);
            if free
                obj.tp = tr;
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
            pos0 = [obj.x0; rand(N-size(obj.x0,1),2) .* (obj.m.lims([2 4]) - obj.m.lims([1 3])) + obj.m.lims([1 3])];
            nsols = 0;
            for k = 1 : N
                try
                    [x, fv, exitflag] = search_intercept(obj, pos0(k,:));
                    
                    assert(exitflag > 0, 'FastPursuit:No_solution', ...
                        'negative exitflag')

                    nsols = nsols + 1;
                    pos(nsols,:) = x;
                    fval(nsols,:) = fv;
                    sol_x0(nsols,:) = pos0(k,:);
                    
                catch ME
                    if ~strcmp(ME.identifier, 'FastPursuit:No_solution') && ...
                       ~strcmp(ME.identifier, 'optim:sqpInterface:UsrObjUndefAtX0')
                        rethrow(ME)
                    end
                end
            end
            
            obj.etc = struct('pos', pos, 'pos0', sol_x0);
            
            % remove duplicates
            % uidx points to the unique points
            [~, ~, uidx] = unique(round(pos / (5*obj.dist_tol)) * 5 *obj.dist_tol, 'rows');
            for k = 1 : max(uidx)
                fpos(k,:) = mean(pos(uidx==k,:), 1);
                ffval(k,1) = mean(fval(uidx==k), 1);
            end
            pos = fpos;
            fval = ffval;
            
            % find lost tracks
            costOfNonAssignment = 5;
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
            rows = [obj.tracks.count] <= 1;
            obj.tracks = obj.tracks(rows);
            for k = 1 : size(unassignedDetections,1)
                obj.tracks(end+1) = struct('pos', pos(unassignedDetections(k),:), ...
                    'cost', fval(unassignedDetections(k)), 'count', 0); 
            end
            
            % update initial guess for next step
            obj.x0 = cat(1, obj.tracks(:).pos);
            fval = cat(1, obj.tracks(:).cost);
            
            x = obj.x0;
        end
        
        function [f, gradf] = objfungrad(obj, x)
            [f, ~, gradf] = find_path(obj.m, obj.tg, x);
            gradf = -gradf;
        end

        function [c, ceq, DC, DCeq] = confungrad(obj, x)
            [cp, ~, vp] = find_path(obj.m, obj.tp, x);
            [ce, ~, ve] = find_path(obj.m, obj.te, x);

            ceq = [];
            c = -(cp/obj.p.vmax - ce/obj.e.vmax);
            DCeq = [];
            DC = (vp/obj.p.vmax - ve/obj.e.vmax)';
        end
        
        function [pos, fval, exitflag, count] = search_intercept(obj, pos0)

            [pos, fval, exitflag, output] = fmincon(@(x) objfungrad(obj,x), ...
                pos0,[],[],[],[],obj.m.lims([1 3]), obj.m.lims([2 4]), @(x)confungrad(obj,x), obj.opt);
            count = [output.funcCount, output.iterations];

%             assert( confungrad(obj, pos) < 1, 'FastPursuit:No_solution', ...
%                     'High constraint gradient' )
        end
        
        function [Hout, H, Hp, He] = hessianfcn(obj, x, lambda)
            x = x';
            
            [~, k] = find_path(obj.m, obj.tg, x);
            [~, kp] = find_path(obj.m, obj.tp, x);
            [~, ke] = find_path(obj.m, obj.te, x);

            r = x - obj.tg.pos(k,:); % (x0,y0) - (x,y)
            rn = sqrt(r * r'); % scalar r
            H = (eye(2)*rn^2 - r' * r) / rn^3;
            
            r = x - obj.tp.pos(kp,:); % (x0,y0) - (x,y)
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
                obj.h = plot([0, 0], [0, 0], '.-', 'color', [0.8 0.8 0.6], 'markersize', 12);
                obj.h(4) = scatter(zeros(2,1), zeros(2,1), [1e-5;1e-5], [1, 0.5, 0], '+', 'linewidth', 1.5);
                obj.h(2) = scatter(zeros(2,1), zeros(2,1), [1e-5;1e-5], [1, 0.5, 0], 'linewidth', 1.5);
                obj.h(3) = scatter(obj.gpos(1), obj.gpos(2), 400, [0, 0.8, 0], 'filled', 'MarkerFaceAlpha', 0.3);
                
                
                x = linspace(obj.m.lims(1), obj.m.lims(2), 20);
                y = linspace(obj.m.lims(3), obj.m.lims(4), 20);
                [xg, yg] = meshgrid(x, y);
                c = zeros(length(y), length(x));
                for k1 = 1 : length(x)
                    for k2 = 1 : length(y)
                        c(k2,k1) = find_path(obj.m, obj.tg, [x(k1), y(k2)]);
                    end
                end
                contour(xg, yg, c);
            end
            
            plot(obj.p)
            plot(obj.e)
%             set(obj.h(1), 'XData', obj.x0(:,1), 'YData', obj.x0(:,2))
            
            if ~isempty(obj.etc)
                pp = cat(3, obj.etc.pos, obj.etc.pos0, NaN(size(obj.etc.pos)));
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

