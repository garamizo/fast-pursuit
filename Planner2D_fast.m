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
%             obj.x0 = rand(30,2) .* (obj.m.lims([2 4]) - obj.m.lims([1 3])) + obj.m.lims([1 3]);
            
%             obj.opt = optimoptions(@fmincon,'Algorithm','interior-point', 'display', 'off');
%             obj.opt = optimoptions(obj.opt,'SpecifyObjectiveGradient',true,'SpecifyConstraintGradient',true, ...
%                 'MaxIterations', 100, 'MaxFunctionEvaluations', 500, 'StepTolerance', 0.1e-3, ...
%                 'TypicalX', [10 10], 'ObjectiveLimit', 0, 'OptimalityTolerance', 100e-3, ...
%                 'ConstraintTolerance', 100e-3, 'HessianFcn', @(x,lambda) hessianfcn(obj, x, lambda));
            
            obj.opt = optimoptions(@fmincon,'Algorithm','sqp', 'display', 'off');
            obj.opt = optimoptions(obj.opt,'SpecifyObjectiveGradient',true,'SpecifyConstraintGradient',true, ...
                'MaxIterations', 100, 'MaxFunctionEvaluations', 500, 'StepTolerance', 5e-3, ...
                'TypicalX', [10 10], 'ObjectiveLimit', 0, 'OptimalityTolerance', 10e-3, ...
                'ConstraintTolerance', 1000e-3, 'HessianApproximation', 'lbfgs');
            
        end
        
        function [trout, free] = grow_trim_tree(obj, trp, pos)
            nnodes = size(obj.m.kpos, 1);
            parent = -ones(nnodes,1);
            
            % compute cost
            converge = false;
            last_avg_cost = NaN;
            cumcost = sqrt(sum((pos - obj.m.kpos).^2, 2));
            
            cumcostp = trp.cumcost * obj.e.vmax / obj.p.vmax;
            
            free = false;
            obj.m.gd = obj.m.gd_safe;
            for k = 1 : nnodes
                if cumcostp(k) > cumcost(k) && ~collide(obj.m, pos, obj.m.kpos(k,:))

                    if trp.parent(k) == nnodes+1
                        vp = trp.pos(k,:) - trp.pos(nnodes+1,:);
                        ve = trp.pos(k,:) - pos;

                        if acosd(vp * ve' / sqrt(vp*vp' * (ve*ve'))) < 30
                            cumcost(k) = Inf;
                            continue
                        end
                    end

                    parent(k) = nnodes+1;
                    free = true;
                else
                    cumcost(k) = Inf;
                end
            end
            obj.m.gd = obj.m.gd_tight;
            
            if free == false
                trout = struct('pos', [obj.m.kpos; pos], 'cumcost', [Inf(nnodes,1); 0], 'parent', [zeros(nnodes,1); 0]);
                return
            end
            
            while ~converge
                for k1 = 1 : nnodes
                    for k2 = (k1+1) : nnodes-1
                        ncost = cumcost(k1) + obj.m.cost(k1,k2);
                        if ncost < cumcost(k2) && cumcostp(k2) > ncost
                            
                            if cumcostp(trp.parent(k2)) < cumcost(k2)
                                vp = trp.pos(k2,:) - trp.pos(trp.parent(k2),:);
                                ve = trp.pos(k2,:) - trp.pos(k1,:);
                            
                                if acosd(vp * ve' / sqrt(vp*vp' * (ve*ve'))) < 30
                                    continue
                                end
                            end
                            cumcost(k2) = ncost;
                            parent(k2) = k1;

                        else
                            ncost = cumcost(k2) + obj.m.cost(k1,k2);
                            if ncost < cumcost(k1) && cumcostp(k1) > ncost
                                
                                if cumcostp(trp.parent(k1)) < cumcost(k1)
                                    vp = trp.pos(k1,:) - trp.pos(trp.parent(k1),:);
                                    ve = trp.pos(k1,:) - trp.pos(k2,:);
                                
                                    if acosd(vp * ve' / sqrt(vp*vp' * (ve*ve'))) < 30
                                        continue
                                    end
                                end
                                cumcost(k1) = ncost;
                                parent(k1) = k2;
                            end
                        end
                    end
                end
                converge = last_avg_cost == mean(cumcost(~isinf(cumcost)));
                last_avg_cost = mean(cumcost(~isinf(cumcost)));
            end

            trout = struct('pos', [obj.m.kpos; pos], 'cumcost', [cumcost; 0], 'parent', [parent; 0]);
        end
        
        function [x, fval] = step(obj)

            % create trees
            [tr, free] = grow_tree(obj.m, obj.p.pos);
            if free
                obj.tp = tr;
            else
                tr
            end
            
%             [tr, free] = grow_tree(obj.m, obj.e.pos);
            [tr, free] = grow_trim_tree(obj, obj.tp, obj.e.pos);
            if free || ~free
                obj.te = tr;
            end
            
            [tr, free] = grow_tree(obj.m, obj.gpos);
            if free
                obj.tg = tr;
            end
            
            ppdist = 2; % pursuer-evader distance to engage pure pursuit
            if sqrt(sum((obj.p.pos - obj.e.pos).^2)) < ppdist && ~collide(obj.m, obj.p.pos, obj.e.pos)
                fprintf('pure pursuit!\n')
                pos = obj.e.pos;
                fval = 0;
                exitflag = 1;
                pos0 = [NaN, NaN];
            
%             elseif find_path(obj.m, obj.tp, obj.gpos)/obj.p.vmax > find_path(obj.m, obj.te, obj.gpos)/obj.e.vmax
%                 fprintf('run home!\n')
%                 pos = obj.gpos;
%                 fval = 0;
%                 exitflag = 1;
%                 pos0 = [NaN, NaN];
                
            else
                % search N random points
                N = 5;
                pos0 = [obj.x0; rand(N-size(obj.x0,1),2) .* (obj.m.lims([2 4]) - obj.m.lims([1 3])) + obj.m.lims([1 3])];
                pos = zeros(size(pos0));
                fval = zeros(size(pos0,1), 1);
                exitflag = zeros(size(pos0,1), 1);
                for k = 1 : size(pos0,1)
                    [pos(k,:), fval(k), exitflag(k)] = search_intercept(obj, pos0(k,:));
                end
            end
            
            pos(exitflag<=0,:) = NaN;
            obj.etc = struct('pos', pos, 'pos0', pos0);
            
            % remove duplicates
            pos = pos(exitflag>0,:);
            fval = fval(exitflag>0);
            parent = zeros(size(pos,1),1);
            for k1 = 1 : size(pos,1)
                for k2 = k1+1 : size(pos,1)
                    dist = sqrt(sum((pos(k1,:) - pos(k2,:)).^2,2));
                    if dist < 1
                        if fval(k1) < fval(k2)
                            parent(k2) = k1;
                        else
                            parent(k1) = k2;
                        end
                    end
                end
            end
            pos = pos(parent==0,:);
            fval = fval(parent==0);
            
            % find lost tracks
            costOfNonAssignment = 7;
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
            rows = [obj.tracks.count] <= 2;
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
            
%             if obj.e.vmax > obj.p.vmax && vp * ve' > 0
%                 c = NaN;
%             end

%             c = [];
%             ceq = (cp - ce);
%             DC = [];
%             DCeq = -(vp - ve)';
        end
        
        function [pos, fval, exitflag, count] = search_intercept(obj, pos0)

            if isinf( find_path(obj.m, obj.te, pos0) ) || isinf( find_path(obj.m, obj.tg, pos0) ) || ...
                    isnan(confungrad(obj, pos0))
                pos = pos0;
                exitflag = 0;
                fval = Inf;
                count = [0, 0];
                return
            end
            
%             [mincost, kend, vend] = find_path(obj.m, obj.tg, pos0)
%             [f, gradf] = objfungrad(obj, pos0)
%             [c, ceq, DC, DCeq] = confungrad(obj, pos0)
            
            [pos, fval, exitflag, output] = fmincon(@(x) objfungrad(obj,x), pos0,[],[],[],[],obj.m.lims([1 3]), obj.m.lims([2 4]), @(x)confungrad(obj,x), obj.opt);
            count = [output.funcCount, output.iterations];
            
            if abs(confungrad(obj, pos)) > 1
                exitflag = 0;
            
            elseif obj.e.vmax >= obj.p.vmax
                [~, ~, vp] = find_path(obj.m, obj.tp, pos);
                [~, ~, ve] = find_path(obj.m, obj.te, pos);
                if acosd(vp * ve') < 30
                    exitflag = 0;
                end
            end
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
            end
            
            plot(obj.p)
            plot(obj.e)
%             set(obj.h(1), 'XData', obj.x0(:,1), 'YData', obj.x0(:,2))
            
            pp = cat(3, obj.etc.pos, obj.etc.pos0, NaN(size(obj.etc.pos)));
            pp = reshape(permute(pp, [3 1 2]), [3*size(obj.etc.pos,1), 2, 1]);
            set(obj.h(1), 'XData', pp(:,1), 'YData', pp(:,2))
            
            
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

