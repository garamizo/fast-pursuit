classdef Map2D_fast < handle
    %UNTITLED4 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        obs
        lims
        
        gd
        gd_tight
        gd_safe
        kpos % keypoints
        cost
        obstacle_dilation = 0.01
    end
    
    methods
        
        function obj = Map2D_fast(varargin)
            for k = 1 : 2 : (nargin)
                switch varargin{k}
                    case properties(obj)
                        obj.(varargin{k}) = varargin{k+1};
                        
                    otherwise
                        warning(['Unknown input parameter: ' varargin{k}]);
                end
            end
            
            for k = 1 : length(obj.obs)
                nrows = size(obj.obs{k},1) * 2 + 1;
                obj.gd(1:nrows,k) = [size(obj.obs{k},1); obj.obs{k}(:,1); obj.obs{k}(:,2)];
            end
            obj.gd_tight = obj.gd; % save temporarily
            
            % create nodes
            [obj.kpos, obj.gd_safe] = gen_kpos(obj, obj.obstacle_dilation);
            
            % create cost
            obj.gd = obj.gd_safe;
            obj.cost = NaN(size(obj.kpos,1));
            for k = 1 : size(obj.cost, 1)
                for k2 = k + 1 : size(obj.cost, 1)
                    if ~collide(obj, obj.kpos(k,:), obj.kpos(k2,:))
                        obj.cost(k,k2) = sqrt(sum((obj.kpos(k,:) - obj.kpos(k2,:)).^2));
                    end
                end
            end
            
            obj.gd = obj.gd_tight;
        end
        
        
        function [keypos, gd2] = gen_kpos(map, tol)
            % keypos: key positions to travel map
            % gd2: new obstacle descriptions given tol
            
            keypos = [];
            gd2 = map.gd;
            kk = 1;
            for k = 1 : size(map.gd, 2)
                npts = map.gd(1,k);
                pos0 = [map.gd(npts,k), map.gd(npts+npts,k)];
                pos1 = [map.gd(2,k), map.gd(2+npts,k)];

                A = sum(map.gd(2:npts,k).*map.gd(3+npts:npts+npts+1,k) - map.gd(3:npts+1,k).*map.gd(2+npts:npts+npts,k));

                for e = 1 : npts-1
                    pos2 = [map.gd(e+2,k), map.gd(e+2+npts,k)];
                    
                    da = (pos1 - pos0);
                    db = (pos2 - pos1);
                    
                    da = da ./ sqrt(da * da.');
                    db = db ./ sqrt(db * db.');
                    
                    na = [da(2), -da(1)] * sign(A);
                    nb = [db(2), -db(1)] * sign(A);
                    
                    alf = [-da.', db.'] \ (pos0-pos1 + (na-nb)*tol).';
                    
                    p2 = pos1 + nb*tol + alf(2)*db;
                    p1 = pos1 - (nb*tol + alf(2)*db)/(1+1e-5);
                    p3 = pos1 + (nb*tol + alf(2)*db)/(1+1e-5);
                    
            % collide function ==================
            alf = [p2(2)-p1(2), p1(1)-p2(1), p1(1)*(p1(2)-p2(2)) - p1(2)*(p1(1)-p2(1))];
            bet = zeros(3,1);
            q1 = zeros(1,2);
            q2 = zeros(1,2);
            col = false;
            for k2 = 1 : size(map.gd, 2)
                if k2 == k
                    continue
                end
                nv = map.gd(1,k2);
                v = map.gd(:,k2);
                for e2 = 2 : nv
                    
                    q1(1) = v(e2);
                    q1(2) = v(e2+nv);
                    q2(1) = v(e2+1);
                    q2(2) = v(e2+1+nv);

                    % pre-computing was not faster
                    bet(1) = q2(2)-q1(2);
                    bet(2) = q1(1)-q2(1);
                    bet(3) = q1(1)*(q1(2)-q2(2)) - q1(2)*(q1(1)-q2(1));

                    if sign(alf(1) * q1(1) + alf(2) * q1(2) + alf(3)) ~= sign(alf(1) * q2(1) + alf(2) * q2(2) + alf(3)) && ...
                       sign(bet(1) * p1(1) + bet(2) * p1(2) + bet(3)) ~= sign(bet(1) * p2(1) + bet(2) * p2(2) + bet(3))
                        col = true;
                        break
                    end
                end
                if col
                    break
                end
            end
            % end collide =====================
                    
%                     keypos(kk,:) = p2;
%                     kk = kk + 1;
                    
                    if ~col && p2(1) > map.lims(1) && p2(1) < map.lims(2) && ...
                            p2(2) > map.lims(3) && p2(2) < map.lims(4)
                        keypos(kk,:) = p2;
                        kk = kk + 1;
                    end
                    gd2([e+1, e+1+npts],k) = p3;
                    
                    pos0 = pos1;
                    pos1 = pos2;
                end
                gd2(npts+1,k) = gd2(2,k);
                gd2(npts+npts+1,k) = gd2(npts+2,k);
            end
            
        end
        
        
        function col = collide(map, p1, p2)
        % map: {obs, lims}
        % pos1, pos2: [x, y] position at start and end
        % col: collision happened

            p1x = p1(1);
            p1y = p1(2);
            p2x = p2(1);
            p2y = p2(2);
            
            alf1 = p2y-p1y;
            alf2 = p1x-p2x;
            alf3 = p1x*(p1y-p2y) - p1y*(p1x-p2x);
            
            for k = 1 : size(map.gd, 2)
                nv = map.gd(1,k);
                v = map.gd(:,k);
                q1x = v(2);
                q1y = v(2+nv);
                for e = 3 : nv+1
                    
                    q2x = v(e);
                    q2y = v(e+nv);

                    % pre-computing was not faster
                    bet1 = q2y-q1y;
                    bet2 = q1x-q2x;
                    bet3 = q1x*(q1y-q2y) - q1y*(q1x-q2x);

                    if sign(alf1 * q1x + alf2 * q1y + alf3) ~= sign(alf1 * q2x + alf2 * q2y + alf3) && ...
                       sign(bet1 * p1x + bet2 * p1y + bet3) ~= sign(bet1 * p2x + bet2 * p2y + bet3) 
                        col = true;
                        return
                    end
                    
                    q1x = q2x;
                    q1y = q2y;
                end
            end
            col = false;
        end

        
        function plot(map)
            for k = 1 : length(map.obs)
%                 plot(map.obs{k}(:,1), map.obs{k}(:,2), '.-')
                h = fill(map.obs{k}(:,1), map.obs{k}(:,2), 'k');
                set(h, 'FaceAlpha', 0.2, 'LineStyle', 'none');
                hold on
            end
            
            dy = 5;
            dx = 5;
            yticks(map.lims(3):dy:map.lims(4))
            xticks(map.lims(1):dx:map.lims(2))
                
            xlim(map.lims(1:2)), ylim(map.lims(3:4))
            daspect([1 1 1])
            set(gcf, 'position', [560   620   413   328]) % for paper
%             set(gcf, 'position', [560   512   686   436]) % for video
            set(gcf, 'units', 'pixels')
%             set(gca,'TickDir','out');
%             set(gca,'box','off')
            
%             set(gca,'xtick',[])
%             set(gca,'ytick',[])
            
            set(gca,'xticklabel',[])
            set(gca,'yticklabel',[])
%             set(gca, 'units', 'pixels')

%             print -clipboard -dbitmap
        end
        
        
        function [tr, free] = grow_tree(map, pos)
            % TODO Compare to Dijkstra Algorithm
            
            fcost = triu(map.cost) + triu(map.cost)'; % full matrix with cost
            
            nnodes = size(map.kpos, 1);
            parent = -ones(nnodes, 1);
            cumcost = Inf(nnodes, 1);
            
            free = false;
            map.gd = map.gd_safe;
            for k = 1 : nnodes
                if ~collide(map, pos, map.kpos(k,:))
                    cumcost(k) = sqrt(sum((pos - map.kpos(k,:)).^2));
                    parent(k) = nnodes+1;
                    free = true;
                end
            end
            map.gd = map.gd_tight;
            
            if free == false % recover
                for k = 1 : nnodes
                    if ~collide(map, pos, map.kpos(k,:))
                        cumcost(k) = sqrt(sum((pos - map.kpos(k,:)).^2));
                        parent(k) = nnodes+1;
                        free = true;
                    end
                end
                if free == false
                    tr = struct('pos', [map.kpos; pos], 'cumcost', [cumcost; 0], 'parent', [parent; 0]);
                    return
                end
            end
            
            converge = false;
            while ~converge
                converge = true;
                for k1 = 1 : nnodes
                    for k2 = 1 : nnodes
                        if cumcost(k1) + fcost(k1,k2) < cumcost(k2) && k1 ~= k2
                            cumcost(k2) = cumcost(k1) + fcost(k1,k2);
                            parent(k2) = k1;
                            converge = false;
                        end
                    end
                end
            end

            tr = struct('pos', [map.kpos; pos], 'cumcost', [cumcost; 0], 'parent', [parent; 0]);
        end

        function plot_tree(map, tr, color)
            persistent pcount
            if isempty(pcount)
                pcount = 0;
            end
            pcount = pcount + 60;
%             spc = 0.5 * [cosd(pcount), sind(pcount)];
            
            if nargin < 3
                color = 'k';
            end
            txtloc = tr.pos(1,1:2);
            plot(map)
            hold on
%             plot(tr.pos(1,1), tr.pos(1,2), 'o', 'markersize', 20)
            scatter(tr.pos(end,1), tr.pos(end,2), 400, color, 'filled', 'MarkerFaceAlpha', 0.3)
            scatter(tr.pos(tr.parent>0,1), tr.pos(tr.parent>0,2), 5, color, 'o', 'filled')
            
            for k = 1 : size(tr.pos,1)
                if tr.parent(k) > 0
                    plot([tr.pos(k,1), tr.pos(tr.parent(k),1)], ...
                        [tr.pos(k,2), tr.pos(tr.parent(k),2)], ':', 'color', color)
                    
                    if all( sqrt(sum((txtloc - tr.pos(k,1:2)).^2, 2)) > 3) && ...
                            tr.pos(k,1) > map.lims(1) + 1 && tr.pos(k,1) < map.lims(2) - 1 && ...
                            tr.pos(k,2) > map.lims(3) + 1 && tr.pos(k,2) < map.lims(4) - 1
                        
                        pcount = -45;
                        spc = 0.8 * [cosd(pcount), sind(pcount)];
                        txtloc(end+1,:) = tr.pos(k,1:2);
%                         text(tr.pos(k,1)+spc(1), tr.pos(k,2)+spc(2), ...
%                             sprintf('%.1f', tr.cumcost(k)), 'FontSize', 11, 'Color', [0 0 0])
                        
                        txtloc2 = (tr.pos(k,:) + tr.pos(tr.parent(k),:))/2;
                        dist = sqrt(sum((tr.pos(k,:) - tr.pos(tr.parent(k),:)).^2));
%                         text(txtloc2(1), txtloc2(2), ...
%                             sprintf('%.1f', dist), 'FontSize', 10, 'Color', [0 0 0.7])
                   
                    end
                end
                text(tr.pos(k,1), tr.pos(k,2), ['  ' num2str(k)])
            end
        end
        
        function [mincost, kend, vend, wpos, v0] = find_path(map, tr, pos)
        % mincost: travel distance from tree root to pos
        % kend: index of key point closest to target
        % vend: approach speed
        % kk: array of indexes backtracing to target
        % v0: departure speed
            
            persistent kmin;
            if isempty(kmin)
                kmin = 1;
            end
            
            nnodes1 = size(tr.pos, 1);
            cdist = tr.cumcost + sqrt(sum((tr.pos - pos).^2, 2));
            
            if ~collide(map, tr.pos(kmin,:), pos)
                mincost = cdist(kmin);
            else
                mincost = Inf;
            end

            for k = 1 : size(tr.pos, 1)
                if cdist(k) < mincost && ~collide(map, tr.pos(k,:), pos)
                    mincost = cdist(k);
                    kmin = k;
                end
            end
            kend = kmin;
            
            if kend > 0
                vend = tr.pos(kend,:) - pos;
                if abs(vend) > 1e-5
                    vend = vend / sqrt(vend * vend');
                else
                    vend = [0, 1];
                end
            else
                vend = [0, 1];
            end
            
            if nargout > 3
                kk = kend;
                while kk(end) ~= nnodes1
                    kk(end+1) = tr.parent(kk(end));
                end
                kk = fliplr(kk);
                wpos = [tr.pos(kk,:); pos];
                v0 = wpos(2,:) - wpos(1,:);
                v0 = v0 / sqrt(v0 * v0');
            end
            
%             if kmin > 17
%                 error('???')
%             end
        end
        

    end
    
end

