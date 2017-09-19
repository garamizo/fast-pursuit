%% import map from pdetool
% export geometry description

valid = all(gd(1,:) == 2 | gd(1,:) == 3);
if ~valid
    error('Only rectangles or polygons are allowed')
end
ds = 10e-2;

output = cell(1, size(gd, 2));
for k = 1 : size(gd, 2)
    npts = gd(2,k);
    output{k} = [gd(2+(1:npts),k), gd(2+npts+(1:npts),k)
        gd(2+1,k), gd(2+npts+1,k)];
    output{k} = round(output{k} / ds) * ds + randn(size(output{k})) * 0;
end

x_constraints = [-15, 15];
y_constraints = [-10, 10];
num = size(gd, 2);

mapfile = 'maze2';
save(fullfile('maps', mapfile), 'output', 'x_constraints', 'y_constraints', 'num')

%% load map

% mapfile = 'sample_block';
mapfile = 'maze';
sdata = load(fullfile('maps', mapfile));
map = Map2D_fast('obs', sdata.output, 'lims', [sdata.x_constraints, sdata.y_constraints]);

% plot(map)

%% test speed of collide function
% 0.0125 ms for maze
% 0.0059 ms for simple

pos1 = rand(10000,2) .* (map.lims([2 4]) - map.lims([1 3])) + map.lims([1 3]);
pos2 = rand(10000,2) .* (map.lims([2 4]) - map.lims([1 3])) + map.lims([1 3]);
col = false([size(pos1,1), 1]);

map.gd = map.gd_safe;
tic
for k = 1 : size(pos1, 1)
    col(k) = collide(map, pos1(k,:), pos2(k,:));
end
toc * 1000 / k
map.gd = map.gd_tight;

% plot(map)

%% test speed of grow_tree function
% 0.55 ms for maze
% 0.5 ms for simple

pos = pos1(find(~col,500),:);
clear c1 c2
tic
for k = 1 : size(pos,1)
    tr = grow_tree(map, pos(k,:));
end
toc * 1000 / k

figure
plot_tree(map, tr)


%% trimmed tree growth
% 2.7 ms for maze

dt = 0.1;
p = Agent2D('pos', [0.9868   -4.7368], 'vmax', 2, 'dt', dt, 'color', [0.3 0.3 1], ...
    'yaw', 2, 'shape', [10, -7.5; 13, 0; 10, 7.5; -10, 7.5; -10, -7.5; 10, -7.5] * 7e-2);
e = Agent2D('pos', [-8.2613    1.4098], 'vmax', 1, 'dt', dt, 'color', [1 0.3 0.3], ...
    'yaw', 0.5, 'shape', [10, -7.5; 13, 0; 10, 7.5; -10, 7.5; -10, -7.5; 10, -7.5] * 7e-2);

plot(map)
scatter(p.pos(end,1), p.pos(end,2), 400, [0.3 0.3 1], 'filled', 'MarkerFaceAlpha', 0.3)
scatter(e.pos(end,1), e.pos(end,2), 400, [1 0.3 0.3], 'filled', 'MarkerFaceAlpha', 0.3)
scatter(0.8177, -1.1842, 400, [0, 0.8, 0], 'filled', 'MarkerFaceAlpha', 0.3)

pl = Planner2D_fast('p', p, 'e', e, 'm', map, 'gpos', [0.8177   -1.1842]);
step(pl)

%%
plot(map)
scatter(p.pos(end,1), p.pos(end,2), 400, [0.3 0.3 1], 'filled', 'MarkerFaceAlpha', 0.3)
scatter(e.pos(end,1), e.pos(end,2), 400, [1 0.3 0.3], 'filled', 'MarkerFaceAlpha', 0.3)
scatter(pl.gpos(1), pl.gpos(2), 400, [0, 0.8, 0], 'filled', 'MarkerFaceAlpha', 0.3)


pl.tp = grow_tree(map, p.pos);

% plot(pl)
% te = grow_tree(map, e.pos);

%%
plot(map)
scatter(p.pos(end,1), p.pos(end,2), 400, [0.3 0.3 1], 'filled', 'MarkerFaceAlpha', 0.3)
scatter(e.pos(end,1), e.pos(end,2), 400, [1 0.3 0.3], 'filled', 'MarkerFaceAlpha', 0.3)
scatter(pl.gpos(1), pl.gpos(2), 400, [0, 0.8, 0], 'filled', 'MarkerFaceAlpha', 0.3)


trout = grow_trim_tree(pl, pl.tp, e.pos);
% trout2 = grow_trim_tree(pl, te, p.pos);



%%
pos = pos1(find(~col,100),:);
clear c1 c2
tic
for k = 1 : size(pos,1)
    tr = grow_trim_tree(pl, tp, pos(k,:));
end
fprintf('Execution time: %.2f ms\n', toc * 1000 / k)

tp.cumcost = tp.cumcost / p.vmax;
te.cumcost = te.cumcost / e.vmax;
trout.cumcost = trout.cumcost / e.vmax;

% close all
figure, plot_tree(map, tp, [0 0 0.8])
% figure, plot_tree(map, te, [0.8 0 0])
figure, plot_tree(map, trout, [0.8 0 0])
% hold on, plot_tree(map, trout2, [0 0 0.8])


%% test speed of find_path
% 0.18 (before 0.15) ms for maze
% 0.03 ms for simple

% tr = grow_tree(map, [1.5553    1.3411]);

pos = pos1(find(~col,5000),:);
tic
for k = 1 : size(pos,1)
    [cost, path, v] = find_path(map, tr, pos(k,:));
end
toc * 1000 / k

% figure
% plot_tree(map, tr)
% quiver(pos(k,1), pos(k,2), v(1), v(2), 'linewidth', 2)

%% test speed of intercept search
% 12 ms for maze

mapfile = 'maze';
sdata = load(fullfile('maps', mapfile));
map = Map2D_fast('obs', sdata.output, 'lims', [sdata.x_constraints, sdata.y_constraints]);

dt = 0.1;
p = Agent2D('pos', [-3, -1], 'yaw', -1.7, 'vmax', 1, 'dt', dt, 'color', [0.3 0.3 1], ...
    'shape', [10, -7.5; 13, 0; 10, 7.5; -10, 7.5; -10, -7.5; 10, -7.5] * 7e-2);
e = Agent2D('pos', [0, -8], 'yaw', 3, 'vmax', 2, 'dt', dt, 'color', [1 0.3 0.3], ...
    'shape', [10, -7.5; 13, 0; 10, 7.5; -10, 7.5; -10, -7.5; 10, -7.5] * 7e-2);

k = 30;
lims = [-7.5609    9.3837, -9.3077    1.9886];
x0 = rand(100,2).*[map.lims(2)-map.lims(1), map.lims(4)-map.lims(3)] + map.lims([1 3]);

pl = Planner2D_fast('p', p, 'e', e, 'm', map, 'gpos', [1.4862   -0.5248], 'x0', x0(:,:));

tic
step(pl)
fprintf('Execution time: %.2f ms\n', toc * 1000 / size(x0,1))

figure, plot(pl)


%%
med = (p.pos + e.pos)/2;
tn = p.pos - e.pos;
tn = tn / sqrt(tn*tn');
tn = [tn(2), -tn(1)];

lin = med + tn.*[-10; 30];

figure, plot(pl)
plot(lin(:,1), lin(:,2), '--', 'linewidth', 1.5)
xlim(lims(1:2))
ylim(lims(3:4))

[xgrid, ygrid] = meshgrid(lims(1):0.1:lims(2), lims(3):0.1:lims(4));
dist = sqrt((xgrid-pl.gpos(1)).^2 + (ygrid-pl.gpos(2)).^2);

hold on
contour(lims(1):0.1:lims(2), lims(3):0.1:lims(4), dist, 2.5:2.5:50, 'showtext', 'off')

set(gca,'Visible','off')

%% test speed of pursuit

mapfile = 'maze';
sdata = load(fullfile('maps', mapfile));
map = Map2D_fast('obs', sdata.output, 'lims', [sdata.x_constraints, sdata.y_constraints]);


dt = 0.1;
p = Agent2D('pos', [-3.7031   -0.8438], 'yaw', 0, 'vmax', 6, 'dt', dt, 'color', [0.3 0.3 1], ...
    'shape', [10, -7.5; 13, 0; 10, 7.5; -10, 7.5; -10, -7.5; 10, -7.5] * 7e-2);
e = Agent2D('pos', [9.7031   -6.5625], 'yaw', 0, 'vmax', 3, 'dt', dt, 'color', [1 0.3 0.3], ...
    'shape', [10, -7.5; 13, 0; 10, 7.5; -10, 7.5; -10, -7.5; 10, -7.5] * 7e-2);

pl = Planner2D_fast('p', p, 'e', e, 'm', map, 'gpos', [1.4862   -0.5248]);
pl.tp = grow_tree(map, p.pos);
pl.te = grow_trim_tree(pl, pl.tp, e.pos);

% [~, ~, ~, path] = find_path(map, pl.te, pl.gpos);
% path = [e.pos; pp; pl.gpos];
% set_plan(e, path, 90);
% pts = [
%     9.7811   -5.5977
%     9.0207   -1.4577
%     4.3203   -7.6968
%    -2.3848   -9.0379
%    -7.9147   -8.9796
%   -11.8548   -7.2886
%   -11.9240   -6.0641
%   -10.3341   -0.6997
%    -6.6014    2.2741
%    -1.6935    2.3907
%    -0.1037   -0.6414
%     0.8641   -0.5831 ];
pts = [
    8.9531   -0.6563
    3.7969   -7.4063
   -2.5781   -7.5000
   -3.1406   -7.1250
   -3.1406   -6.2813
    1.5469   -4.7813
    1.6406   -0.8438 ];
set_plan(e, [e.pos; pts], 100);

% VIP position
pts = [
   -1.3594   -2.1563
   -3.1406   -5.2500
   -2.9531   -3.0938
   -1.3594    0.4687
   -0.9844    2.4375
    4.9219    3.0000
    9.5156    3.0000
   11.2031    6.2812 ];
s = [0; cumsum(sqrt(sum(diff(pts).^2,2)))];
ss = 0 : 1 : s(end);
GPOS = interp1(s, pts, ss);
K = linspace(0, 500, size(GPOS,1));

data = zeros(3000,6);

% tt = grow_tree(map, p.pos)
% figure, plot_tree(map, tt)

record = false;
if record
    v = VideoWriter('videos/pursuit_x3', 'Motion JPEG AVI');
    v.Quality  = 100;
    decim = 1;
    v.FrameRate = decim/dt;
    open(v)
end

% initial assessment
[x, fval] = step(pl);
[~, idx] = min(fval);
[~, ~, ~, path] = find_path(map, pl.tp, x(idx,:));
pplan = [p.pos; path];

plot(map), hold on
hh = plot(0, 0, '^');
hold on

% close all
time = zeros(3000);
t0 = tic;
for k = 1 : 3000
    tic

    ppix = get(0, 'PointerLocation'); % pointer absolute location
    mpix = get(gcf, 'position').*[1 1 0 0] + get(gca, 'position');
    ppos = [interp1([mpix(1), mpix(1)+mpix(3)], map.lims(1:2), ppix(1), 'linear', 'extrap'), ...
        interp1([mpix(2), mpix(2)+mpix(4)], map.lims(3:4), ppix(2), 'linear', 'extrap')];
    set(hh, 'XData', ppos(1), 'YData', ppos(2))
    set_plan(e, [e.pos; ppos], 100);
    
    if sqrt(sum((e.pos - p.pos).^2)) < 2
        set_plan(p, [p.pos; e.pos], 100);
        xpur = e.pos;
        
    elseif mod(k-1, 1) == 0
        
        set_plan(p, pplan, 100);
        xpur = pplan(end,:);
        plot(pl)
        
        % start calculating new plan
        [x, fval] = step(pl);
        if ~isempty(x)
            [~, idx] = min(fval);
            [~, ~, ~, path] = find_path(map, pl.tp, x(idx,:));
            pplan = [p.pos; path];
        end
    end
    
%     pl.gpos = interp1(K, GPOS, 1+0*min(k,K(end)), 'linear');

    p0 = step(p);
    if collide(map, p0, p.pos)
%         fprintf('collision!\n')
        p.pos = p0;
%         break
    end
    
    p0 = step(e);
    if collide(map, p0, e.pos)
        e.pos = p0;
%         fprintf('collision!\n')
%         break
    end
    
    plot(p)
    plot(e)
    
    if sqrt(sum((e.pos - pl.gpos).^2)) < 1 || sqrt(sum((e.pos - p.pos).^2)) < 1
        break
    end
    data(k,:) = [p.pos, e.pos, xpur];
    
    drawnow
    pause(dt- toc)
    time(k) = toc(t0);
    
    if record
        frame = getframe(gca);
        writeVideo(v, frame);
    end
end
fprintf('Execution time: %.1f ms\n', toc * 1000 / k)

if record
    close(v)
end

% hold off
% plot(diff(time(1:k-1)))

%% time stamped, not delayed
pl.tracks = repmat(struct(pl.tracks), 1, 0);

plot(pl)
plot(data(1:k-1,1), data(1:k-1,2), 'color', [0.3 0.3 1])
plot(data(1:k-1,3), data(1:k-1,4), 'color', [1 0.3 0.3])
rows = [1 31 34];
plot(data(rows,1), data(rows,2), 'o', 'markersize', 3, 'color', [0.3 0.3 1], 'MarkerFaceColor', [0.3 0.3 1])
plot(data(rows,3), data(rows,4), 'o', 'markersize', 3, 'color', [1 0.3 0.3], 'MarkerFaceColor', [1 0.3 0.3])
plot(data(rows,5), data(rows,6), 'o', 'markersize', 20, 'color', [1, 0.5, 0], 'linewidth', 1.5)

txt = {'t_1', 't_2', 't_3'};
text(data(rows,1)+0.5, data(rows,2)-0.5, txt)
text(data(rows,3)-2, data(rows,4)+1, txt)

% rows = [31 34 51];
% txt = {'t_1 - t_2', 't_3', 't_4'};
% text(data(rows,5)+1.2, data(rows,6)+1, txt)

rows = [31 34];
txt = {'A (t_1 - t_2)', 'B (t_3)'};
text(data(rows,5)-1.8, data(rows,6)-1.2, txt)

%% time stamped, delayed

plot(pl)
plot(data(1:k-1,1), data(1:k-1,2), 'color', [0.3 0.3 1])
plot(data(1:k-1,3), data(1:k-1,4), 'color', [1 0.3 0.3])
rows = [1 31 34 51];
plot(data(rows,1), data(rows,2), 'o', 'markersize', 3, 'color', [0.3 0.3 1], 'MarkerFaceColor', [0.3 0.3 1])
plot(data(rows,3), data(rows,4), 'o', 'markersize', 3, 'color', [1 0.3 0.3], 'MarkerFaceColor', [1 0.3 0.3])
plot(data(rows,5), data(rows,6), 'o', 'markersize', 20, 'color', [1, 0.5, 0], 'linewidth', 1.5)

txt = {'t_1', 't_2', 't_3', 't_4'};
text(data(rows,1)+0.5, data(rows,2)-0.5, txt)
text(data(rows,3)-2, data(rows,4)+1, txt)

% rows = [31 34 51];
% txt = {'t_1 - t_2', 't_3', 't_4'};
% text(data(rows,5)+1.2, data(rows,6)+1, txt)

rows = [1 34 51];
txt = {'A (t_1 - t_2)', 'B (t_3)', 'C (t_4)'};
text(data(rows,5)-1.8, data(rows,6)-1.2, txt)


%% visualize interception line

dt = 0.1;
p = Agent2D('pos', [-3, -1], 'yaw', -1.7, 'vmax', 1, 'dt', dt, 'color', [0.3 0.3 1], ...
    'shape', [10, -7.5; 13, 0; 10, 7.5; -10, 7.5; -10, -7.5; 10, -7.5] * 7e-2);
e = Agent2D('pos', [0, -8], 'yaw', 3, 'vmax', 2, 'dt', dt, 'color', [1 0.3 0.3], ...
    'shape', [10, -7.5; 13, 0; 10, 7.5; -10, 7.5; -10, -7.5; 10, -7.5] * 7e-2);

pl = Planner2D_fast('p', p, 'e', e, 'm', map, 'gpos', [1.4862   -0.5248]);
pl.tp = grow_tree(map, p.pos);
pl.te = grow_trim_tree(pl, pl.tp, e.pos);

%%
step(pl)

x = map.lims(1) : 0.3: map.lims(2);
y = map.lims(3) : 0.3 : map.lims(4);

[xg, yg] = meshgrid(x, y);

c = zeros(length(y), length(x));
vxe = zeros(length(y), length(x));
vye = zeros(length(y), length(x));
cp = zeros(length(y), length(x));
ce = zeros(length(y), length(x));
tic
for k1 = 1 : length(x)
    for k2 = 1 : length(y)
        c(k2,k1) = find_path(map, pl.tg, [x(k1), y(k2)]);
        [ce(k2,k1), ~, ve] = find_path(map, pl.te, [x(k1), y(k2)]);
        cp(k2,k1) = find_path(map, pl.tp, [x(k1), y(k2)]);
        
        vxe(k2,k1) = ve(1);
        vye(k2,k1) = ve(2);
    end
end
toc

% te2 = grow_tree(map, e.pos);
% te2.cumcost = te2.cumcost * p.vmax / e.vmax;
% pl.te.cumcost = pl.te.cumcost * p.vmax / e.vmax;

% figure, plot_tree(map, pl.te)
% figure, plot_tree(map, te2)

% pl.te.cumcost = pl.te.cumcost * e.vmax / p.vmax;

% d = 5;
% figure, plot_tree(map, pl.te)
% quiver(xg(1:d:end,1:d:end), yg(1:d:end,1:d:end), -vxe(1:d:end,1:d:end), -vye(1:d:end,1:d:end))
% plot(e), plot(p)


%%
% pl = Planner2D_fast('p', p, 'e', e, 'm', map, 'gpos', [1.4862   -0.5248]);
% 
% p2 = Agent2D('pos', [0, -8], 'yaw', -1.7, 'vmax', 1, 'dt', dt);
% e2 = Agent2D('pos', [-3, -1], 'yaw', 3, 'vmax', 1.5, 'dt', dt);
% 
% pl2 = Planner2D_fast('p', e2, 'e', p2, 'm', map, 'gpos', [1.4862   -0.5248]);
% step(pl2)
% step(pl)
% te = grow_tree(map, e.pos);
% te.cumcost = te.cumcost * p2.vmax / e2.vmax;
% tp = grow_trim_tree(pl2, te, p.pos);
step(pl)

figure
contour(x, y, ce/e.vmax - cp/p.vmax, [-100, 0], '-.', 'linewidth', 1.5, 'color', [1 0.5 0])
hold on
plot(pl)
plot_tree(pl.m, pl.te, [0.8, 0, 0])
% plot_tree(map, tp, [0, 0, 0.8])
plot(p)
plot(e)
% plot(pl.gpos(1), pl.gpos(2), 's', 'markersize', 20)
% scatter(pl2.gpos(1), pl2.gpos(2), 400, [0, 0.8, 0], 'filled', 'MarkerFaceAlpha', 0.3)



%%
[sol, fval2] = step(pl);
cc = c;
cc(isinf(cc)) = NaN;

figure
plot(map)
contour(x, y, ce/e.vmax - cp/p.vmax, [-100, 0], '-.', 'linewidth', 1.5, 'color', [1 0.5 0])
hold on
plot(p)
plot(e)
scatter(pl.gpos(1), pl.gpos(2), 400, [0, 0.8, 0], 'filled', 'MarkerFaceAlpha', 0.3)
contour(x, y, cc, 5:3:20, 'showtext', 'on')
h = scatter(sol(:,1), sol(:,2), 0.2*(150 + 1500*mean(fval2)./(1+fval2)), [1, 0.5, 0], 'linewidth', 1.5)

%%
figure
plot_tree(map, pl.tg, [0 0 0])
contour(x, y, cc, 'ShowText', 'on')
hold on


%%

pos0 = pos1(find(~col,100),:);
pos = zeros(size(pos0));
exitflag = zeros(size(pos0,1), 1);
fval = zeros(size(pos0,1), 1);
tic
for k = 1 : size(pos0,1)
    [pos(k,:), fval(k), exitflag(k)] = search_intercept(pl, pos0(k,:));
end
toc * 1000 / k

100 * sum(exitflag>0) / length(exitflag)

figure
plot(map)
plot(p)
plot(e)
plot(pl.gpos(1), pl.gpos(2), 'o', 'markersize', 20)
plot(pos0(:,1), pos0(:,2), 'x')

for k = 1 : size(pos0,1)
    if exitflag(k) > 0
        plot([pos0(k,1), pos(k,1)], [pos0(k,2), pos(k,2)])
        plot(pos(k,1), pos(k,2), 'x', 'markersize', 20)
    end
end


%% visualize path plan
shape = [10, -7.5; 13, 0; 10, 7.5; -10, 7.5; -10, -7.5; 10, -7.5] * 1e-2;
plan = [0.5, 1; 1.5, 1; 2.5, 2; 2.5, 3];
pts = [
  -12.3442   -5.2479
  -11.7068   -1.9759
  -10.3895    0.6161
   -8.2649    2.4008
   -5.2054    3.2507
   -2.3584    3.2507
   -1.1686    1.6360
   -0.9986   -0.4462
   -1.0836   -3.1657 ];
s = [0; cumsum(sqrt(sum(diff(pts).^2,2)))];
ss = s(1) : 0.1 : s(end);
pts = interp1(s, pts, ss, 'pchip');
plan = pts;

p = Agent2D('pos', [1, 1.4], 'yaw', 30 * pi/180, 'vmax', 3, 'dt', dt, 'color', [0.3 0.3 1], ...
    'shape', shape);

% p.plan = plan;
set_plan(p, plan, 80)
% line_follow2(p)
% posp = shape * R(p.yaw)' + p.pos;
% 
% plot(plan(:,1), plan(:,2), 's-')
% fill(posp(:,1), posp(:,2), [0.5 0.5 1])
% axis equal

[x, y] = meshgrid(map.lims(1):0.5:map.lims(2), map.lims(3):0.7:map.lims(4));
dth = zeros(size(x));
mindist = zeros(size(x));
for k1 = 1 : size(x,1)
    for k2 = 1 : size(x,2)
        p.pos = [x(k1,k2), y(k1,k2)];
        [~,~,dth(k1,k2)] = line_follow(p);
        mindist(k1,k2) = min( sum((p.pos - p.plan).^2, 2) );
    end
end

u = cos(dth) .* (mindist < 3);
v = sin(dth) .* (mindist < 3);

figure
quiver(x, y, u, v, 'color', [0.5 0.3 0])
hold on
plot(p.plan(:,1), p.plan(:,2), '-', 'linewidth', 2, 'color', [0.7 0 0])
axis equal
plot(map)
plot(pts(1,1), pts(1,2), 'o', 'markersize', 15, 'MarkerFaceColor', [0.7 0 0], 'MarkerEdgeColor', 'none')
plot(pts(end,1), pts(end,2), '^', 'markersize', 15, 'MarkerFaceColor', [0.7 0 0], 'MarkerEdgeColor', 'none')

%% validate hessian

dt = 0.1;
p = Agent2D('pos', [5, 3], 'vmax', 3, 'dt', dt, 'color', [0.3 0.3 1], ...
    'shape', [10, -7.5; 13, 0; 10, 7.5; -10, 7.5; -10, -7.5; 10, -7.5] * 7e-2);
e = Agent2D('pos', [12, -4], 'vmax', 3, 'dt', dt, 'color', [1 0.3 0.3], ...
    'shape', [10, -7.5; 13, 0; 10, 7.5; -10, 7.5; -10, -7.5; 10, -7.5] * 7e-2);

pl = Planner2D_fast('p', p, 'e', e, 'm', map, 'gpos', [0, 3]);
pl.tp = grow_tree(map, p.pos);
pl.te = grow_tree(map, e.pos);

plot_tree(map, tg)
plot(p)
plot(e)
plot(ipos(1), ipos(2), 's')
guess = ginput;

[ipos, fval, exitflag, fcount] = search_intercept(pl, guess);

tg = grow_tree(map, pl.gpos);
[mincost, kend, vend, wpos, v0] = find_path(map, tg, guess);

x = guess(1) + (-5:5) * 1e-3;
y = guess(2) + (-3:3) * 1e-3;
[xg, yg] = meshgrid(x, y);
mcost = zeros(size(xg));
for k1 = 1 : size(xg,1)
    for k2 = 1 : size(yg,2)
        mcost(k1,k2) = find_path(map, pl.tp, [xg(k1,k2), yg(k1,k2)]) - ...
            find_path(map, pl.te, [xg(k1,k2), yg(k1,k2)]);
    end
end

dx = mean(diff(x));
dy = mean(diff(y));
cx = diff(mcost,[],2) / dx;
cy = diff(mcost,[],1) / dy;

cxx = diff(cx,[],2) / dx;
cyy = diff(cy,[],1) / dy;

cxy = diff(cx,[],1) / dy;
cyx = diff(cy,[],2) / dx;

grad = [mean(cx(:)), mean(cy(:))]
Hc = [mean(cxx(:)), mean(cxy(:))
    mean(cyx(:)), mean(cyy(:))]

[Hout, H, Hp, He] = hessianfcn(pl, guess', struct('eqnonlin', 1));
Hc - (Hp - He)

%%

plot_tree(map, tg)
plot(p)
plot(e)
plot(ipos(1), ipos(2), 's')

contourf(x, y, mcost,'ShowText','on'), colorbar

plot_tree(map, tg, [1 0 0])
plot(pos(1), pos(2), '^')
