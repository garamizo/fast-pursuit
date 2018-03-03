%% import map from pdetool
% export geometry description (on PDE, Draw > Export Geometry
% Description...)

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

mapfile = 'test_map';
save(fullfile('maps', mapfile), 'output', 'x_constraints', 'y_constraints', 'num')

%% OR create struct by hand
% mocap z -> map y
% mocap x -> map -x
R = Rz(pi) * Rx(pi/2); % to map

% place a marker in each corner of each obstacle and save it as a body
% record the setup and export body markers
m = Mocap('C:\Users\hirolab\Documents\Optitrack\pursuit\gen_map\Take 2018-02-23 07.38.35 PM.csv');
num = length(m.BodyName);

clear output
for k = 1 : num
    fprintf('Detected body [%s]\n', m.BodyName{k})
    b = m.body(m.BodyName{k});
    output{k} = [nanmean(b.marker(1).trans)
                    nanmean(b.marker(2).trans)
                    nanmean(b.marker(3).trans)
                    nanmean(b.marker(4).trans) ] * R' * [1, 0; 0, 1; 0, 0];
                
    % fix order
    p0 = mean(output{k});
    pp = output{k} - p0;
    th = atan2(pp(:,2), pp(:,1));
    [~, order] = sort(th);
    output{k} = output{k}(order([1:end, 1]),:);
end

x_constraints = [-2.6266, 3.4542]; % fill in manually
y_constraints = [-1.0266, 1.3198];

mapfile = 'lab_map';
save(fullfile('maps', mapfile), 'output', 'x_constraints', 'y_constraints', 'num')

%% load map

% mapfile = 'sample_block';
mapfile = 'lab_map';
sdata = load(fullfile('maps', mapfile));
map = Map2D_fast('obs', sdata.output, 'lims', [sdata.x_constraints, sdata.y_constraints]);

plot(map)

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

figure
plot_tree(map, tr)
quiver(pos(k,1), pos(k,2), v(1), v(2), 'linewidth', 2)

%% test speed of intercept search
% 12 ms for maze

mapfile = 'maze';
sdata = load(fullfile('maps', mapfile));
map = Map2D_fast('obs', sdata.output, 'lims', [sdata.x_constraints, sdata.y_constraints]);
tree = map.grow_tree([-0.7969   -2.4375]);

N = 1000;
X0 = rand(N,2) .* (map.lims([2 4]) - map.lims([1 3])) + map.lims([1 3]);
all_reachable = false;
while ~all_reachable
    for k = 1 : size(X0, 1)
        reachable(k,1) = isfinite(find_path(map, tree, X0(k,:)));
    end
    X0(~reachable,:) = rand(sum(~reachable),2) .* (map.lims([2 4]) - map.lims([1 3])) + map.lims([1 3]);
    all_reachable = all(reachable);
end
%%

th = linspace(0, 2*pi, 15)'; shape = [cos(th), sin(th)] * 1;
n = 100;
rate = zeros(n, 1);
converge = zeros(n, 1);
for k = 1 : n
    dt = 0.1;
    vmax = 1.5;
    rows = randi(N, [4, 1]);
    rowsN = randi(N, [n, 1]);
    p(1) = Agent2D('pos', X0(rows(1),:), 'vmax', vmax, 'dt', dt, 'color', [0.3 0.3 1], ...
        'yaw', 2, 'shape', shape);
    p(2) = Agent2D('pos', X0(rows(2),:), 'vmax', vmax, 'dt', dt, 'color', [0.3 0.3 1], ...
        'yaw', 2, 'shape', shape);
    e = Agent2D('pos', X0(rows(3),:), 'vmax', vmax, 'dt', dt, 'color', [1 0.3 0.3], ...
        'yaw', 0.5, 'shape', shape);
    x0 = X0(rowsN,:);
    pl = Planner2D_fast('p', p, 'e', e, 'm', map, 'gpos', X0(rows(4),:), 'x0', x0);

    tic
    step(pl);
    rate(k) = toc * 1000 / size(x0,1);
    if ~isempty(pl.etc)
        converge(k) = 100 * sum(isfinite(pl.etc.x(:,1))) / size(x0,1);
    else
        converge(k) = 0;
    end
    
    fprintf('Execution time: %.2f ms\n', rate(k))
end

% figure, plot(pl), plot(p(1)), plot(p(2)), plot(e)

%% Plot visibility
mapfile = 'maze';

sdata = load(fullfile('maps', mapfile));
map = Map2D_fast('obs', sdata.output, 'lims', [sdata.x_constraints, sdata.y_constraints]);
plot_visibility(map)

%% Planner step
% close all
th = linspace(0, 2*pi, 15)';

% agent_positions = [ 1.4862      -0.5248
%                     -2.1180     0.8210
%                     2.5         -0.5
%                     2.5         0.8
%                     1.5         0.9
%                     1.5         -0.5
%                     3.5         -0.5 ];
% mapfile = 'lab_map';
% shape = [cos(th), sin(th)] * 0.1;
% vmax = 0.5;

agent_positions = [ 1.4862      -0.5248
                    -12.1523    -2.8
                    -2 3.5
                      -12.3778   -8.4023
                        0.6485   -8.3459
                        8.3741   -5.1880
                       10.2350    3.4398 ];
                   

mapfile = 'maze';
shape = [cos(th), sin(th)] * 0.5;
vmax = 1.5;

sdata = load(fullfile('maps', mapfile));
map = Map2D_fast('obs', sdata.output, 'lims', [sdata.x_constraints, sdata.y_constraints]);
dt = 0.1;

e = Agent2D('pos', agent_positions(2,:), 'vmax', vmax, 'dt', dt, 'color', [1 0.3 0.3], ...
    'yaw', 0.5, 'shape', shape, 'ctrl', HolonomicController());
p = Agent2D('pos', agent_positions(3,:), 'vmax', vmax, 'dt', dt, 'color', [0.3 0.3 1], ...
    'yaw', 0.5, 'shape', shape, 'ctrl', HolonomicController());
p(2) = Agent2D('pos', agent_positions(5,:), 'vmax', vmax, 'dt', dt, 'color', [0.3 0.3 1], ...
    'yaw', 0.5, 'shape', shape, 'ctrl', HolonomicController());
% p(3) = Agent2D('pos', agent_positions(5,:), 'vmax', vmax, 'dt', dt, 'color', [0.3 0.3 1], ...
%     'yaw', 0.5, 'shape', shape, 'ctrl', HolonomicController());
% p(4) = Agent2D('pos', agent_positions(6,:), 'vmax', vmax, 'dt', dt, 'color', [0.3 0.3 1], ...
%     'yaw', 0.5, 'shape', shape, 'ctrl', HolonomicController());
% p(5) = Agent2D('pos', agent_positions(7,:), 'vmax', vmax, 'dt', dt, 'color', [0.3 0.3 1], ...
%     'yaw', 0.5, 'shape', shape, 'ctrl', HolonomicController());

pl = Planner2D_fast('p', p, 'e', e, 'm', map, 'gpos', agent_positions(1,:));

% initial assessment
x = step(pl);
figure, plot(pl)

%%

plot_constraint(pl)

text(p(1).pos(1)+1, p(1).pos(2), 'P1')
text(p(2).pos(1)+1, p(2).pos(2), 'P2')
text(e.pos(1)+1, e.pos(2), 'E')
text(pl.gpos(1)-0.5, pl.gpos(2), 'T')
text(pl.tracks(1).pos(1)-2, pl.tracks(1).pos(2)-1, 'I_1')
text(pl.tracks(2).pos(1)-2, pl.tracks(2).pos(2)-1, 'I_3')
text(pl.tracks(3).pos(1)-2, pl.tracks(3).pos(2)-1, 'I_2')

% plot(pl)


%% Plot tree
mapfile = 'maze';
th = linspace(0, 2*pi, 15)';
shape = [cos(th), sin(th)] * 0.5;
vmax = 1;

sdata = load(fullfile('maps', mapfile));
map = Map2D_fast('obs', sdata.output, 'lims', [sdata.x_constraints, sdata.y_constraints]);

agent = Agent2D('pos', [-1.3594   -2.4375], 'vmax', vmax, 'dt', dt, 'color', [1 0.3 0.3], ...
    'yaw', 0.5, 'shape', shape, 'ctrl', HolonomicController());
tree = grow_tree(map, agent.pos);

close all


                ds = min(diff(map.lims(1:2)), diff(map.lims(3:4))) / 50;
                x = map.lims(1) : ds : map.lims(2);
                y = map.lims(3) : ds : map.lims(4);
                [xg, yg] = meshgrid(x, y);
                c = zeros(length(y), length(x));
                for k1 = 1 : length(x)
                    for k2 = 1 : length(y)
                        c(k2,k1) = find_path(map, tree, [x(k1), y(k2)]);
                    end
                end
                contour(xg, yg, c);
                
hold on, plot_tree(map, tree)

npos = [9.1406   -1.4063];
candidate_pos = [7.76, 1.51
    3.51, -7.01];
link_cost = sqrt(sum((candidate_pos - npos).^2, 2));
avg_pos = (candidate_pos + npos) / 2;

h = plot(npos(1), npos(2), 'ko', 'MarkerFaceColor', [0 0 0])
plot([candidate_pos(1,1), npos(1)], [candidate_pos(1,2), npos(2)], 'k-.')
plot([candidate_pos(2,1), npos(1)], [candidate_pos(2,2), npos(2)], 'k--')
text(avg_pos(:,1)+1, avg_pos(:,2), num2str(link_cost, '(%.1f)'))
% text(npos(1)+1, npos(2), num2str(find_path(map, tree, npos), '%.1f'), 'FontSize', 13)
text(npos(1)+1, npos(2), num2str(15.7, '%.1f'), 'FontSize', 13)

text(agent.pos(1)-2.5, agent.pos(2)+0.2, 'R', 'FontSize', 13)
text(npos(1)-2, npos(2)+0.2, 'D', 'FontSize', 13)

p1 = [7.76, 1.51];
p2 = [.8, -7.6];
text(p1(1), p1(2), 'N_i')
text(p2(1), p2(2), 'N_j')


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
close all

th = linspace(0, 2*pi, 15)';

% agent_positions = [ 1.4862      -0.5248
%                     -1.4207    0.8781
%                     2.1340    0.9581
%     0.8424   -0.6421
%                     1.5         0.9
%                     1.5         -0.5
%                     3.5         -0.5 ];
% mapfile = 'lab_map';
% shape = [cos(th), sin(th)] * 0.1;
% vmax = 0.3;
% phack = [0, -30, 100, -50];  % pointer hack

% agent_positions = [ 1.4862      -0.5248
%                     -12.2086    1.6917
%                     -1.2688   -3.3271
%                       -12.3778   -8.4023
%                         0.6485   -8.3459
%                         8.3741   -5.1880
%                        10.2350    3.4398 ];
%                    
agent_positions = [ 1.4862      -0.5248
    10.2350   -1.3534
    -1.1560    7.5564
   -4.2656   -3.4688];
    
mapfile = 'maze';
shape = [cos(th), sin(th)] * 0.5;
vmax = 2;
phack = [0, -30, 30, 30];  % pointer hack

sdata = load(fullfile('maps', mapfile));
map = Map2D_fast('obs', sdata.output, 'lims', [sdata.x_constraints, sdata.y_constraints]);
dt = 0.1;
capdist = min(diff(map.lims(1:2)), diff(map.lims(3:4))) / 10;

e = Agent2D('pos', agent_positions(2,:), 'vmax', vmax, 'dt', dt, 'color', [1 0.3 0.3], ...
    'yaw', 0.5, 'shape', shape, 'ctrl', HolonomicController());
p = Agent2D('pos', agent_positions(3,:), 'vmax', vmax, 'dt', dt, 'color', [0.3 0.3 1], ...
    'yaw', 0.5, 'shape', shape, 'ctrl', HolonomicController());
% p(2) = Agent2D('pos', agent_positions(4,:), 'vmax', vmax, 'dt', dt, 'color', [0.3 0.3 1], ...
%     'yaw', 0.5, 'shape', shape, 'ctrl', HolonomicController());
% p(3) = Agent2D('pos', agent_positions(5,:), 'vmax', vmax, 'dt', dt, 'color', [0.3 0.3 1], ...
%     'yaw', 0.5, 'shape', shape, 'ctrl', HolonomicController());
% p(4) = Agent2D('pos', agent_positions(6,:), 'vmax', vmax, 'dt', dt, 'color', [0.3 0.3 1], ...
%     'yaw', 0.5, 'shape', shape, 'ctrl', HolonomicController());
% p(5) = Agent2D('pos', agent_positions(7,:), 'vmax', vmax, 'dt', dt, 'color', [0.3 0.3 1], ...
%     'yaw', 0.5, 'shape', shape, 'ctrl', HolonomicController());

pl = Planner2D_fast('p', p, 'e', e, 'm', map, 'gpos', agent_positions(1,:));
% pl.tp = grow_tree(map, p(1).pos);
% pl.tp(2) = grow_tree(map, p(2).pos);
% pl.te = grow_tree(map, e.pos);

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
x = step(pl);
clear pplan
for k = 1 : length(p)
    [~, ~, ~, pplan{k}] = find_path(map, pl.tp(k), x(k,:));
end

plot(map), hold on
hh = plot(0, 0, '^');
hold on

set_plan(e, [   e.pos
                            8.4305    0.3383
    3.9756   -7.3872
   -2.7350   -7.7820
   -2.9605   -6.8233
    1.3816   -0.90239], 100);

% close all
clear dist data
time = zeros(3000, 1);
t0 = tic;
for k = 1 : 3000
    t12 = tic;

%     ppix = get(0, 'PointerLocation'); % pointer absolute location (X, Y) [px]
%     mpix = get(gcf, 'position').*[1 1 0 0] + get(gca, 'Position');
%     ppos = [interp1([mpix(1), mpix(1)+mpix(2)]+phack(1:2), map.lims(1:2), ppix(1), 'linear', 'extrap'), ...
%         interp1([mpix(3), mpix(3)+mpix(4)]+phack(3:4), map.lims(3:4), ppix(2), 'linear', 'extrap')];
%     set(hh, 'XData', ppos(1), 'YData', ppos(2))
%     set_plan(e, [e.pos; ppos], 100);
%     plot(e.ctrl)
        
    if mod(k-1, 20) == 0
        
        plot(pl)
        
        for k2 = 1 : length(p)
            set_plan(p(k2), pplan{k2}, 100);
            plot(p(k2).ctrl)
        end

        % start calculating new plan
        x = step(pl);
        if ~isempty(x)
            for k2 = 1 : length(p)
                [~, ~, ~, pplan{k2}] = find_path(map, pl.tp(k2), x(k2,:));
%                 set_plan(p(k2), pplan{k2}, 100);
%                 plot(p(k2).ctrl)
            end
        end
    end
    
%     pl.gpos = interp1(K, GPOS, 1+0*min(k,K(end)), 'linear');

    p0 = step(e);
    if collide(map, p0, e.pos)
        fprintf('collision!\n')
        e.pos = p0;
    end
    plot(e)
    
    for k2 = 1 : length(p)
        p0 = step(p(k2));
        if collide(map, p0, p(k2).pos)
            fprintf('collision!\n')
            p(k2).pos = p0;
        end
        plot(p(k2))
        dist(k2) = sqrt(sum((e.pos - p(k2).pos).^2));
    end
    
    if sqrt(sum((e.pos - pl.gpos).^2)) < capdist || any(dist < capdist)
        break
    end
    
    tmp = NaN(1, 2);
    if length(pl.tracks) == 1
        tmp(1:2) = pl.tracks(1).pos;
    elseif length(pl.tracks) >= 2
        tmp = cat(2, pl.tracks(1:2).pos);
    end
    data(k,:) = [p(1).pos, e.pos, tmp];
    
    drawnow
    pause(dt- toc(t12))
    time(k) = toc(t0);
    
    if record
        frame = getframe(gca);
        writeVideo(v, frame);
    end
end
fprintf('Execution time: %.1f ms\n', toc(t0) * 1000 / k)

if record
    close(v)
end

% figure, plot(1./diff(time(1:k-1)), '.-')

%%
p.color = [0.3 0.5 0.7];
e.pos = agent_positions(2,:);
p(1).pos = agent_positions(3,:);
% p(2).pos = agent_positions(4,:);
close all
figure, plot(pl)
plot(data(:,5), data(:,6), '-', 'linewidth', 5, 'color', [1, 0.5, 0, 0.5])
plot(data(:,7), data(:,8), '-', 'linewidth', 5, 'color', [1, 0.5, 0, 0.5])
% plot(data([1 end],7), data([1 end],8), '-', 'linewidth', 5, 'color', [1, 0.5, 0, 0.5])


plot(data(:,1), data(:,2), '-', 'color', p.color)
% plot(data(:,9), data(:,10), 'b-')
plot(data(:,3), data(:,4), 'r-')

text(p(1).pos(1)+1, p(1).pos(2), 'P')
% text(p(2).pos(1)+0.15, p(2).pos(2), 'P2')
text(e.pos(1)+1, e.pos(2), 'E')
text(pl.gpos(1)-0.5, pl.gpos(2), 'T')

text(data(end,7)-1.5, data(end,8)-1.2, 'I_{2}')
text(data(1,7)-0.5, data(1,8)+2, 'I_{20}')
text(data(end,5)+1, data(end,6)-1, 'I_{1}')
text(data(1,5)+1.0, data(1,6), 'I_{10}')

scatter(data(1,[5 7])', data(1,[6 8])', [200; 200], [1, 0.5, 0], 'linewidth', 1.5);
scatter(data(end,[5 7])', data(end,[6 8])', [200; 200], [1, 0.5, 0], 'linewidth', 1.5);
scatter(data(end,[5 ]), data(end,[6 ]), [200], [1, 0.5, 0], '+', 'linewidth', 1.5);
scatter(data(1,[7 ]), data(1,[8 ]), [200], [1, 0.5, 0], '+', 'linewidth', 1.5);

plot(data(21,1), data(21,2), 'b.', 'markersize', 15, 'color', p.color)
plot(data(62,1), data(62,2), 'b.', 'markersize', 15, 'color', p.color)
plot(data(21,3), data(21,4), 'r.', 'markersize', 15)
plot(data(13,3), data(13,4), 'r.', 'markersize', 15)
plot(data(62,3), data(62,4), 'r.', 'markersize', 15)

text(data(21,1)-1.5, data(21,2), 't_2')
text(data(21,3)+0.5, data(21,4)-1, 't_2')
text(data(13,3)+0.7, data(13,4)+0.5, 't_1')
text(data(62,3)+0.7, data(62,4)-0.8, 't_3')
text(data(62,1)-1.5, data(62,2)+1, 't_3')




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
p = Agent2D('pos', [-2.1180    0.8210], 'vmax', 1, 'dt', dt, 'color', [0.3 0.3 1], ...
    'yaw', 2, 'shape', 0.1*[10, -7.5; 13, 0; 10, 7.5; -10, 7.5; -10, -7.5; 10, -7.5] * 7e-2);
e = Agent2D('pos', [2.7970   -0.3792], 'vmax', 1, 'dt', dt, 'color', [1 0.3 0.3], ...
    'yaw', 0.5, 'shape', 0.1*[10, -7.5; 13, 0; 10, 7.5; -10, 7.5; -10, -7.5; 10, -7.5] * 7e-2);

pl = Planner2D_fast('p', p, 'e', e, 'm', map, 'gpos', [1.4862   -0.5248]);
pl.tp = grow_tree(map, p.pos);
pl.te = grow_trim_tree(pl, pl.tp, e.pos);

%%
step(pl)

x = map.lims(1) : 0.2: map.lims(2);
y = map.lims(3) : 0.2 : map.lims(4);

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

te2 = grow_tree(map, e.pos);
te2.cumcost = te2.cumcost * p.vmax / e.vmax;
pl.te.cumcost = pl.te.cumcost * p.vmax / e.vmax;

figure, plot_tree(map, pl.te)
figure, plot_tree(map, te2)

pl.te.cumcost = pl.te.cumcost * e.vmax / p.vmax;

d = 5;
figure, plot_tree(map, pl.te)
quiver(xg(1:d:end,1:d:end), yg(1:d:end,1:d:end), -vxe(1:d:end,1:d:end), -vye(1:d:end,1:d:end))
plot(e), plot(p)


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
