clear
t = tcpip('141.219.123.228', 80);

%%
fopen(t)

c = natnet();
c.HostIP = '141.219.208.59';
c.ClientIP = '141.219.208.59';
c.connect()

% c.disconnect()

%%
fprintf(t, '75r 90r 5')

if (fgetl(t) > 0)
    fgetl(t)
else
    fprintf('Disconnected');
end

%%
fclose(t)

%%
mapfile = 'maze';
sdata = load(fullfile('maps', mapfile));
map = Map2D_fast('obs', sdata.output, 'lims', [sdata.x_constraints, sdata.y_constraints]);

dt = 0.1;
p = Agent2D('pos', [-3.7031   -0.8438], 'yaw', 0, 'vmax', 5, 'wmax', 10, 'dt', dt, 'color', [0.3 0.3 1], ...
    'hb', t, 'hc', c, 'shape', [10, -7.5; 13, 0; 10, 7.5; -10, 7.5; -10, -7.5; 10, -7.5] * 7e-2);
e = Agent2D('pos', [9.7031   -6.5625], 'yaw', 2, 'vmax', 5, 'wmax', 10, 'dt', dt, 'color', [1 0.3 0.3], ...
    'shape', [10, -7.5; 13, 0; 10, 7.5; -10, 7.5; -10, -7.5; 10, -7.5] * 7e-2);

pl = Planner2D_fast('p', p, 'e', e, 'm', map, 'gpos', [1.4862   -0.5248]);

pts = [
    8.9531   -0.6563
    3.7969   -7.4063
   -2.5781   -7.5000
   -3.1406   -7.1250
   -3.1406   -6.2813
    1.5469   -4.7813
    1.6406   -0.8438 ];
set_plan(e, [e.pos; pts], 100);

data = zeros(3000,6);

% initial assessment
[x, fval] = step(pl);
[~, idx] = min(fval);
[~, ~, ~, path] = find_path(map, pl.tp, x(idx,:));
pplan = [p.pos; path];

% mouse pointer
plot(map), hold on
hh = plot(0, 0, '^');

% close all
for k = 1 : 3000
    tic

    % get mouse location and set evader path
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
    
    p0 = step(p);
    if collide(map, p0, p.pos)
        p.pos = p0;
    end
    
    p0 = step(e);
    if collide(map, p0, e.pos)
        e.pos = p0;
    end
    
    plot(p)
    plot(e)
    
    % test capture or interception
    if sqrt(sum((e.pos - pl.gpos).^2)) < 1 || sqrt(sum((e.pos - p.pos).^2)) < 1
        break
    end

    drawnow
    pause(dt - toc)
end


