%% Instrument Connection

% Find a Bluetooth connection object.
p = instrfind('Type', 'bluetooth', 'Name', 'Bluetooth-HIRO0:1', 'Tag', '');

% Create the Bluetooth connection object if it does not exist
% otherwise use the object that was found.
if isempty(p)
    p = Bluetooth('HIRO0', 1);
else
    fclose(p);
    p = p(1);
end

% Connect to instrument object, obj1.
fopen(p);

fprintf(p, '%d %d\n', [100 0]);
pause(0.5)
fprintf(p, '%d %d\n', [0 0]);

%%
c = natnet();
c.HostIP = '141.219.208.59';
c.ClientIP = '141.219.208.59';
pause(0.5);
c.connect()

% c.disconnect()

%%
body0 = [10e-2, -7.5e-2
        13e-2,  0
        10e-2, 7.5e-2
        -10e-2, 7.5e-2
        -10e-2, -7.5e-2
        10e-2, -7.5e-2]; % robot shape

Rz = @(th) [cos(th), -sin(th)
            sin(th), cos(th)];

dt = 0.05; % time update 
vmax = 2.5/3;
thdmax = vmax / 6.5e-2;

% initialize robot's state
x = 0;
y = 0;
th = 0;

%%
o1 = [0.212277, 0.033398 
    0.208017, 0.283829];
o2 = [-0.044225, 0.576599
    -0.198203, 0.482857];
o3 = [-0.792549, 0.204438
    -0.670156, 0.421669];
o4 = [-0.467801, -0.051063
    -0.276209, 0.118246];
o5 = [-0.520799, -0.330429
    -0.271033, -0.371516];
o6 = [0.001535, -0.215879
    0.194662, -0.391545];
figure(2)
plot(body0(:,1), body0(:,2), o1(:,1), o1(:,2), o2(:,1), o2(:,2), o3(:,1), o3(:,2), o4(:,1), o4(:,2), o5(:,1), o5(:,2), o6(:,1), o6(:,2))
xlim([-1, 1]), ylim([-1, 1]), axis square
%% create trajectory
fposition = [162    74   628   594];

body = body0 * Rz(th).' + [x, y];
plot(body(:,1), body(:,2), o1(:,1), o1(:,2), o2(:,1), o2(:,2), o3(:,1), o3(:,2), o4(:,1), o4(:,2), o5(:,1), o5(:,2), o6(:,1), o6(:,2))
set(gcf, 'position', fposition)
xlim([-1, 1]), ylim([-1, 1]), axis square

[xx, yy] = getpts();

s = [0; cumsum(sqrt(diff(xx).^2 + diff(yy).^2))];
ss = s(1) : (0.9*vmax*dt) : s(end);

xt = interp1(s, xx, ss, 'pchip');
yt = interp1(s, yy, ss, 'pchip');

hold on
plot(xt, yt, xx, yy, 'o')


autoref = true; % automatic reference

%% Calibrate mouse pointer
figure
% plot(xt, yt, xx, yy, 'o')
% hold on
h = plot(0, 0, 'x', 'MarkerSize', 50);
xlim([-5, 5]), ylim([-5, 5]), axis square

alocs = [-5, -5; 0, 0; 5, 5];
plocs = alocs;
for k = 1 : size(alocs, 1)
    set(h, 'XData', alocs(k,1), 'YData', alocs(k,2))
    ginput(1);
    plocs(k,:) = get(0, 'PointerLocation'); % pointer absolute location
end

fposition = get(gcf, 'position');
close all
autoref = false;


%% Display motion

% define trajectory
x = 0;
y = 0;
th = 0;
pref = [x, y];

if autoref 
    runtime = length(xt);
else
    runtime = round(60 / dt);
end

try
% clear logv
zerr = [0, 0, 0, 0];
for k = 1 : runtime
    tic
    
    if autoref
        pref = [xt(k), yt(k)];
    else
        paloc = get(0, 'PointerLocation'); % pointer absolute location
        pref = [interp1(plocs(:,1), alocs(:,1), paloc(1), 'linear', 0), ...
            interp1(plocs(:,2), alocs(:,2), paloc(2), 'linear', 0)]; % axis position
    end
    
    % acquire x, y, th from Mocap (implementation)
    f = c.getFrame();
    
    q0 = [cosd(-90/2), sind(-90/2), 0, 0];
    quat = double([f.RigidBody(1).qw, f.RigidBody(1).qx, f.RigidBody(1).qy, f.RigidBody(1).qz]);
    quat = quatmultiply(quatmultiply(quatinv(q0), quat), (q0));
    trans = double([f.RigidBody(1).x, f.RigidBody(1).y, f.RigidBody(1).z]);
    trans = quatrotate(q0, trans);
    
    x = trans(1);
    y = trans(2);
    th = quat2angle((quat), 'ZYX');
    
%     if abs(x) > 0.75 || abs(y) > 0.75
%         fprintf(p, '%d %d\n', [0, 0]);
%         warning('Out of bounds!')
%         break
%     end
    
    %
    
    perr = (pref - [x, y]) * Rz(th); % position error on car frame
    therr = atan2(perr(2), perr(1));
    therr = atan2(sin(therr), cos(therr)); % wrap angle
    
    zerr = [perr(1), therr, zerr(3) + dt*perr(1), zerr(4) + dt*therr];
    if sqrt( perr * perr' ) > 7e-2 % if error is large
        u = [   2.5, 0, 0, 0
                0, 20, 0, 0] * zerr';
    else % if error is small
        zerr(3:4) = 0;
        u = [1, 0, 0, 0
                0, 1, 0, 0] * zerr';
    end

    k1 = vmax/10;
    k2 = thdmax/10;
    sol = [k1, k1; k2, -k2] \ u;

    if max(abs(sol)) > 5
        sol = sol * 5 / max(abs(sol));
    end
    v1 = sol(1);
    v2 = sol(2);
    
    logv(k,:) = [pref, x, y, th, v1, v2];
    
    % send v1, v2 to agent (implementation)
    fprintf(p, '%d %d\n', round([v1, v2]*255/5));

    %{
    % update position and angle (simulation)
    v = (v1 + v2) * vmax/10;
    thd = (v1 - v2) * thdmax/10; % v1 is right wheel
    x = x + v*cos(th)*dt;
    y = y + v*sin(th)*dt;
    th = th + thd*dt;
    th = atan2(sin(th), cos(th)); % wrap angle 
    %}
    
    outter = [2.62, -1.2
    2.61 1.07
    -3.32 1.10
    -3.14 -1.8
    -1.54 -3.37
    -0.33 -2.84
    2.23 -2.77
    2.13 -1.23
    2.62, -1.2];

    inner = [0.35 -1.17
    0.16 0.10
    -0.85 0.10
    -0.85 -0.58
    -2.07 -0.6
    -2.11 -1.97
    -0.06 -2.46
    1.09 -1.99
    1.06 -1.18
    0.35 -1.17];
    

    plot_enable = true;
    if plot_enable
        % plotting
        body = body0 * Rz(th).' + [x, y];
        hold off
        plot(body(:,1), body(:,2))
        set(gcf, 'position', fposition)
        xlim([-5, 5]), ylim([-5, 5]), axis square
        hold on
        if autoref
            plot(xt, yt)
        end
        plot(logv(1:k,3), logv(1:k,4))
        v1plot = [0, -6.5e-2; v1*20e-2/5, -6.5e-2] * Rz(th)' + [x, y];
        v2plot = [0, 6.5e-2; v2*20e-2/5, 6.5e-2] * Rz(th)' + [x, y];
%         plot(v1plot(:,1), v1plot(:,2), v2plot(:,1), v2plot(:,2), o1(:,1), o1(:,2), o2(:,1), o2(:,2), o3(:,1), o3(:,2), o4(:,1), o4(:,2), o5(:,1), o5(:,2), o6(:,1), o6(:,2))
        plot(pref(1), pref(2), 'x', 'MarkerSize', 30)
        plot(outter(:,1), outter(:,2), inner(:,1), inner(:,2))
        drawnow

        pause(0.1 - toc)
    end
end

mean( sum((logv(:,1:2) - logv(:,3:4)).^2, 2) ) * 1e2

catch ME
%     fprintf(p, '%d %d\n', round([100 -100]));
%     pause(1)
    fprintf(p, '%d %d\n', [0 0]);
    
    
    throw(ME)
end

fprintf(p, '%d %d\n', [0 0]);
