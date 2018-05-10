
t = tcpip('141.219.125.115', 80);

%%
fopen(t)

%%
fprintf(t, '75r 90r 5')

if (fgetl(t) > 0)
    fgetl(t)
else
    fprintf('Disconnected');
end

%%
% 50 50 5
% 7*30.5+29 / 8*30.5 / 8*30.5+6.5 = 245.6667
% 75 75 50
% 10*30.5+23.5 / 10*30.5+22 / 10*30.5+24 = 328.1667
% 100 100 5
% 12*30.5+16 / 12*30.5+17.5 / 12*30.5+13 = 381.5
% 75 90 5
% 11*30.5+1 / 30.5+7

%%
vr = 75;
vl = 90;
th = pi/2;

dt = 1/5;
pos = 0;
lastPos = pos;

k1 = 0.4376;
k2 = -0.9333;

Rz = @(th) [cos(th), -sin(th), 0
    sin(th), cos(th), 0
    0, 0, 1];

for k = 1 : 25
    model = [(vr + vl) * k1; 0
        (vr - vl) * k2];

    pos = lastPos + Rz(th) * model * dt;
    
    lastPos = pos;
    
%     pause(0.2);
end

%%
fclose(t)


