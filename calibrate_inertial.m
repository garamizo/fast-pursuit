% Place markers on all corners of bodies 2 and 4
% Record for 1 second

m = Mocap('C:\Users\hirolab\Documents\Optitrack\pursuit\calib\Take 2018-03-04 04.23.22 PM.csv');

b2 = m.body('Rigid Body 2');
b4 = m.body('Rigid Body 4');

p2 = squeeze(mean(cat(3, b2.marker.trans) - b2.trans))';
th = cart2pol(p2(:,1), p2(:,3));
[~, id] = sort(th);
points2 = squeeze(mean(cat(3, b2.marker(id).trans)))';

p4 = squeeze(mean(cat(3, b4.marker.trans) - b4.trans))';
th = cart2pol(p4(:,1), p4(:,3));
[~, id] = sort(th);
points4 = squeeze(mean(cat(3, b4.marker(id).trans)))';

%%
m = Mocap('C:\Users\hirolab\Documents\Optitrack\pursuit\gen_map\Take 2018-02-23 07.38.35 PM.csv');

b2 = m.body('Rigid Body 2');
b4 = m.body('Rigid Body 4');

p2 = squeeze(mean(cat(3, b2.marker.trans) - b2.trans))';
th = cart2pol(p2(:,1), p2(:,3));
[~, id] = sort(th);
points20 = squeeze(mean(cat(3, b2.marker(id).trans)))';

p4 = squeeze(mean(cat(3, b4.marker.trans) - b4.trans))';
th = cart2pol(p4(:,1), p4(:,3));
[~, id] = sort(th);
points40 = squeeze(mean(cat(3, b4.marker(id).trans)))';

%% Assign manually
p0 = [points20; points40];
p1 = [points2; points4];

figure, 
plot3(p0(:,1), p0(:,2), p0(:,3), 'o')
hold on, plot3(p1(:,1), p1(:,2), p1(:,3), 's')

pp = reshape(permute(cat(3, p0, p1, NaN(size(p0))), [3 1 2]), [], 3);
plot3(pp(:,1), pp(:,2), pp(:,3), 'k')

text(points20(:,1)+0.1, points20(:,2), points20(:,3), num2str((1:4)'))
text(points40(:,1)+0.1, points40(:,2), points40(:,3), num2str((1:4)'))

text(points2(:,1)+0.1, points2(:,2), points2(:,3), num2str((1:4)'))
text(points4(:,1)+0.1, points4(:,2), points4(:,3), num2str((1:4)'))

%%
t = sym('t', [1, 3]);
q = sym('q', [1, 4]);

p0 = [points20; points40];
p1 = [points2; points4];
clear tmp
for k = 1 : size(p0, 1)
    tmp(k,1) = norm(p0(k,:) - (quatrotate2(quatinv2(q), p1(k,:)) - t));
end
objfun = matlabFunction(sum(tmp), 'Vars', {[t, q]});
constrfun = matlabFunction([], q * q.' - 1, 'Vars', {[t, q]});

x0 = [0, 0, 0, 1, 0, 0, 0];
lb = [-1, -1, -1, 0.5, -0.3, -0.3, -0.3];
ub = [1, 1, 1, 1, 0.3, 0.3, 0.3];
[x, fval, exitflag, output] = fmincon(objfun, x0, [], [], [], [], lb, ub, constrfun)

opts = optimoptions(@fmincon,'Algorithm', 'interior-point');
problem = createOptimProblem('fmincon', ...
    'objective', objfun, 'x0', x0, 'lb', lb, 'ub', ub ,'options', opts, 'nonlcon', constrfun);
gs = GlobalSearch ('Display', 'off', 'MaxTime', 50);
[xg, fvalg] = run(gs, problem);
p11 = quatrotate(quatinv(xg(4:7)), p1) - xg(1:3);

mocap2pursuit = @(pos) quatrotate(quatinv(xg(4:7)), pos) - xg(1:3);

objfun(x0)
objfun(x)
objfun(xg)

%%
figure, 

plot3(p0(:,1), p0(:,2), p0(:,3), 'o')
hold on, plot3(p1(:,1), p1(:,2), p1(:,3), 's')
plot3(p11(:,1), p11(:,2), p11(:,3), 'x')

pp = reshape(permute(cat(3, p0, p1, NaN(size(p0))), [3 1 2]), [], 3);
plot3(pp(:,1), pp(:,2), pp(:,3), 'k')

pp = reshape(permute(cat(3, p0, p11, NaN(size(p0))), [3 1 2]), [], 3);
plot3(pp(:,1), pp(:,2), pp(:,3), ':b')

xlabel('x'), ylabel('y'), zlabel('z'), grid on, 
camup([0 1 0])

axis equal
xlim([-3, 3]), ylim([-0.5, 1]), zlim([-2, 2])

view(3)




