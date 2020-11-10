function error_ellipse_rob(zeta,Sigma, p,trail_axes)
hold off
s = -2 * log(1 - p);
t = linspace(0, 2 * pi);

% robot
Sigma_robot = Sigma(1:2,1:2);
[V, D] = eig(Sigma_robot * s);
a = (V * sqrt(D)) * [cos(t(:))'; sin(t(:))'];
plot(a(1, :) + zeta(1), a(2, :) + zeta(2), 'c','Parent',trail_axes);
xlim(trail_axes, [0,5])
ylim(trail_axes, [0,5])
hold on
end