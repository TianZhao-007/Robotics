function error_ellipse_lm(zeta,Sigma, p)
% hold off
s = -2 * log(1 - p);
t = linspace(0, 2 * pi);
Sigma_robot = Sigma(1:2,1:2);

% landmark
num_landmark = (length(zeta) - 3)/2;
if num_landmark ~= 0
    for i = 1:num_landmark
        Sigma_landmark = Sigma([3+2*i-1:3+2*i],[3+2*i-1:3+2*i]);
        zx = zeta(3+2*i-1);
        zy = zeta(3+2*i);
        [V, D] = eig(Sigma_robot * s);
        a = (V * sqrt(D)) * [cos(t(:))'; sin(t(:))'];
%         plot(a(1, :) + zx, a(2, :) + zy, 'm','Parent',trail_axes);
        scatter(a(1, :) + zx,a(2, :) + zy,5,...
            'MarkerFaceColor',box(i),...
            'MarkerEdgeColor','none');
    end
end
% hold on
end
