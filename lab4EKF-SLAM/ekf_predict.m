function [zeta, Sigma] = ekf_predict(zeta, Sigma, v, q, R, dt)
% This function predicts the state and covariance.

theta = zeta(3);
num_landmark = (length(zeta) - 3)/2;

% A
A1 = [       1,         0,    -dt*v*sin(theta);...
             0,         1,    dt*v*cos(theta); ...
             0        , 0,    1];
if num_landmark == 0
    A = A1;
else
    A = [A1, zeros(3,2*num_landmark);...
    zeros(2*num_landmark,3), eye(2*num_landmark)];
end

% B
B1 = dt*[cos(theta),  0;...
        sin(theta),   0; ...
            0,        1];
if num_landmark == 0
    B = B1;
else
    B = [B1; zeros(2*num_landmark,2)];
end

% new Sigma
Sigma = A*Sigma*A' + B*R*B';
% new zeta
zeta(1:3) = [zeta(1)+dt*cos(theta)*v; zeta(2)+dt*sin(theta)*v; theta+dt*q;];
end
