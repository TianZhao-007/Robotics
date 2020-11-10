function [zeta,Sigma] = update(zeta,Sigma,ids,Q,lms,state_ids)
% This function updates the state
if isempty(ids)
    return
end

x = zeta(1);
y = zeta(2);
theta = zeta(3);

width = length(zeta);
num_landmark = length(ids);

Cid_landmark = [cos(theta),sin(theta);...
    -sin(theta),cos(theta)];

C = zeros(2*num_landmark,width);
err = zeros(2*num_landmark, 1);

Pinv = [cos(theta), -sin(theta), x;
    sin(theta), cos(theta),  y;
    0,          0,       1  ]^(-1);

% Hashmap
m = java.util.HashMap;
for i = 1:length(state_ids)
    m.put(state_ids(i),i);
end

% comupte C and error
for k = 1:num_landmark
    % lms seen from robot
    lm = lms(:,k);
    id = ids(k);
    % record the place of landmark
    place = m.get(id);
    
    % last time landmarks
    lx = zeta(2*place+2);
    ly = zeta(2*place+3);
    
    zx = lm(1);
    zy = lm(2);
    % compute C
    %        X             Y                        theta
    Cid_robot = [-cos(theta), -sin(theta),     sin(theta)*(x-lx)-cos(theta)*(y-ly);...
        sin(theta),  -cos(theta),     cos(theta)*(x-lx)+sin(theta)*(y-ly)];
    
    Cid = [Cid_robot,zeros(2,2*place-2),Cid_landmark,zeros(2,width-3-2*place)];
    
    % C_(t+1)
    C([2*k-1,2*k],:) = Cid;
    
    % compute error
    z_hat = Pinv * [lx; ly; 1];
    err([2*k-1,2*k]) = [z_hat(1)-zx; ...
                        z_hat(2)-zy];
end

%  Q_total
Q_total = eye(2*num_landmark)*Q(1,1);

% compute K
K = Sigma * C' * (C * Sigma * C' + Q_total)^(-1);

% update covariance
Sigma = (eye(width) - K * C) * Sigma;

% update state
zeta = zeta - K * err;

end