function [zeta,Sigma,state_ids] = augment(zeta,Sigma,R,lms,ids,state_ids)
% This function augment new landmark
x = zeta(1);
y = zeta(2);
theta = zeta(3);
% find out new landmarks
for i = 1:numel(ids)
    lm = lms(:,i);
    id = ids(i);
    len = length(zeta);
    if any(id == state_ids)
        continue
    end
    % If the landmark is new, add it to the state
    inertial_lm = convert_to_inertial(zeta(1:3), lm);
    N = numel(state_ids);
    assert(3+2*N == numel(zeta), "State vector size does not match state ids!");
    state_ids = [state_ids, id];
    zeta = [zeta; inertial_lm];
    zx = lm(1);
    zy = lm(2);
    
    Gz = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    Gzeta = [1, 0,-sin(theta)*zx-cos(theta)*zy,  zeros(1,len-3);...
        0, 1, cos(theta)*zx-sin(theta)*zy, zeros(1,len-3)];
    Sigma = [Sigma, Sigma*Gzeta';
        Gzeta*Sigma, Gzeta*Sigma*Gzeta'+Gz*R*Gz'];
end
end
