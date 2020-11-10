function armse = ARMSE(phat,landmarks)
% ------inputs------
% phat: currently observed landmarks positions
% landmarks: selected ground truth data corresponds to the observed landmarks
    n = size(phat,2);
    p = phat(1:2,:);
    
    % compute average of phat and ground truth(dimension = 2)
    uhat =  mean(p,2);
    u = mean(landmarks,2);
    Sigma = ((landmarks-u)*(p-uhat)')/n;
    [U,D,V] = svd(Sigma);
    
    if det(Sigma)>=0
        A = [1 0;0 1];
    else
        A = [1 0;0 -1];
    end
    
    Rs = V*A*U';
    xs = uhat - Rs*u;
    
%     armse = 0;
%     for i = 1:n
%         armse = armse + (norm(Rs'*(p(:,i)-xs)-landmarks(:,i)))^2;
%     end
%     armse = sqrt(armse/n);
    armse = sqrt(sum((Rs'*(p-xs) - landmarks).^2,'all')/n);

end