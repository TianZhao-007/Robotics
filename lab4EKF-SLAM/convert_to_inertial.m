function inertial_lm = convert_to_inertial(pose, lm)
th = pose(3);
R = [ cos(th), -sin(th);
      sin(th),  cos(th)];
position = pose(1:2,:);
inertial_lm = R * lm + position;
end
