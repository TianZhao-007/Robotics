function drive_circle(pb, radius)
% DRIVE_CIRCLE send a sequence of commands to the pibot to make it drive in a circle.

% pb is the pibot instance to send commands
% radius is the radius of the circle to drive

t = 20;
q = 2*pi/t;
u = 2*pi*radius/t;

[wl,wr] = inverse_kinematics(u,q);

pb.setVelocity([wl,wr]);
pb.simulate(t);
end