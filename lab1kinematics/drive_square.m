function drive_square(pb, sideLength)
% DRIVE_SQUARE send a sequence of commands to the pibot to make it drive in a square.

% pb is the pibot instance to send commands
% sideLength is the length of each side of the square to drive
t = sideLength/0.25;
a = 0;
for i=0:4
    if(a<4)
        [wl,wr] = inverse_kinematics(0.25,0);
        pb.setVelocity([wl,wr],t);
        pb.setVelocity([-10,10],2.2999);
    end        
     a = a+1;
end





end