%%
% conditions
waypts = [0,0,1;
      0.5,-1, 2.5;
      1, 0, 1]';

n_order = [6;
           6]; %Order of polynomial for each trajectory segment

T = 4; %Total time for the trajectory
ts = arrangeT(waypts,T)'; %Timestamp for each segment

%%
%optimize trajectory
[cost, polys_x, polys_y, polys_z] = ComputeTrajectory_xyz(ts, waypts, n_order);
%% result show
figure(1)
plot(waypts(1,:),waypts(2,:),'*r');hold on;
plot(waypts(1,:),waypts(2,:),'b--');
title('minimum snap trajectory');
color = ['grc'];
for i=1:size(polys_x,2)
    
    tt = ts(i):0.01:ts(i+1);
    xx = polys_vals_cell(polys_x,ts,tt,0);
    yy = polys_vals_cell(polys_y,ts,tt,0);
    plot(xx,yy,color(mod(i,3)+1));
end


figure(2)
tt = ts(1):0.01:ts(end);
xx = polys_vals_cell(polys_x,ts,tt,0);
vxx = polys_vals_cell(polys_x,ts,tt,1);
axx = polys_vals_cell(polys_x,ts,tt,2);
jxx = polys_vals_cell(polys_x,ts,tt,3);
yy = polys_vals_cell(polys_y,ts,tt,0);
vyy = polys_vals_cell(polys_y,ts,tt,1);
ayy = polys_vals_cell(polys_y,ts,tt,2);
jyy = polys_vals_cell(polys_y,ts,tt,3);
zz = polys_vals_cell(polys_z,ts,tt,0);
vzz = polys_vals_cell(polys_z,ts,tt,1);
azz = polys_vals_cell(polys_z,ts,tt,2);
jzz = polys_vals_cell(polys_z,ts,tt,3);


subplot(431),plot(tt,xx);title('x position');
subplot(432),plot(tt,yy);title('y position');
subplot(433),plot(tt,zz);title('z position');
subplot(434),plot(tt,vxx);title('x velocity');
subplot(435),plot(tt,vyy);title('y velocity');
subplot(436),plot(tt,vzz);title('z velocity');
subplot(437),plot(tt,axx);title('x acceleration');
subplot(438),plot(tt,ayy);title('y acceleration');
subplot(439),plot(tt,azz);title('z acceleration');
subplot(4,3,10),plot(tt,jxx);title('x jerk');
subplot(4,3,11),plot(tt,jyy);title('y jerk');
subplot(4,3,12),plot(tt,jzz);title('z jerk');


