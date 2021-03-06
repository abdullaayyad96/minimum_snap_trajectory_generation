%%
% conditions
waypts = [0, 0, 1;
      0.433, 0.5, 1;
      0.48, 0.8, 1;
      0.3923, 0.9, 1;
      0, 1, 1;
      -0.3923, 0.9, 1;
      -0.48, 0.8, 1;
      -0.433, 0.5, 1;
      0, 0, 1;
      0.433, -0.5, 1;
      0.48, -0.8, 1;
      0.3923, -0.9, 1;
      0, -1, 1;
      -0.3923, -0.9, 1;
      -0.48, -0.8, 1;
      -0.433, -0.5, 1;
      0, 0, 1]';

n_order = [7;
           7;
           7;
           7;
           7;
           7;
           7;
           7;
           7;
           7;
           7;
           7;
           7;
           7;
           7;
           7]; %Order of polynomial for each trajectory segment

T = 5.5; %Total time for the trajectory
ts_initial = arrangeT(waypts,T)'; %Timestamp for each segment

%%
% Constraint matrices

%increasing time
A = zeros(length(ts_initial)-1, length(ts_initial));
b = - 0.2 * ones(length(ts_initial)-1, 1);
for i=1:length(ts_initial)-1
    A(i, i:i+1) = [1 -1];
end

%initial and final time
Aeq = zeros(2, length(ts_initial));
Aeq(1,1) = 1;
Aeq(end, end) = 1;
beq = [0; T];

%%
%solve problem
options = optimoptions('fmincon','Display','final', 'OptimalityTolerance', 1e-9, 'MaxIterations', 10);
[optimal_ts, optimal_cost] = fmincon(@(sol)ComputeTrajectory(sol, waypts, n_order), ts_initial, A, b, Aeq, beq, [], [], [], options); 


cost_1 = ComputeTrajectory(ts_initial, waypts, n_order);
cost_2 = ComputeTrajectory(optimal_ts, waypts, n_order);
cost_1
cost_2

%%
ts = optimal_ts;
[~, polys_x, polys_y, polys_z] = ComputeTrajectory(ts, waypts, n_order);

%% result show
figure()
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


figure()
tt = ts(1):0.01:ts(end);
xx = polys_vals_cell(polys_x,ts,tt,0);
vxx = polys_vals_cell(polys_x,ts,tt,1);
axx = polys_vals_cell(polys_x,ts,tt,2);
jxx = polys_vals_cell(polys_x,ts,tt,3);
sxx = polys_vals_cell(polys_x,ts,tt,4);
yy = polys_vals_cell(polys_y,ts,tt,0);
vyy = polys_vals_cell(polys_y,ts,tt,1);
ayy = polys_vals_cell(polys_y,ts,tt,2);
jyy = polys_vals_cell(polys_y,ts,tt,3);
syy = polys_vals_cell(polys_y,ts,tt,4);
zz = polys_vals_cell(polys_z,ts,tt,0);
vzz = polys_vals_cell(polys_z,ts,tt,1);
azz = polys_vals_cell(polys_z,ts,tt,2);
jzz = polys_vals_cell(polys_z,ts,tt,3);
szz = polys_vals_cell(polys_z,ts,tt,4);


subplot(531),plot(tt,xx);title('x position');
subplot(532),plot(tt,yy);title('y position');
subplot(533),plot(tt,zz);title('z position');
subplot(534),plot(tt,vxx);title('x velocity');
subplot(535),plot(tt,vyy);title('y velocity');
subplot(536),plot(tt,vzz);title('z velocity');
subplot(537),plot(tt,axx);title('x acceleration');
subplot(538),plot(tt,ayy);title('y acceleration');
subplot(539),plot(tt,azz);title('z acceleration');
subplot(5,3,10),plot(tt,jxx);title('x jerk');
subplot(5,3,11),plot(tt,jyy);title('y jerk');
subplot(5,3,12),plot(tt,jzz);title('z jerk');
subplot(5,3,13),plot(tt,sxx);title('x snap');
subplot(5,3,14),plot(tt,syy);title('y snap');
subplot(5,3,15),plot(tt,szz);title('z snap');
