%%
% conditions
waypts = [-1.5,0,2;
      -0.25,-1, 1.5;
      0.25, -1, 1.5;
      1.5, -0.25, 2;
      1.5, 0.25, 2;
      0.25, 1, 2.5;
      -0.25, 1, 2.5;
      -1.5,0, 2]';

n_order = [6;
           6;
           6;
           1;
           6;
           6;
           6]; %Order of polynomial for each trajectory segment

T = 8; %Total time for the trajectory
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
[~, polys_x, polys_y, polys_z] = ComputeTrajectory(optimal_ts, waypts, n_order);

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

subplot(421),plot(tt,xx);title('x position');
subplot(422),plot(tt,yy);title('y position');
subplot(423),plot(tt,vxx);title('x velocity');
subplot(424),plot(tt,vyy);title('y velocity');
subplot(425),plot(tt,axx);title('x acceleration');
subplot(426),plot(tt,ayy);title('y acceleration');
subplot(427),plot(tt,jxx);title('x jerk');
subplot(428),plot(tt,jyy);title('y jerk');


