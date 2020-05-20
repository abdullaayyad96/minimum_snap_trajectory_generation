%%
% conditions
waypts = [0,0,1;
      0.75,0.75, 1;
      -0.75, 0.75, 1;
      0.75, -0.75, 1;
      -0.75, -0.75, 1;
      0, 0, 1]';

n_order = 6; %Order of polynomial

T = 10; %Total time for the trajectory
ts_initial = arrangeT(waypts,T)'; %Timestamp for each segment

%%
% Constraint matrices

%increasing time
A = zeros(length(ts_initial)-1, length(ts_initial));
b = - 0.1 * ones(length(ts_initial)-1, 1);
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
options = optimoptions('fmincon','Display','final', 'OptimalityTolerance', 1e-6, 'Algorithm', 'interior-point', 'MaxIterations', 10);
[optimal_ts, optimal_cost] = fmincon(@(sol)ComputeTrajectory(sol, waypts, n_order), linspace(0,T,length(ts_initial))', A, b, Aeq, beq, [], [], [], options); 

cost_1 = ComputeTrajectory(ts_initial, waypts, n_order);
cost_2 = ComputeTrajectory(optimal_ts, waypts, n_order);
cost_1
cost_2

%%
ts = optimal_ts;
[~, polys_x, polys_y, polys_z] = ComputeTrajectory(ts, waypts, n_order);

%% result show
figure(1)
plot(waypts(1,:),waypts(2,:),'*r');hold on;
plot(waypts(1,:),waypts(2,:),'b--');
title('minimum snap trajectory');
color = ['grc'];
for i=1:size(polys_x,2)
    tt = ts(i):0.01:ts(i+1);
    xx = polys_vals(polys_x,ts,tt,0);
    yy = polys_vals(polys_y,ts,tt,0);
    plot(xx,yy,color(mod(i,3)+1));
end


