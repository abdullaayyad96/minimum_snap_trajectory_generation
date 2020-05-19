%%
% conditions
waypts = [0,0,1;
      1,1, 1;
      -1,1, 1;
      0.75,-0.75, 1;
      -0.75,-0.75, 1;
      0, 0, 1]';

n_order = 6; %Order of polynomial

T = 10; %Total time for the trajectory
ts_initial = arrangeT(waypts,T)'; %Timestamp for each segment

%%
% Constraint matrices

%increasing time
A = zeros(length(ts_initial)-1, length(ts_initial));
b = zeros(length(ts_initial)-1, 1);
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
[optimal_ts, optimal_cost] = fmincon(@(sol)ComputeTrajectory(sol, waypts, n_order), ts_initial, A, b, Aeq, beq); 

ts = optimal_ts;
[~, polys_x, polys_y, polys_z] = ComputeTrajectory(ts, waypts, n_order);
