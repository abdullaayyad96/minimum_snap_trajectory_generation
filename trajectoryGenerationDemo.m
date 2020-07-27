%%
% conditions
waypts = [-3, 0, 0.6;
           -0.75, 1, 1.5;
           1.5, 0, 0.6]';
  
T = 3; %Total time for the trajectory
ts = arrangeT(waypts,T)'; %Timestamp for each segment

v_forced = [2.7, 0, NaN;
            NaN, 0, NaN]';
        
t_v_forced = [];
        
a_forced = [0, NaN, NaN;
            0, NaN, -9.81;
            0, NaN, NaN]';

t_a_forced = [ts(2)-0.03; ts(2); ts(2)+0.03];

forced_roll = [85*pi/180; 85*pi/180]';

t_forced_roll = [];%[ts(2)-0.03; ts(2)+0.03];%[ts(2)-0.03, ts(2)+0.03];%[ts(2)-0.01; ts(2)+0.01];


%inequality constraints
larger_than_p = [NaN, NaN 0.5;
                    NaN, NaN 0.5;
                    NaN, NaN 0.5;
                    NaN, NaN 0.5;
                    NaN, NaN 0.5;
                    NaN, NaN 0.5;
                    NaN, NaN 0.5;
                    NaN, NaN 0.5;
                    NaN, NaN 0.5;
                    NaN, NaN 0.5;
                    NaN, NaN 0.5;
                    NaN, NaN 0.5;
                    NaN, NaN 0.5;
                    NaN, NaN 0.5;
                    NaN, NaN 0.5]';

t_larger_than_p = linspace(0.1, T-0.1, 10);

%acceleration inequality constraints
max_acc = -2.6;
less_than_a = [NaN, max_acc, NaN;
                    NaN, NaN, -9;
                    NaN, NaN, NaN;
                    NaN, NaN, NaN]';

t_less_than_a = [];

%jerk inequality constraints
max_jerk = 30;
less_than_j = [max_jerk, max_jerk, max_jerk;
                    max_jerk, max_jerk, max_jerk;
                    max_jerk, max_jerk, max_jerk;
                    max_jerk, max_jerk, max_jerk;
                    max_jerk, max_jerk, max_jerk;
                    max_jerk, max_jerk, max_jerk;
                    max_jerk, max_jerk, max_jerk;
                    max_jerk, max_jerk, max_jerk;
                    max_jerk, max_jerk, max_jerk;
                    max_jerk, max_jerk, max_jerk;
                    max_jerk, max_jerk, max_jerk;
                    max_jerk, max_jerk, max_jerk;
                    max_jerk, max_jerk, max_jerk;
                    max_jerk, max_jerk, max_jerk;
                    max_jerk, max_jerk, max_jerk;
                    max_jerk, max_jerk, max_jerk;
                    max_jerk, max_jerk, max_jerk;
                    max_jerk, max_jerk, max_jerk;
                    max_jerk, max_jerk, max_jerk;
                    max_jerk, max_jerk, max_jerk;
                    max_jerk, max_jerk, max_jerk;
                    max_jerk, max_jerk, max_jerk;
                    max_jerk, max_jerk, max_jerk;
                    max_jerk, max_jerk, max_jerk;
                    max_jerk, max_jerk, max_jerk]';

t_less_than_j = linspace(0.1, T-0.1, 15);


%jerk inequality constraints
max_snap = 300;
less_than_s = [max_snap, max_snap, max_snap;
                    max_snap, max_snap, max_snap;
                    max_snap, max_snap, max_snap;
                    max_snap, max_snap, max_snap;
                    max_snap, max_snap, max_snap;
                    max_snap, max_snap, max_snap;
                    max_snap, max_snap, max_snap;
                    max_snap, max_snap, max_snap;
                    max_snap, max_snap, max_snap;
                    max_snap, max_snap, max_snap;
                    max_snap, max_snap, max_snap;
                    max_snap, max_snap, max_snap;
                    max_snap, max_snap, max_snap;
                    max_snap, max_snap, max_snap;
                    max_snap, max_snap, max_snap;
                    max_snap, max_snap, max_snap;
                    max_snap, max_snap, max_snap;
                    max_snap, max_snap, max_snap;
                    max_snap, max_snap, max_snap;
                    max_snap, max_snap, max_snap;
                    max_snap, max_snap, max_snap;
                    max_snap, max_snap, max_snap;
                    max_snap, max_snap, max_snap;
                    max_snap, max_snap, max_snap;
                    max_snap, max_snap, max_snap;
                    max_snap, max_snap, max_snap;
                    max_snap, max_snap, max_snap;
                    max_snap, max_snap, max_snap;
                    max_snap, max_snap, max_snap;
                    max_snap, max_snap, max_snap;
                    max_snap, max_snap, max_snap;
                    max_snap, max_snap, max_snap;
                    max_snap, max_snap, max_snap;
                    max_snap, max_snap, max_snap;
                    max_snap, max_snap, max_snap]';

t_less_than_s = linspace(0+0.1, T-0.1, 15);
t_less_than_s = [];

n_order = [10;
           10]; %Order of polynomial for each trajectory segment

    

%%
%optimize time
ts_initial = ts;

% Constraint matrices
%increasing time
A = zeros(length(ts_initial)-1, length(ts_initial));
b = - 0.2 * ones(length(ts_initial)-1, 1);
for i=1:length(ts_initial)-1
    A(i, i:i+1) = [1 -1];
end

%initial and final timet
Aeq = zeros(2, length(ts_initial));
Aeq(1,1) = 1;
Aeq(end, end) = 1;
beq = [0; T];

options = optimoptions('fmincon','Display','final', 'OptimalityTolerance', 1e-9, 'MaxIterations', 2);
[optimal_ts, optimal_cost] = fmincon(@(sol)ComputeTrajectory_xyz(sol, waypts, n_order, v_forced, t_v_forced, a_forced, t_a_forced, forced_roll, t_forced_roll, larger_than_p, t_larger_than_p, less_than_a, t_less_than_a, less_than_j, t_less_than_j, less_than_s, t_less_than_s), ts_initial, A, b, Aeq, beq, [], [], [], options); 

ts = optimal_ts;
%%
%optimize trajectory
[cost, polys_x, polys_y, polys_z] = ComputeTrajectory_xyz(ts, waypts, n_order, v_forced, t_v_forced, a_forced, t_a_forced, forced_roll, t_forced_roll, larger_than_p, t_larger_than_p, less_than_a, t_less_than_a, less_than_j, t_less_than_j, less_than_s, t_less_than_s);

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


