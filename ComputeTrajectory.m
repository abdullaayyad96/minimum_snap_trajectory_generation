function [total_cost, polys_x, polys_y, polys_z] = ComputeTrajectory(ts, waypts, n_order)

%% conditions
v0 = [0,0];
a0 = [0,0];
v1 = [0,0];
a1 = [0,0];

%% trajectory plan
%independantly solve for x and y
[polys_x, cost_x] = minimum_snap_single_axis_simple(waypts(1,:),ts,n_order,v0(1),a0(1),v1(1),a1(1));
[polys_y, cost_y] = minimum_snap_single_axis_simple(waypts(2,:),ts,n_order,v0(2),a0(2),v1(2),a1(2));
[polys_z, cost_z] = minimum_snap_single_axis_simple(waypts(3,:),ts,n_order,v0(2),a0(2),v1(2),a1(2));

total_cost = cost_x^2 + cost_y^2 + cost_z^2;

end

function [polys, cost] = minimum_snap_single_axis_simple(waypts,ts,n_order,v0,a0,ve,ae)
p0 = waypts(1);
pe = waypts(end);

n_poly = length(waypts)-1;
n_coef = cumsum(n_order+1);

% compute Q: Q incorporates the hessian matrix for all the segments in a
% block diagonal manner. It serves as the cost function for the QP problem
Q_all = [];
for i=1:n_poly
    Q_all = blkdiag(Q_all,computeQ_ayyad(n_order(i),4,ts(i),ts(i+1)));
end
b_all = zeros(size(Q_all,1),1);

%constraints on states and their derivates
%For each polynomial trajectory, constraints on [1-position at goal
%waypoint 2-position continuity, 3-velocity continuity, 4-acceleration,
%5-jerk continuity].
%Additionally, 3 constraints on initial states and 3 on terminal states
%acceleration
Aeq = zeros(5*(n_poly-1)+6,n_coef(end));
beq = zeros(5*(n_poly-1)+6,1);

% start/terminal position-velocity-acceleration constraints  (6 equations)
Aeq(1:3,1:n_coef(1)) = [calc_tvec(ts(1),n_order(1),0);
                     calc_tvec(ts(1),n_order(1),1);
                     calc_tvec(ts(1),n_order(1),2)];
Aeq(4:6,n_coef(end-1)+1:n_coef(end)) = ...
                    [calc_tvec(ts(end),n_order(end),0);
                     calc_tvec(ts(end),n_order(end),1);
                     calc_tvec(ts(end),n_order(end),2)];
beq(1:6,1) = [p0,v0,a0,pe,ve,ae]';

% position constraints on middle waypoints   (n_ploy-1 equations)
neq = 6;
for i=1:n_poly-1
    neq=neq+1;
    Aeq(neq,n_coef(i)+1:n_coef(i+1)) = calc_tvec(ts(i+1),n_order(i+1),0);
    beq(neq) = waypts(i+1);
end

% continuous constraints  ((n_poly-1)*3 equations)
for i=1:n_poly-1
    tvec_p1 = calc_tvec(ts(i+1),n_order(i),0);
    tvec_v1 = calc_tvec(ts(i+1),n_order(i),1);
    tvec_a1 = calc_tvec(ts(i+1),n_order(i),2);    
    tvec_j1 = calc_tvec(ts(i+1),n_order(i),3);
    tvec_p2 = calc_tvec(ts(i+1),n_order(i+1),0);
    tvec_v2 = calc_tvec(ts(i+1),n_order(i+1),1);
    tvec_a2 = calc_tvec(ts(i+1),n_order(i+1),2);    
    tvec_j2 = calc_tvec(ts(i+1),n_order(i+1),3);
    neq=neq+1;
    if i<2
        start_index=1;
    else
        start_index = n_coef(i-1)+1;
    end
    Aeq(neq,start_index:n_coef(i+1))=[tvec_p1,-tvec_p2];
    neq=neq+1;
    Aeq(neq,start_index:n_coef(i+1))=[tvec_v1,-tvec_v2];
    neq=neq+1;
    Aeq(neq,start_index:n_coef(i+1))=[tvec_a1,-tvec_a2];
    neq=neq+1;
    Aeq(neq,start_index:n_coef(i+1))=[tvec_j1,-tvec_j2];
end

Aieq = [];
bieq = [];

options = optimoptions('quadprog','Display','final', 'ConstraintTolerance', 1e-12, 'MaxIterations', 1000, 'Algorithm', 'trust-region-reflective');
[p, cost] = quadprog(Q_all,b_all,Aieq,bieq,Aeq,beq, [], [], [], options);

polys = {};
for i=1:n_poly
    if i<2
        start_index=1;
    else
        start_index = n_coef(i-1)+1;
    end
    polys{i} = p(start_index:n_coef(i));
end

end
