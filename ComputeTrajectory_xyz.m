function [cost, polys_x, polys_y, polys_z]  = ComputeTrajectory_xyz(ts, waypts, n_order)
%%inputs:
%   - ts: Nx1 vector on the timestamp for each waypoint
%   - waypts: Nx3 on the xyz waypoints
%   - n_order: Nx1 on the order of the x,y and z polynomials
%% conditions
v0 = [0,0,0];
a0 = [0,0,0];
v1 = [0,0,0];
a1 = [0,0,0];

%% trajectory plan
%independantly solve for x and y
[polys, cost] = minimum_snap_3_axis(waypts, ts, n_order, v0, a0, v1, a1);

polys_x = {polys{1, :}};
polys_y = {polys{2, :}};
polys_z = {polys{3, :}};
end

function [polys, cost] = minimum_snap_3_axis(waypts,ts,n_order,v0,a0,ve,ae)
p0 = waypts(:, 1);
pe = waypts(:, end);

n_poly = size(waypts, 2) - 1;
n_coef = cumsum(n_order+1);

% compute Q: Q incorporates the hessian matrix for all the segments in a
% block diagonal manner. It serves as the cost function for the QP problem
Q_x_axis = [];
for i=1:n_poly
    Q_x_axis = blkdiag(Q_x_axis,computeQ_ayyad(n_order(i),4,ts(i),ts(i+1)));
end
%incorporate x, y, and z
Q_all = blkdiag(Q_x_axis,Q_x_axis, Q_x_axis);
b_all = zeros(size(Q_all,1),1);

%constraints on states and their derivates
%For each polynomial trajectory, constraints on [1-position at goal
%waypoint 2-position continuity, 3-velocity continuity, 4-acceleration,
%5-jerk continuity].
%Additionally, 3 constraints on initial states and 3 on terminal states
%acceleration


Aeq = [];
beq = [];

for j=1:3
    Aeq_blk = zeros(5*(n_poly-1)+6, n_coef(end));
    beq_blk = zeros(5*(n_poly-1)+6, 1);
    
    % start/terminal position-velocity-acceleration constraints  (6 equations)
    Aeq_blk(1:3,1:n_coef(1)) = [calc_tvec(ts(1),n_order(1),0);
                         calc_tvec(ts(1),n_order(1),1);
                         calc_tvec(ts(1),n_order(1),2)];
    Aeq_blk(4:6,n_coef(end-1)+1:n_coef(end)) = ...
                        [calc_tvec(ts(end),n_order(end),0);
                         calc_tvec(ts(end),n_order(end),1);
                         calc_tvec(ts(end),n_order(end),2)];
    beq_blk(1:6,1) = [p0(j), v0(j), a0(j), pe(j), ve(j), ae(j)]';

    % position constraints on middle waypoints   (n_ploy-1 equations)
    neq = 6;
    for i=1:n_poly-1
        neq=neq+1;
        Aeq_blk(neq,n_coef(i)+1:n_coef(i+1)) = calc_tvec(ts(i+1),n_order(i+1),0);
        beq_blk(neq) = waypts(j, i+1);
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
        Aeq_blk(neq,start_index:n_coef(i+1))=[tvec_p1,-tvec_p2];
        neq=neq+1;
        Aeq_blk(neq,start_index:n_coef(i+1))=[tvec_v1,-tvec_v2];
        neq=neq+1;
        Aeq_blk(neq,start_index:n_coef(i+1))=[tvec_a1,-tvec_a2];
        neq=neq+1;
        Aeq_blk(neq,start_index:n_coef(i+1))=[tvec_j1,-tvec_j2];
    end
    
    Aeq=blkdiag(Aeq, Aeq_blk);
    beq = [beq; beq_blk];
end

Aieq = [];
bieq = [];

options = optimoptions('quadprog','Display','final', 'ConstraintTolerance', 1e-9, 'MaxIterations', 1000, 'Algorithm', 'trust-region-reflective');
[p_initial, cost] = quadprog(Q_all,b_all,Aieq,bieq,Aeq,beq, [], [], [], options);

cost_func = @(x) 0.5 * x' * Q_all * x + b_all' * x;
options_fmin = optimoptions('fmincon','Display','final', 'MaxFunctionEvaluations', 10000, 'StepTolerance', 1e-15, 'algorithm', 'sqp');
[p, cost] = fmincon(cost_func, p_initial, Aieq, bieq, Aeq, beq, [], [], @(x) attitude_contraint(x, 2, ts, n_order, 'pitch'), options_fmin);
attitude_contraint(p, 2, ts, n_order, 'pitch')
% p = p_initial;

polys = {};
for j=1:3 
    offset = (j-1) * n_coef(end);
    for i=1:n_poly
        if i<2
            start_index=1;
        else
            start_index = n_coef(i-1)+1;
        end
        polys{j, i} = p((offset + start_index): (offset + n_coef(i)));
    end
end

end
