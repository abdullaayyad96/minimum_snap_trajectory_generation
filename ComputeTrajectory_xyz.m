function [cost, polys_x, polys_y, polys_z]  = ComputeTrajectory_xyz(ts, waypts, n_order, v_forced, t_v_forced, a_forced, t_a_forced, forced_roll, t_forced_roll, larger_than_p, t_larger_than_p, less_than_a, t_less_than_a, less_than_j, t_less_than_j, less_than_s, t_less_than_s);
%%inputs:
%   - ts: Nx1 vector on the timestamp for each waypoint
%   - waypts: Nx3 on the xyz waypoints
%   - n_order: Nx1 on the order of the x,y and z polynomials
%% conditions
v0 = [0,0,0];
a0 = [0,0,0];
j0 = [0,0,0];
v1 = [0,0,0];
a1 = [0,0,0];
j1 = [0,0,0];

%% trajectory plan
%independantly solve for x and y
[polys, cost] = minimum_snap_3_axis(waypts, ts, n_order, v0, a0, j0, v1, a1, j1, v_forced, t_v_forced, a_forced, t_a_forced, forced_roll, t_forced_roll, larger_than_p, t_larger_than_p, less_than_a, t_less_than_a, less_than_j, t_less_than_j, less_than_s, t_less_than_s);

polys_x = {polys{1, :}};
polys_y = {polys{2, :}};
polys_z = {polys{3, :}};
end

function [polys, cost] = minimum_snap_3_axis(waypts,ts,n_order,v0,a0,j0,ve,ae,je, v_forced, t_v_forced, a_forced, t_a_forced, forced_roll, t_forced_roll, larger_than_p, t_larger_than_p, less_than_a, t_less_than_a, less_than_j, t_less_than_j, less_than_s, t_less_than_s)
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
Q_y_axis = [];
for i=1:n_poly
    Q_y_axis = blkdiag(Q_y_axis,computeQ_ayyad(n_order(i),4,ts(i),ts(i+1)));
end
Q_z_axis = [];
for i=1:n_poly
    Q_z_axis = blkdiag(Q_z_axis,computeQ_ayyad(n_order(i),4,ts(i),ts(i+1)));
end
%incorporate x, y, and z
Q_all = blkdiag(Q_x_axis,Q_y_axis, Q_z_axis);
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
    
    if j<5
        Aeq_blk = zeros(5*(n_poly-1)+8, n_coef(end));
        beq_blk = zeros(5*(n_poly-1)+8, 1);
    
        % start/terminal position-velocity-acceleration-jerk constraints  (8 equations)
        Aeq_blk(1:4,1:n_coef(1)) = [calc_tvec(ts(1),n_order(1),0);
                             calc_tvec(ts(1),n_order(1),1);
                             calc_tvec(ts(1),n_order(1),2);                         
                             calc_tvec(ts(1),n_order(1),3)];
        Aeq_blk(5:8,n_coef(end-1)+1:n_coef(end)) = ...
                            [calc_tvec(ts(end),n_order(end),0);
                             calc_tvec(ts(end),n_order(end),1);
                             calc_tvec(ts(end),n_order(end),2);
                             calc_tvec(ts(end),n_order(end),3)];
        beq_blk(1:8,1) = [p0(j), v0(j), a0(j), j0(j), pe(j),  ve(j), ae(j), je(j)]';

        % position constraints on middle waypoints   (n_ploy-1 equations)
        neq = 8;
    else
        Aeq_blk = zeros(5*(n_poly-1)+6, n_coef(end));
        beq_blk = zeros(5*(n_poly-1)+6, 1);
    
        % start/terminal position-velocity-acceleration-jerk constraints  (8 equations)
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
    end
    
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
        tvec_s1 = calc_tvec(ts(i+1),n_order(i),4);
        tvec_p2 = calc_tvec(ts(i+1),n_order(i+1),0);
        tvec_v2 = calc_tvec(ts(i+1),n_order(i+1),1);
        tvec_a2 = calc_tvec(ts(i+1),n_order(i+1),2);    
        tvec_j2 = calc_tvec(ts(i+1),n_order(i+1),3);
        tvec_s2 = calc_tvec(ts(i+1),n_order(i+1),4);
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
%         neq=neq+1;
%         Aeq_blk(neq,start_index:n_coef(i+1))=[tvec_s1,-tvec_s2];
    end
    
    %velocity constraints
    for iterator=1:length(t_v_forced)
        if ~isnan(v_forced(j, iterator))
            %determine trajectory segment
            trajectory_segment = 0;
            for i=1:n_poly
                if (t_v_forced(iterator) > ts(i)) && (t_v_forced(iterator)  <= ts(i+1))
                    trajectory_segment = i;
                    break
                end
            end            
            if trajectory_segment<2
                start_index=1;
            else
                start_index = n_coef(trajectory_segment-1)+1;
            end
            neq=neq+1;
            Aeq_blk(neq,start_index:n_coef(trajectory_segment))=calc_tvec(t_v_forced(iterator),n_order(trajectory_segment),1);
            beq_blk(neq) = v_forced(j, iterator);
        end
    end
    
    %acceleration constraints
    for iterator=1:length(t_a_forced)
        if ~isnan(a_forced(j, iterator))
            %determine trajectory segment
            trajectory_segment = 0;
            for i=1:n_poly
                if (t_a_forced(iterator) > ts(i)) && (t_a_forced(iterator)  <= ts(i+1))
                    trajectory_segment = i;
                    break
                end
            end            
            if trajectory_segment<2
                start_index=1;
            else
                start_index = n_coef(trajectory_segment-1)+1;
            end
            neq=neq+1;
            Aeq_blk(neq,start_index:n_coef(trajectory_segment))=calc_tvec(t_a_forced(iterator),n_order(trajectory_segment),2);
            beq_blk(neq) = a_forced(j, iterator);
        end
    end
        
    Aeq=blkdiag(Aeq, Aeq_blk);
    beq = [beq; beq_blk];
end


%roll angle constraints
for iterator=1:length(t_forced_roll)
    if ~isnan(forced_roll(iterator))
        Aeq_line = zeros(1, size(Aeq, 2));
        %determine trajectory segment
        trajectory_segment = 0;
        for i=1:n_poly
            if (t_forced_roll(iterator) > ts(i)) && (t_forced_roll(iterator)  <= ts(i+1))
                trajectory_segment = i;
                break
            end
        end            
        if trajectory_segment<2
            y_start_index = n_coef(end) + 1;
            z_start_index = 2 * n_coef(end) + 1;
        else
            y_start_index = n_coef(end) + n_coef(trajectory_segment-1)+1;
            z_start_index = 2 * n_coef(end) + n_coef(trajectory_segment-1)+1;
        end
        y_end_index = n_coef(end) + n_coef(trajectory_segment);
        z_end_index = 2 * n_coef(end) + n_coef(trajectory_segment);
        neq=neq+1;
        
        Aeq_line(y_start_index:y_end_index)=calc_tvec(ts(2),n_order(trajectory_segment),2);
        Aeq_line(z_start_index:z_end_index)=-tan(-forced_roll(iterator))*calc_tvec(t_forced_roll(iterator),n_order(trajectory_segment),2);
        
        Aeq = [Aeq; Aeq_line];
        beq = [beq; tan(-forced_roll(iterator))*9.81];
    end
end


Aieq = [];
bieq = [];


nieq = 0;
for j=1:3
     Aieq_blk = zeros(0, n_coef(end));
     bieq_blk = zeros(0, 1);   
    
    %position_inequality constraints
    for iterator=1:length(t_larger_than_p)
        if ~isnan(larger_than_p(j, iterator))
            %determine trajectory segment
            trajectory_segment = 0;
            for i=1:n_poly
                if (t_larger_than_p(iterator) >= ts(i)) && (t_larger_than_p(iterator)  <= ts(i+1))
                    trajectory_segment = i;
                    break
                end
            end            
            if trajectory_segment<2
                start_index=1;
            else
                start_index = n_coef(trajectory_segment-1)+1;
            end
            nieq=nieq+1;
            Aieq_blk(nieq,start_index:n_coef(trajectory_segment))=-calc_tvec(t_larger_than_p(iterator),n_order(trajectory_segment),0);
            bieq_blk(nieq, 1) = -larger_than_p(j, iterator);
        end
    end
    
    %acceleration inqueality constraints
    for iterator=1:length(t_less_than_a)
        if ~isnan(less_than_a(j, iterator))
            %determine trajectory segment
            trajectory_segment = 0;
            for i=1:n_poly
                if (t_less_than_a(iterator) >= ts(i)) && (t_less_than_a(iterator)  <= ts(i+1))
                    trajectory_segment = i;
                    break
                end
            end            
            if trajectory_segment<2
                start_index=1;
            else
                start_index = n_coef(trajectory_segment-1)+1;
            end
            nieq=nieq+1;
            Aieq_blk(nieq,start_index:n_coef(trajectory_segment))=calc_tvec(t_less_than_a(iterator),n_order(trajectory_segment),2);
            bieq_blk(nieq, 1) = less_than_a(j, iterator);
            nieq=nieq+1;
            Aieq_blk(nieq,start_index:n_coef(trajectory_segment))=-calc_tvec(t_less_than_a(iterator),n_order(trajectory_segment),2);
            bieq_blk(nieq, 1) = less_than_a(j, iterator);
        end
    end
        
    
    %jerk inqueality constraints
    for iterator=1:length(t_less_than_j)
        if ~isnan(less_than_j(j, iterator))
            %determine trajectory segment
            trajectory_segment = 0;
            for i=1:n_poly
                if (t_less_than_j(iterator) >= ts(i)) && (t_less_than_j(iterator)  <= ts(i+1))
                    trajectory_segment = i;
                    break
                end
            end            
            if trajectory_segment<2
                start_index=1;
            else
                start_index = n_coef(trajectory_segment-1)+1;
            end
            nieq=nieq+1;
            Aieq_blk(nieq,start_index:n_coef(trajectory_segment))=calc_tvec(t_less_than_j(iterator),n_order(trajectory_segment),3);
            bieq_blk(nieq, 1) = less_than_j(j, iterator);
            nieq=nieq+1;
            Aieq_blk(nieq,start_index:n_coef(trajectory_segment))=-calc_tvec(t_less_than_j(iterator),n_order(trajectory_segment),3);
            bieq_blk(nieq, 1) = less_than_j(j, iterator);
        end
    end
    
    %snap inqueality constraints
    for iterator=1:length(t_less_than_s)
        if ~isnan(less_than_s(j, iterator))
            %determine trajectory segment
            trajectory_segment = 0;
            for i=1:n_poly
                if (t_less_than_s(iterator) >= ts(i)) && (t_less_than_s(iterator)  <= ts(i+1))
                    trajectory_segment = i;
                    break
                end
            end            
            if trajectory_segment<2
                start_index=1;
            else
                start_index = n_coef(trajectory_segment-1)+1;
            end
            nieq=nieq+1;
            Aieq_blk(nieq,start_index:n_coef(trajectory_segment))=calc_tvec(t_less_than_s(iterator),n_order(trajectory_segment),4);
            bieq_blk(nieq, 1) = less_than_s(j, iterator);
            nieq=nieq+1;
            Aieq_blk(nieq,start_index:n_coef(trajectory_segment))=-calc_tvec(t_less_than_s(iterator),n_order(trajectory_segment),4);
            bieq_blk(nieq, 1) = less_than_s(j, iterator);
        end
    end
    
    Aieq=blkdiag(Aieq, Aieq_blk);
    bieq = [bieq; bieq_blk];
end

options = optimoptions('quadprog','Display','final', 'ConstraintTolerance', 1e-9, 'MaxIterations', 1e4, 'Algorithm', 'interior-point-convex');
[p_initial, cost] = quadprog(Q_all,b_all,Aieq,bieq,Aeq,beq, [], [], [], options);

cost_func = @(x) 0.5 * x' * Q_all * x + b_all' * x;
options_fmin = optimoptions('fmincon','Display','final', 'MaxFunctionEvaluations', 4e3, 'MaxFunctionEvaluations', 100000, 'StepTolerance', 1e-20, 'algorithm', 'active-set');
% [p, cost] = fmincon(cost_func, p_initial, Aieq, bieq, Aeq, beq, [], [], @(x) trajectoryConstraints(x, t_forced_roll, forced_roll, ts, n_order), options_fmin);
%[p, cost] = fmincon(cost_func, p_initial, Aieq, bieq, Aeq, beq, [], [], @(x) shape_constraints(x, ts, n_order), options_fmin);
p = p_initial;
%trajectoryConstraints(p, ts(2), forced_roll, ts, n_order);

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
