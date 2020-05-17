function demo1_minimum_snap_simple()
    clear,clc;

    %% condition
%% condition
    waypts = [0,0;
          1,2;
          2,0;
          4,5;
          5,2]';
      
    v0 = [0,0];
    a0 = [0,0];
    v1 = [0,0];
    a1 = [0,0];
    T = 5; %Total time for the trajectory
    ts = arrangeT(waypts,T); %Timestamp for each segment
    n_order = 5; %Order of polynomial
    
    %% trajectory plan
    %independantly solve for x and y
    polys_x = minimum_snap_single_axis_simple(waypts(1,:),ts,n_order,v0(1),a0(1),v1(1),a1(1));
    polys_y = minimum_snap_single_axis_simple(waypts(2,:),ts,n_order,v0(2),a0(2),v1(2),a1(2));
    
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

    figure(2)
    vxx = polys_vals(polys_x,ts,tt,1);
    axx = polys_vals(polys_x,ts,tt,2);
    jxx = polys_vals(polys_x,ts,tt,3);
    vyy = polys_vals(polys_y,ts,tt,1);
    ayy = polys_vals(polys_y,ts,tt,2);
    jyy = polys_vals(polys_y,ts,tt,3);
    
    subplot(421),plot(tt,xx);title('x position');
    subplot(422),plot(tt,yy);title('y position');
    subplot(423),plot(tt,vxx);title('x velocity');
    subplot(424),plot(tt,vyy);title('y velocity');
    subplot(425),plot(tt,axx);title('x acceleration');
    subplot(426),plot(tt,ayy);title('y acceleration');
    subplot(427),plot(tt,jxx);title('x jerk');
    subplot(428),plot(tt,jyy);title('y jerk');
end


function polys = minimum_snap_single_axis_simple(waypts,ts,n_order,v0,a0,ve,ae)
p0 = waypts(1);
pe = waypts(end);

n_poly = length(waypts)-1;
n_coef = n_order+1;

% compute Q: Q incorporates the hessian matrix for all the segments in a
% block diagonal manner. It serves as the cost function for the QP problem
Q_all = [];
for i=1:n_poly
    Q_all = blkdiag(Q_all,computeQ_ayyad(n_order,4,ts(i),ts(i+1)));
    Q_old = computeQ(n_order,3,ts(i),ts(i+1));
    Q_ayyad = computeQ_ayyad(n_order,3,ts(i),ts(i+1));
end
b_all = zeros(size(Q_all,1),1);

%constraints on states and their derivates
%For each polynomial trajectory, constraints on [1-position at goal
%waypoint 2-position continuity, 3-velocity continuity, 4-acceleration
%continuity].
%Additionally, 3 constraints on initial states and 3 on terminal states
%acceleration
Aeq = zeros(4*(n_poly-1)+6,n_coef*n_poly);
beq = zeros(4*(n_poly-1)+6,1);

% start/terminal position-velocity-acceleration constraints  (6 equations)
Aeq(1:3,1:n_coef) = [calc_tvec(ts(1),n_order,0);
                     calc_tvec(ts(1),n_order,1);
                     calc_tvec(ts(1),n_order,2)];
Aeq(4:6,n_coef*(n_poly-1)+1:n_coef*n_poly) = ...
                    [calc_tvec(ts(end),n_order,0);
                     calc_tvec(ts(end),n_order,1);
                     calc_tvec(ts(end),n_order,2)];
beq(1:6,1) = [p0,v0,a0,pe,ve,ae]';

% position constraints on middle waypoints   (n_ploy-1 equations)
neq = 6;
for i=1:n_poly-1
    neq=neq+1;
    Aeq(neq,n_coef*i+1:n_coef*(i+1)) = calc_tvec(ts(i+1),n_order,0);
    beq(neq) = waypts(i+1);
end

% continuous constraints  ((n_poly-1)*3 equations)
for i=1:n_poly-1
    tvec_p = calc_tvec(ts(i+1),n_order,0);
    tvec_v = calc_tvec(ts(i+1),n_order,1);
    tvec_a = calc_tvec(ts(i+1),n_order,2);
    neq=neq+1;
    Aeq(neq,n_coef*(i-1)+1:n_coef*(i+1))=[tvec_p,-tvec_p];
    neq=neq+1;
    Aeq(neq,n_coef*(i-1)+1:n_coef*(i+1))=[tvec_v,-tvec_v];
    neq=neq+1;
    Aeq(neq,n_coef*(i-1)+1:n_coef*(i+1))=[tvec_a,-tvec_a];
end

Aieq = [];
bieq = [];

p = quadprog(Q_all,b_all,Aieq,bieq,Aeq,beq);

polys = reshape(p,n_coef,n_poly);

end

