syms t
assume(t >= 0)
poly_order = 10;
x = sym('x',[1 poly_order], 'real');
polynomial = 0;


T = 5.5;
R = 2;

for i=1:poly_order
    polynomial = polynomial + x(i) * t^(i-1);
end
polynomial = 2*pi*polynomial / T - pi/2;

pos_x = R*cos(polynomial)*sin(polynomial);
vel_x = diff(pos_x, t, 1);
acc_x = diff(pos_x, t, 2);
jerk_x = diff(pos_x, t, 3);
snap_x = diff(pos_x, t, 4);

pos_y = R*cos(polynomial);
vel_y = diff(pos_y, t, 1);
acc_y = diff(pos_y, t, 2);
jerk_y = diff(pos_y, t, 3);
snap_y = diff(pos_y, t, 4);

snap = snap_x^2 + snap_y^2;
%%
snap_int = vpaintegral(snap, t, 0, T);

%%
%convert to function handles for speed
polynomial_f = matlabFunction(polynomial, 'Vars', {t, x});
pos_x_f = matlabFunction(pos_x, 'Vars', {t, x});
vel_x_f = matlabFunction(vel_x, 'Vars', {t, x});
acc_x_f = matlabFunction(acc_x, 'Vars', {t, x});
jerk_x_f = matlabFunction(jerk_x, 'Vars', {t, x});
pos_y_f = matlabFunction(pos_y, 'Vars', {t, x});
vel_y_f = matlabFunction(vel_y, 'Vars', {t, x});
acc_y_f = matlabFunction(acc_y, 'Vars', {t, x});
jerk_y_f = matlabFunction(jerk_y, 'Vars', {t, x});

snap_f = matlabFunction(snap, 'Vars', {t, x});

%%
%options = optimoptions('fsolve', 'Algorithm', 'levenberg-marquardt', 'StepTolerance', 1e-30);
options = optimoptions('fmincon', 'StepTolerance', 1e-12);

x0 = zeros(size(x));
x0(2) = 1;

x_solution = fmincon(@(x_subs) shape_8_snap(x_subs, x, snap_f, T), x0, [], [], [], [], [], [], @(x_subs) shape_8_cons(x_subs, x, t, polynomial_f, pos_x_f, vel_x_f, acc_x_f, jerk_x_f, pos_y_f, vel_y_f, acc_y_f, jerk_y_f, T), options);

%%
%plot
figure()
tt = 0:0.01:T;

xx = subs(subs(pos_x, x, x_solution), t, tt);
vxx = subs(subs(vel_x, x, x_solution), t, tt);
axx = subs(subs(acc_x, x, x_solution), t, tt);
jxx = subs(subs(jerk_x, x, x_solution), t, tt);
sxx = subs(subs(snap_x, x, x_solution), t, tt);
yy = subs(subs(pos_y, x, x_solution), t, tt);
vyy = subs(subs(vel_y, x, x_solution), t, tt);
ayy = subs(subs(acc_y, x, x_solution), t, tt);
jyy = subs(subs(jerk_y, x, x_solution), t, tt);
syy = subs(subs(snap_y, x, x_solution), t, tt);


subplot(521),plot(tt,xx);title('x position');
subplot(522),plot(tt,yy);title('y position');
subplot(523),plot(tt,vxx);title('x velocity');
subplot(524),plot(tt,vyy);title('y velocity');
subplot(525),plot(tt,axx);title('x acceleration');
subplot(526),plot(tt,ayy);title('y acceleration');
subplot(527),plot(tt,jxx);title('x jerk');
subplot(528),plot(tt,jyy);title('y jerk');
subplot(5,2,9),plot(tt,sxx);title('x snap');
subplot(5,2,10),plot(tt,syy);title('y snap');

%%
%calculate waypoint in faster way

time=0:0.001:T;
t_now=0;
for i=0:poly_order-1
    t_now = t_now + x_solution(i+1) * time.^i;
end
t_now = 2*pi*t_now/T - pi/2;
x = R*cos(t_now).*sin(t_now);
y = R*cos(t_now);