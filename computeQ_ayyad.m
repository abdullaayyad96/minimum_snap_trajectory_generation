% n:polynormial order
% r:derivertive order, 1:minimum vel 2:minimum acc 3:minimum jerk 4:minimum snap
% t1:start timestamp for polynormial
% t2:end timestap for polynormial
% output: Hessian matrix of the cost function for a specific segment
% relative to polynomial parameters
function Q = computeQ_ayyad(n,r,t1,t2)
Q = zeros(n+1);
for i = r:n
    for j = i:n
        for x=0:2*(n-r)
            if (i+j-2*r) == x
                T_scale = t2^(x+1) - t1^(x+1);
                denomenator = (x+1) * factorial(i-r) * factorial(j-r);
                Q(i+1,j+1) = factorial(i) * factorial(j) * T_scale / denomenator;
                Q(j+1,i+1) = Q(i+1,j+1);
            end
        end
    end
end
