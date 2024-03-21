function dudt = model_N30M0(t,u,cse,params)
%MODEL_N30M1BL ODE function for N = 30, M = 0, inverse barrier model
%   Thirty robots, no obstacles, inverse barrier explicit ODE.
%
%   u (120,1) = [dx_1; dy_1;
%              x_1;  y_1;
%               ...
%              dx_30; dy_30;
%              x_30;  y_30]
%
%   params
%       r (1,1) - robot barrier parameter
%       mu_1 (1,1) - robot barrier parameter
%       mu_2 (1,1) - obstacle barrier parameter

N = 30;
M = 0;
dudt = zeros(N*4,1);

for i = 1:N
    dx = u(i*4-3);
    dy = u(i*4-2);
    x = u(i*4-1);
    y = u(i*4-0);

    al = 0;
    be = 0;
    for j = 1:N
        if i == j
            continue;
        end
        ga = gp(norm([x;y]-u(j*4-1:j*4-0),2)^2,params.r^2,params.mu_1);
        al = al + ga * (x-u(j*4-1));
        be = be + ga * (y-u(j*4-0));
    end
    for j = 1:M
        ga = gp(norm([x,y]-cse.c(j,:),2)^2,cse.R^2,params.mu_2);
        al = al + ga * (x-cse.c(j,1));
        be = be + ga * (y-cse.c(j,2));
    end

    dudt(i*4-3:i*4-0) = [ ...
            al;
            be;
            dx;
            dy
        ];
end

end

%% barrier functions

function gx = g(x,d,mu)
    gx = 1/mu/(x-d);
end

function gxp = gp(x,d,mu)
    gxp = -1/mu/((x-d)^2);
end