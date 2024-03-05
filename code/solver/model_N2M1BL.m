function dudt = model_N2M1BL(t,u,cse,params)
%MODEL_N2M1BL ODE function for N = 2, M = 1, log barrier model
%   Two robot, one obstacle, log barrier explicit ODE.
%
%   u (8,1) = [dx_1; dy_1;
%              x_1;  y_1;
%              dx_2; dy_2;
%              x_2;  y_2]
%
%   params
%       r (1,1) - barrier parameter
%       mu_1 (1,1) - barrier parameter
%       mu_2 (1,1) - barrier parameter

N = 2;
M = 1;
dudt = zeros(8,1);

for i = 1:N
    dx = u(i*4-3);
    dy = u(i*4-2);
    x = u(i*4-1);
    y = u(i*4-0);

    al = norm([dx,dy],2);
    be = 0;
    ga = 0;
    for j = 1:N
        if i == j
            continue;
        end
        de = norm([x,y]-u(j*4-1:j*4-0)',2);
        be = be + gp(de,params.r,params.mu_1) * (x-u(j*4-1)) / de;
        ga = ga + gp(de,params.r,params.mu_1) * (y-u(j*4-0)) / de;
    end
    for j = 1:M
        ep = norm([x,y]-cse.c(j,:),2);
        be = be + gp(ep,cse.R,params.mu_2) * (x-cse.c(j,1)) / ep;
        ga = ga + gp(ep,cse.R,params.mu_2) * (y-cse.c(j,2)) / ep;
    end

    ze = (al^2-dx^2)*(al^2-dy^2) - dx^2*dy^2/al^3;
    dudt(i*4-3:i*4-0) = [ ...
            ((al^2-dy^2)*be + (dx*dy)*ga) / ze;
            ((dx*dy)*be + (al^2-dx^2)*ga) / ze;
            dx;
            dy
        ];
end

end

%% barrier functions

function gx = g(x,d,mu)
    gx = -log(x-d)/mu;
end

function gxp = gp(x,d,mu)
    gxp = -1/mu/(x-d);
end