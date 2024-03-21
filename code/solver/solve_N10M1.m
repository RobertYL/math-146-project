%% initialize

clear; close all; clc;

% cse = Case(10,1);
% cse.c = [0,200];
% cse.R = 1;

load("cases/case-N10M1_conv");

N = cse.N;

params.r = 0;
params.mu_1 = 1e-5;
params.mu_2 = 5e-4*1e7;

%% secant method

iter_scnt = 50;
err = nan(N,iter_scnt+2);

for iter = -1:iter_scnt
    if iter == -1
        theta = rand(1,N)*2*pi;
        dp_0 = [cos(theta); sin(theta)];
        for i = 1:N
            dp_0(:,i) = dp_0(:,i)*norm(cse.x(i,:)-cse.y(i,:),2);
        end
    elseif iter == 0
        dp_0_prev = dp_0;
        p_e_prev = p_e;

        dtheta = (rand(1,N)-0.5)*pi/18/10;
        dp_0 = dp_0_prev;
        for i = 1:N
            dp_0(:,i) = [cos(dtheta(i)),  sin(dtheta(i)); ...
                         -sin(dtheta(i)), cos(dtheta(i))] * dp_0(:,i);
        end
    else
        ga = dp_0 - p_e .* (dp_0_prev - dp_0) ./ (p_e_prev - p_e);
        dp_0_prev = dp_0;
        p_e_prev = p_e;
        dp_0 = ga;
    end

    % generate init cond
    u0 = zeros(4*N,1);
    for i = 1:N
        i0 = 4*(i-1);
        u0(i0+(1:2)) = dp_0(:,i);
        u0(i0+(3:4)) = cse.x(i,:)';
    end

    % sim
    [t,u] = ode45(@(t,u) model_N10M1(t,u,cse,params),[0,1],u0);

    p_e = zeros(2,N);
    for i = 1:N
        i0 = 4*(i-1);
        p_e(:,i) = (u(end,i0+(3:4)) - cse.y(i,:))';

        err(i,iter+2) = norm(p_e(:,i),2);
    end

    if norm(err(:,iter+2),2) < 1e-8
        break;
    end
end

%%

fig_render = render_case(cse);

N_anim = 5e2;
t_anim = linspace(0,1,N_anim);
l_anim = arrayfun(@(x) animatedline(HandleVisibility="off"),1:N);
hold on;
for k = 1:N_anim
    for i = 1:N
        i0 = 4*(i-1);
        addpoints(l_anim(i),interp1(t,u(:,i0+3),t_anim(k)), ...
            interp1(t,u(:,i0+4),t_anim(k)));
    end
    drawnow
end
% scatter(u(:,3),u(:,4));
% scatter(u(:,7),u(:,8));
hold off;

fig_err = figure();
scatter(1:size(err,2),vecnorm(err,2,1));
set(gca,"YScale","log");
grid on;