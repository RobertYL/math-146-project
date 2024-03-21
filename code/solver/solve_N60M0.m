%% initialize

clear; close all; clc;

load("cases/case-N60M0_conv");

N = cse.N;

params.r = 0;
params.mu_1 = 1e-2;
params.mu_2 = 1;

save_figs = false;

%% secant method

iter_scnt = 80;
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
    [t,u] = ode45(@(t,u) model_NM(t,u,cse,params),[0,1],u0);

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

%% plot

% animation of paths
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
hold off;
title(sprintf("Optimal Paths of {%d} Robots",N));
xlabel("x");
ylabel("y");
set(gcf, 'Position',[680,100,560*3/2, 420*3/2])
if save_figs
    exportgraphics(fig_render,sprintf("../report/figs/N%dM0_paths.png",N), ...
        Resolution=300);
end

% robot ensemble target error
fig_t_err = figure();
scatter(1:size(err,2),sum(err,1));
set(gca,"YScale","log");
grid on;
title(sprintf("Secant Method Convergence with {%d} Robots",N));
ylabel("Total Target Position Error");
xlabel("Iteration");
if save_figs
    exportgraphics(fig_t_err,sprintf("../report/figs/N%dM0_t_err.png",N), ...
        Resolution=300);
end

% target positional error
fig_p_err = figure();
th = atan2(p_e(2,:),p_e(1,:));
logr = log10(vecnorm(p_e,2,1));
polarplot(th,logr+ceil(abs(min(logr))),'o');
set(gca,"RTickLabel", ...
    compose("10^{%2d}",get(gca,"RTick")-ceil(abs(min(logr)))));
title(sprintf("Final vs. Target Position Offset of {%d} Robots",N));
if save_figs
    exportgraphics(fig_p_err,sprintf("../report/figs/N%dM0_p_err.png",N), ...
        Resolution=300);
end






