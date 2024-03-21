%% initialize

clear; close all; clc;

load("cases/case-N2M1_X") 

N = cse.N;

params.r = 0;
params.mu_1 = 1e-3;
params.mu_2 = 5e-4;

save_figs = true;

%% hand tuning

al = 1.06;
be = 0.932;
[t,u] = ode45(@(t,u) model_N2M1(t,u,cse,params),[0,1], ...
    [+100*al;+18.795*al; ...
     -50;-20; ...
     -100*be;+64.81*be; ...
     +50;-20]);

%%

% animation of paths
fig_render = render_case(cse);

N_anim = 5e2;
t_anim = linspace(0,1,N_anim);
line_styles = ["-","--"];
l_anim = arrayfun(@(x) animatedline(HandleVisibility="on", ...
    DisplayName=sprintf("Robot %d",x),LineStyle=line_styles(x)),1:N);
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
title(sprintf("Optimal Paths of {%d} Robots with {%d} Obstacle",N,cse.M));
xlabel("x");
ylabel("y");
xlim([-60,60]);
ylim([-60,60]);
xticks(-100:10:100);
yticks(-100:10:100);
if save_figs
    exportgraphics(fig_render,sprintf("../report/figs/N%dM1_paths.png",N), ...
        Resolution=300);
end