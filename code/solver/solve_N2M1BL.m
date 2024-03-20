cse = Case(2,1,20);
cse.x = [-50,-20;
         +50,-20];
cse.y = [+50,+20;
         -50,+20];
cse.c = [0,0];
cse.R = 10;

params.r = 1.5;
params.mu_1 = 1e-3;
params.mu_2 = 5e-4;

al = 1.05;
be = 0.94;
[t,u] = ode45(@(t,u) model_N2M1BL(t,u,cse,params),[0,1], ...
    [+100*al;+18.79*al; ...
     -50;-20; ...
     -100*be;+64.8*be; ...
     +50;-20]);

%%

fig = render_case(cse);

N_anim = 1e3;
t_anim = linspace(0,1,N_anim);
l_1 = animatedline(LineStyle="-",DisplayName="Robot 1");
l_2 = animatedline(LineStyle="--",DisplayName="Robot 2");
hold on;
for k = 1:N_anim
    addpoints(l_1,interp1(t,u(:,3),t_anim(k)),interp1(t,u(:,4),t_anim(k)));
    addpoints(l_2,interp1(t,u(:,7),t_anim(k)),interp1(t,u(:,8),t_anim(k)));
    drawnow
end
% scatter(u(:,3),u(:,4));
% scatter(u(:,7),u(:,8));
hold off;