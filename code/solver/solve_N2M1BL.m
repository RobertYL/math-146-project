cse = Case(2,1,20);
cse.x = [-70,-90;
         -90,-70];
cse.y = [+70,+90;
         +90,+70];
cse.c = [0,0];

params.r = 10;
params.mu_1 = 1;
params.mu_2 = 100000;

[t,u] = ode45(@(t,u) model_N2M1BL(t,u,cse,params),[0,1], ...
    [150;140;-70;-90;125;175;-90;-70]);

%%

fig = render_case(cse);
hold on;
scatter(u(:,3),u(:,4));
scatter(u(:,7),u(:,8));
hold off;