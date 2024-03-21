function fig = render_case(cse)
%RENDER_CASE Render a case
%  Renders initial and target location of robots and obstacles of case CSE.

    arguments
        cse Case
    end

    fig = figure();
    
    scatter(cse.x(:,1),cse.x(:,2),'*',DisplayName="Initial Position");
    hold on;
    scatter(cse.y(:,1),cse.y(:,2),'*',DisplayName="Target Position");
    if cse.M
        circle(cse.c(:,1),cse.c(:,2),cse.R);
    end
    hold off;

    xlim([-cse.opts.MaxPos,cse.opts.MaxPos]);
    ylim([-cse.opts.MaxPos,cse.opts.MaxPos]);
    xticks(linspace(-cse.opts.MaxPos,cse.opts.MaxPos,11));
    yticks(linspace(-cse.opts.MaxPos,cse.opts.MaxPos,11));
    axis square;
    grid on;
    legend(Location="eastoutside");
end

function h = circle(x,y,r)
%CIRCLE Helper circle plotting function

    theta = 0:pi/50:2*pi;
    x_unit = r * cos(theta) + x;
    y_unit = r * sin(theta) + y;
    h0 = plot(x_unit(1,:),y_unit(1,:),DisplayName="Circular Obstacle");
    for i = 2:length(x)
        h = plot(x_unit(i,:),y_unit(i,:),Color=get(h0,'Color'), ...
            HandleVisibility="off");
    end
end