function data = PlotSimulationResults(data,par)

%% Initialize
    % Define colors
    color_red = [85 33 10]/100;
    color_blue = [0 .45 0.74];


%% Plot results
% Increase length of pendulum by factor for figure - looks better
par.l = par.r*4;

% Get data for initial plot
    iidx = 1;
    t           = data.t(iidx);
    x           = data.p(iidx);    
    d_p         = data.d_p(iidx);
    theta       = data.theta(iidx);
    d_theta     = data.d_theta(iidx);

% Create figure    
    fh  = figure;
    % Callback functions
        set(fh, 'WindowScrollWheelFcn',  @fun_scroll)
        set(fh, 'WindowButtonDownFcn',   @fun_mousedown)
        set(fh, 'WindowButtonUpFcn',     @fun_mouseup)
        set(fh, 'WindowButtonMotionFcn', @fun_MouseMove);
    % Subplots arrangement
        nr  = 4;
        nc  = 2;
        mlw = 2;

    sph(1) = subplot(nr,nc,1);grid on;hold on;
        plot(data.t,data.p);title('Position in m');
        ph(1) = stem(0,diff(get(gca, 'ylim')),'color',color_red,'BaseValue',min(get(gca, 'ylim')), 'marker','none', 'linewidth',mlw);
    sph(2) = subplot(nr,nc,3);grid on;hold on;
        plot(data.t,data.d_p);th = title('Velocity in m/s');
        ph(2) = stem(0,diff(get(gca, 'ylim')),'color',color_red,'BaseValue',min(get(gca, 'ylim')), 'marker','none', 'linewidth',mlw);
    sph(3) = subplot(nr,nc,5);grid on;hold on;
        plot(data.t,data.theta*360/2/pi);title('Angle in deg');
        ph(3) = stem(0,diff(get(gca, 'ylim')),'color',color_red,'BaseValue',min(get(gca, 'ylim')), 'marker','none', 'linewidth',mlw);
    sph(4) = subplot(nr,nc,7);grid on;hold on;
        plot(data.t,data.d_theta);title('Angular Velocity in deg/s');
        ph(4) = stem(0,diff(get(gca, 'ylim')),'color',color_red,'BaseValue',min(get(gca, 'ylim')), 'marker','none', 'linewidth',mlw);
    set(sph, 'ylimmode','manual');

    % Plot model of segway
        sph_model = subplot(nr,nc,2:2:nr*nc);grid on;hold on;
            set(gca, 'xlimmode','manual','xlim',[min(data.p)-par.l, max(data.p)+par.l]);
            set(gca, 'ylimmode','manual','ylim',[par.r-par.l, par.r+par.l]);
            h_tit = title(sprintf('Simulation Speed = 1 * real time'));
        % Plot wheel
            rads =linspace(0,2*pi,111);
            wx0 = par.r * cos(rads);
            wy  = par.r * sin(rads)+par.r;
            h_w = patch(wx0+x, wy,color_red);    
            set(h_w, 'edgecolor','none','facecolor',color_blue);

        % Plot body
            p1 = [x par.r];
            p2 = [x+par.l*sin(theta) par.r+par.l*cos(theta)];
            h_b = plot([p1(1) p2(1)], [p1(2) p2(2)],'color', color_red, 'linewidth',5);

            % Plot radius
            phi = x/par.r;
            h_r = plot([x, x+par.r*sin(phi)],[par.r, par.r+par.r*cos(phi)], 'k', 'linewidth',2);

% Initialize callback values            
    MouseDown   = 0;
    scrollidx   = 1;
    stoptime    = 0;
    curtimestep0 = round(0.1*length(data.t)/(data.t(end) - data.t(1)));
    curtimestep = curtimestep0;
    
% Buttons for start/stop
    uib_h = 0.05;
    uib_w = 0.08;
    ii_uib = 1;
    h_uib_start=uicontrol('Parent',gcf,'Style','pushbutton','String','Start','Units','normalized','Position',[1-uib_w, 1-ii_uib*uib_h, uib_w/2, uib_h]);
    h_uib_stop=uicontrol('Parent',gcf,'Style','pushbutton','String','Stop','Units','normalized','Position',[1-uib_w/2, 1-ii_uib*uib_h, uib_w/2, uib_h]);
    ii_uib = 2;
    h_uib_fast=uicontrol('Parent',gcf,'Style','pushbutton','String','Faster','Units','normalized','Position',[1-uib_w, 1-ii_uib*uib_h, uib_w, uib_h]);
    ii_uib = 3;
    h_uib_slow=uicontrol('Parent',gcf,'Style','pushbutton','String','Slower','Units','normalized','Position',[1-uib_w, 1-ii_uib*uib_h, uib_w, uib_h]);
    addlistener(h_uib_start,'Action',@(src,evnt)fun_start);
    addlistener(h_uib_stop,'Action',@(src,evnt)fun_stop);
    addlistener(h_uib_fast,'Action',@(src,evnt)fun_fast);
    addlistener(h_uib_slow,'Action',@(src,evnt)fun_slow);

%% Callback functions    
function fun_scroll(fh, EventData)
    if EventData.VerticalScrollCount > 0 % downscroll
        scrollidx = scrollidx - 10;
    elseif EventData.VerticalScrollCount < 0 % upscroll
        scrollidx = scrollidx + 10;
    end
    updateTimeIdx(scrollidx,0)
end
function fun_mousedown(fh, EventData)
    MouseDown = 1;
end
function fun_mouseup(fh, EventData)
    MouseDown = 0;
end
function updateTimeIdx(idx,istime)
    if istime
        idx = (idx>0)*idx;
        scrollidx = interp1(data.t, 1:length(data.t),idx, 'nearest');
    else
        scrollidx = idx;
    end
        scrollidx   = max(min(scrollidx,length(data.t)),1);
        x           = data.p(scrollidx);   
        d_p         = data.d_p(scrollidx);        
        theta       = data.theta(scrollidx);
        d_theta     = data.d_theta(scrollidx);

        t           = data.t(scrollidx);

        set(ph, 'XData',t);
        set(h_w, 'XData',wx0+x);
        phi = x/par.r;
        set(h_r, 'XData',[x, x+par.r*sin(phi)], 'YData',[par.r, par.r+par.r*cos(phi)]);
            
        p1 = [x par.r];
        p2 = [x+par.l*sin(theta) par.r+par.l*cos(theta)];
        set(h_b, 'XData',[p1(1) p2(1)], 'YData',[p1(2) p2(2)]);
end
function fun_MouseMove(fh, EventData)
    if MouseDown
        tmp = get(get(fh, 'CurrentAxes'), 'CurrentPoint');
        updateTimeIdx(tmp(1,1),1);
    end
end
function fun_start
    try
        stoptime = 0;
        idx = (t>0)*t;
        scrollidx = interp1(data.t, 1:length(data.t),idx, 'nearest');
        
        while stoptime==0
            updateTimeIdx(scrollidx,0);
            scrollidx = scrollidx+curtimestep;
        pause(0.1);
            if scrollidx>=length(data.t)
                stoptime = 1;
            end        
        end

    catch
        return;
    end
end
function fun_stop
    stoptime = 1;
end
function fun_fast
    curtimestep = round(curtimestep*2);
    set(h_tit, 'String',sprintf('Simulation Speed = %1.2f * real time',curtimestep/curtimestep0));

end
function fun_slow
curtimestep = round(max(curtimestep/2,1));
set(h_tit, 'String',sprintf('Simulation Speed = %1.2f * real time',curtimestep/curtimestep0));
end
end
