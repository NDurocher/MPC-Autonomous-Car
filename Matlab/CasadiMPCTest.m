clear;
clc;
close all;
N = 12; % Horizon
Np = 5; % number of way points into the future used to polyfit

xstart = 1;
ystart = xstart + N;
psistart = ystart + N;
vstart = psistart + N;
Pestart = vstart + N; % Path error
Hestart = Pestart + N; % heading error
astart = Hestart + N;
deltastart = astart + N;

dt = 0.01; % seconds
tf = 30;    % seconds

psi = pi/2; %pi/2;%0.68;%pi/2;

%     X               Y             psi                V              Pe
%    He             a             delta
x = [30; zeros(N-1,1); 0; zeros(N-1,1); psi; zeros(N-1,1); 10; zeros(N-1,1); 0; zeros(N-1,1); ...
      0; zeros(N-1,1); 0; zeros(N-1,1); 0; zeros(N-1,1)]; 
x0 = x;
lf = 1;%2.5; % typical distance from center of gravity of car to front tires



%% Simple Circluar Track
Wp = 50; %number of waypoints;
r = linspace(dt,2*pi+dt,Wp);
radius = 100;
%  trackFunc = @(r)[(radius-2).*[cos(r); sin(r)];
%           (radius+2).*[cos(r); sin(r)];
%           radius.*[cos(r); sin(r)]];%circle

trackFunc = @(pIn)2*[[sin(pIn*0.1)*pIn+2;pIn];
[sin(pIn*0.1)*pIn-2;pIn];
[sin(pIn*0.1)*pIn;pIn]];%sinus

% trackFunc = @(pIn)[[pIn+2;pIn];[pIn-2;pIn];[pIn;pIn]]; line
% track= generate_track(trackFunc, 500);
track = load('../FinalTrack/FinalTrack.csv')';



%% Loop initializations
xArray = [];
yArray = [];
vref = 11; % m/s


options = optimoptions('fmincon','Display','off','MaxFunctionEvaluations',1000);
% for i = 1:tf/dt
% 
i = 1;
j = 220;
X = track(5,j);
Y = track(6,j);
while( j<= length(track))
    %% Constraint
    
x(xstart) = 0;
x(ystart) = 0;
x(psistart) = 0;    

% possition is +-9999, heading is +-9999 degs, velocity is 0-9999m/s, 
% Pe & He are both -9999 to 9999 accel is +-1 and delta is -25 to 25 degs 

lb = [x(xstart); -0*ones(N-1,1); x(ystart); -0*ones(N-1,1); x(psistart); -0*ones(N-1,1); x(vstart); -0*ones(N-1,1); ...
    x(Pestart); -0*ones(N-1,1); x(Hestart); -0*ones(N-1,1); -3*ones(N,1); -0.4363*ones(N,1)];
ub = [x(xstart); 0*ones(N-1,1); x(ystart); 0*ones(N-1,1); x(psistart); 0*ones(N-1,1); x(vstart); -0*ones(N-1,1); ...
    x(Pestart); 0*ones(N-1,1); x(Hestart); 0*ones(N-1,1); 3*ones(N,1); 0.4363*ones(N,1)];  



        %% Path function update
    px = X(i);
    py = Y(i);
    waypoints_x = zeros(Np,1);
    waypoints_y = zeros(Np,1);
    cpsi = cos(psi(i));
    spsi = sin(psi(i));
      
    test = norm([X(i)-track(5,j) Y(i)-track(6,j)]);
    if (test < 1.5)
        j = j+1;
    end
    
    distance = inf(10,1);
    for d=0:10
        if(j+d<length(track))
            distance(d+1) = norm([X(i)-track(5,j+d) Y(i)-track(6,j+d)]);
        end
    end
    [val,index]=min(distance);
    if(index >1)
        if(index-1>1)
            warning("Overjumped more than 1 waypoint")
        end
        j=j+(index-1);
    end 

% j = i;
    if (j+Np) <= length(track)
        fprintf('Next waypoint: %d, x: %d, y: %d\n',j, track(5,j), track(6,j))
        for k = 1: Np-1
        delta_x = track(5,j+k) - px;
        delta_y = track(6,j+k) - py;
        waypoints_x(k+1) = (delta_x * cpsi + delta_y * spsi);
        waypoints_y(k+1) = (delta_x * -spsi + delta_y * cpsi);
        end
    else
        k = (length(track)-j);
        looped_x = [track(5,j:end) track(5,1:(N-k-1))];
        looped_y = [track(6,j:end) track(6,1:(N-k-1))];
        for k = 2:Np
        delta_x = looped_x(k) - px;
        delta_y = looped_y(k) - py;
        waypoints_x(k) = (delta_x * cpsi + delta_y * spsi);
        waypoints_y(k) = (delta_x * -spsi + delta_y * cpsi);
        end
       
    end
%     scatter(waypoints_x,waypoints_y); 
     P = polyfit(waypoints_x,waypoints_y,3);
%     s = linspace(0,waypoints_x(2),20);
%     T = polyval(P,s);
%     plot(s,T);
        
    x0(xstart) = 0;
    x0(ystart) = 0;
    x0(psistart) = 0;
    x0(vstart) = x(vstart);
    x0(Pestart) = P(4);
    x0(Hestart) = -atan(P(3));
    
    
    %% Function update
   func = @(x)my_cost_function(x,vref,N,P,lf,dt);
   nonlinear = @(x)mycon(x,lf,dt,P,N);
   
    %% Simulation & optimize
    xArray = [xArray x];
    yArray = [yArray [x(astart) ;x(deltastart)]];
    %% Begin casadi
    addpath('C:\Users\schmi\OneDrive - Syddansk Universitet\Classical Autonomous Systems\casadi-windows-matlabR2016a-v3.5.5')
    import casadi.*

%     x = SX.sym('w'); % Decision variables (controls)
%     obj = @(x)my_cost_function(x,vref,N,P,lf,dt);%x^2-6*x+13 ; % calculate obj
% 
%     g = [];  % Optimization constraints – empty (unconstrained)
%     P = [];  % Optimization problem parameters – empty (no parameters used here)
% 
%     OPT_variables = x;  %single decision variable
%     nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);
% 
% 
%     opts = struct;
%     opts.ipopt.max_iter = 1000;
%     opts.ipopt.print_level = 0; %0,3
%     opts.print_time = 0; %0,1
%     opts.ipopt.acceptable_tol =1e-8; % optimality convergence tolerance
%     opts.ipopt.acceptable_obj_change_tol = 1e-6; 
% 
%     solver = nlpsol('solver', 'ipopt', nlp_prob,opts);
% 
%     args = struct;
%     args.lbx = -inf;  % unconstrained optimization 
%     args.ubx = inf;   % unconstrained optimization
%     args.lbg = -inf;  % unconstrained optimization
%     args.ubg = inf;   % unconstrained optimization
% 
%     args.p   =  [];  % There are no parameters in this optimization problem
%     args.x0  = -0.5; % initialization of the optimization problem
% 
%     sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
%         'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
%     x_sol = full(sol.x)            % Get the solution
%     min_value = full(sol.f)   % Get the value function
    
    %% End casadi
     result=fmincon(func, x0)

    casX=MX.sym('cost');
    f=Function('f', {casX}, my_cost_function(x,vref,N,P,lf,dt));
    fminunc(@x, full(f(x)), 0.1)
%     x   = fmincon(func,x0,[],[],[],[],lb,ub,nonlinear,options);  
    
    
    x(vstart) = x0(vstart) + x(astart+1)*dt;
    x(psistart) = - x(vstart)/lf*x(deltastart+1)*dt;
    psi = [psi; psi(i)+x(psistart)];
    %disp(x(deltastart));
    
    x(xstart) =  x(vstart)*cos(psi(i))*dt;
    x(ystart) =  x(vstart)*sin(psi(i))*dt;    
    X = [X; X(i)+x(xstart)];
    Y = [Y; Y(i)+x(ystart)];
    
    
    if ( mod(i,10) == 0)
        fprintf("Time: %d seconds\n",i*dt);
    end
    if(mod(i,1)==0)
    T = polyval(P,waypoints_x(:,1));
        subplot(3,6,[13:18])
        plot(waypoints_x,waypoints_y,'k','linewidth',1);
        axis equal
        hold on
        plot(waypoints_x,T,'b');
        hold off
        subplot(3,6,[1:5 7:11]);
        plot(track(1,:),track(2,:),'k','linewidth',1);
        hold on
        plot(track(3,:),track(4,:),'k','linewidth',1);
        plot(track(5,:),track(6,:),'r','linewidth',0.5);
        plot(X,Y,'b','linewidth',2);
%         axis([458 478 0 160])
        h3 = subplot(3,6,[6 12]);
        cla(h3)
        axis off
        text(.1,.5,['Velocity: ' num2str(round(x(vstart),2)) 'm/s'],'Fontsize',12);
        text(.1,0,['Steering Angle: ' num2str((round(x(deltastart+1),2))*180/pi) 'deg'],'Fontsize',12);
        hold off
        pause(0.05);
    end
         i = i+1;

end 

if(length(yArray)>2)
    csvwrite('Export/heading.csv', psi);
    csvwrite('Export/velocity.csv', xArray(vstart,:)');
    yArrayHeader = {'Acceleration', 'Steering angle'};
    trackHeader = {'xRoadEnd1', 'yRoadEnd1', 'xRoadEnd2', 'xRoadEnd2', 'xTrack', 'yTrack'};
    savefig('Export/Result.fig');
    csvwrite_with_headers('Export/Y-Array.csv', yArray', yArrayHeader);
    csvwrite_with_headers('Export/Track.csv', track',trackHeader);
end

figure;
subplot(2,1,1);
plot(dt:dt:(i-1)*dt,yArray(1,:));
subplot(2,1,2);
plot(dt:dt:(i-1)*dt,yArray(2,:));


% figure('Name','Track with Center Line');
% hold on
% plot(track(1,:),track(2,:),'k','linewidth',1);
% plot(track(3,:),track(4,:),'k','linewidth',1);
% plot(track(5,:),track(6,:),'r','linewidth',0.5);
% plot(X,Y,'b','linewidth',2);
    
    