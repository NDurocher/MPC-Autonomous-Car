clear;
clc;
close all;

N = 6; % Horizon

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

psi = pi/2;

%     X               Y             psi                V              Pe
%    He             a             delta
x = [30; zeros(N-1,1); 0; zeros(N-1,1); psi; zeros(N-1,1); 0; zeros(N-1,1); 0; zeros(N-1,1); ...
      0; zeros(N-1,1); 0; zeros(N-1,1); 0; zeros(N-1,1)]; 
x0 = x;
lf = 2.5; % typical distance from center of gravity of car to front tires

X = 30;
Y = 0;


%% Simple Circluar Track
Wp = 100; %number of waypoints;
r = linspace(dt,2*pi+dt,Wp);
radius = 30;
track = [(radius-2).*[cos(r); sin(r)];
         (radius+2).*[cos(r); sin(r)];
         radius.*[cos(r); sin(r)]];


%% Loop initializations
xArray = [];
yArray = [];
vref = 100; % m/s

Np = 6; % number of way points into the future used to polyfit

options = optimoptions('fmincon','Display','off');
for i = 1:tf/dt
% 
% i = 1;
% j = 1;
% while( j<= length(track))
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
%     test = norm([X(i)-track(5,j) Y(i)-track(6,j)]);
%     if (test < 2.5)
%         j = j+1;
%         fprintf('Next waypoint: %d\n',j);
%     end
j = i;
    if (j+Np) <= length(track)
        for k = 0: Np-1
        delta_x = track(5,j+k) - px;
        delta_y = track(6,j+k) - py;
        waypoints_x(k+1) = -(delta_y * cpsi - delta_x * spsi);
        waypoints_y(k+1) = (delta_y * spsi + delta_x * cpsi);
        end
    else
        k = (length(track)-j);
        looped_x = [track(5,j:end) track(5,1:(N-k-1))];
        looped_y = [track(6,j:end) track(6,1:(N-k-1))];
        for k = 1:Np
        delta_x = looped_x(k) - px;
        delta_y = looped_y(k) - py;
        waypoints_x(k) = (delta_x * cpsi + delta_y * spsi);
        waypoints_y(k) = -(delta_y * cpsi - delta_x * spsi);
        end

    end
    
    P = polyfit(waypoints_x,waypoints_y,3);
    
    
    
%      for k = 1:N-1
%     
%       if (P == 0)
%         He = atan(0);
%     else
%         He = atan(polyder(P)*[x(xstart+(k-1))^2 x(xstart+(k-1)) 1]');
%       end    
%     x(xstart +k) = x(xstart +(k-1)) + x(vstart +(k-1))*cos(x(psistart +(k-1)))*dt;
%     x(ystart +k) = x(ystart +(k-1)) + x(vstart +(k-1))*sin(x(psistart +(k-1)))*dt;
%     x(psistart +k) = x(psistart +(k-1)) - x(vstart +(k-1))/lf*x(deltastart +(k-1))*dt;
%     x(vstart +k) = x(vstart +(k-1)) + x(astart +(k-1))*dt;
%     x(Pestart +k) = (P*[x(xstart +(k-1))^3 x(xstart +(k-1))^2 x(xstart +(k-1)) 1]') ...
%         - x(ystart +(k-1)) +x(vstart +k-1) * sin(x(Hestart +k-1))* dt;
%     x(Hestart +k) = (x(psistart +k-1) - He) -x(vstart +k-1)/lf*x(deltastart)*dt;
%     
%     end
    
    
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
    x   = fmincon(func,x0,[],[],[],[],lb,ub,nonlinear,options);  
    
    
    x(vstart) = x0(vstart) + x(astart+1)*dt;
    x(psistart) = - x(vstart)/lf*x(deltastart+1)*dt;
    psi = [psi; psi(i)+x(psistart)];
    %disp(x(deltastart));
    
    x(xstart) =  x(vstart)*cos(psi(i))*dt;
    x(ystart) =  x(vstart)*sin(psi(i))*dt;    
    X = [X; X(i)+x(xstart)];
    Y = [Y; Y(i)+x(ystart)];
    
    
%     if ( mod(i,10) == 0)
%         fprintf("Time: %d seconds\n",i*dt);
%     end
    T = polyval(P,waypoints_x(:,1));
        hold on
        subplot(2,1,1);
        plot(waypoints_x,waypoints_y,'k');
        plot(waypoints_x,T,'b');
        subplot(2,1,2);
        plot(X,Y)
%       i = i+1;
end


figure;
subplot(2,1,1);
plot(dt:dt:tf,yArray(1,:));
subplot(2,1,2);
plot(dt:dt:tf,yArray(2,:));

figure('Name','Track with Center Line');
hold on
plot(track(1,:),track(2,:),'k','linewidth',1);
plot(track(3,:),track(4,:),'k','linewidth',1);
plot(track(5,:),track(6,:),'r','linewidth',0.5);
plot(X,Y,'b','linewidth',2);
    
    