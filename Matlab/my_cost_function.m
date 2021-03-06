function [cost] = my_cost_function(x,Vref,N,P,lf,dt)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    %% Weights
    Wv = 0.2; %cost of velocity difference from reference velocity
    Wa = 1; % cost to acceleration
    Wd = 50; % cost to turn
    WPe = 600;%cost of path error
    WHe = 3;% cost of heading error
    Wdr = 400;% cost of change in steering angle
    War = 1;% cost of change in acceleration
    
    cCost=zeros(7,1);


    %% Vector Starts
    xstart = 1;
    ystart = xstart + N;
    psistart = ystart + N;
    vstart = psistart + N;
    Pestart = vstart + N; % Path error
    Hestart = Pestart + N; % heading error
    astart = Hestart + N;
    deltastart = astart + N;
    cost = 0;
    
    for i = 1:N-1
    
      if (P == 0)
        He = atan(0);
    else
        He = atan(polyder(P)*[x(xstart+(i-1))^2 x(xstart+(i-1)) 1]');
      end
    
    x(xstart +i) = x(xstart +(i-1)) + x(vstart +(i-1))*cos(x(psistart +(i-1)))*dt;
    x(ystart +i) = x(ystart +(i-1)) + x(vstart +(i-1))*sin(x(psistart +(i-1)))*dt;
    x(psistart +i) = x(psistart +(i-1)) - x(vstart +(i-1))/lf*x(deltastart +(i-1))*dt;
    x(vstart +i) = x(vstart +(i-1)) + x(astart +(i-1))*dt;
    x(Pestart +i) = polyval(P,x(xstart + (i-1))) ...
        - x(ystart +(i-1)) +x(vstart +i-1) * sin(x(Hestart +i-1))* dt;
    x(Hestart +i) = (x(psistart +i-1) - He) - x(vstart +i-1)/lf*x(deltastart)*dt;
    
    end
    
    %% Calculations
    for i = 0:N-1
        cost = cost + Wv*(x(vstart+i)-Vref)^2  + ...
            WPe*(x(Pestart+i))^2 + WHe*(x(Hestart+i))^2;
        cCost(1) = cCost(1)+ Wv*(x(vstart+i)-Vref)^2;
        cCost(4) = cCost(4)+WPe*(x(Pestart+i))^2;
        cCost(5) = cCost(5)+WHe*(x(Hestart+i))^2;
    end
    for i = 0:N-2
%         if ( x(astart+i) < 0)
%             Wa = 50;
%         end
        cost = cost + Wa*(x(astart+i))^2 + Wd*(x(deltastart+i))^2;
        cCost(2) = cCost(2)+  Wa*(x(astart+i))^2 ;
        cCost(3) = cCost(3)+ Wd*(x(deltastart+i))^2;
    end
    for i = 1:N-2
        cost = cost + Wdr*(x(deltastart+i)-x(deltastart+(i-1)))^2 ...
            + War*(x(astart+i)-x(astart+(i-1)))^2;
        cCost(6) = cCost(6)+ Wdr*(x(deltastart+i)-x(deltastart+(i-1)))^2;
        cCost(7) = cCost(7)+ War*(x(astart+i)-x(astart+(i-1)))^2;
    end
%         fprintf("Cost: %f,  vector: [",cost);
%         fprintf("%g ", cCost);
%         fprintf(']\n');
end












