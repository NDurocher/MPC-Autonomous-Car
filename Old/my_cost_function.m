function [cost] = my_cost_function(x,Vref,N,P,lf,dt)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    %% Weights
    Wv = 50;
    Wa = 1;
    Wd = 1;
    WPe = 500;
    WHe = 300;
    Wdr = 1;
    War = 1;


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
    x(Pestart +i) = (P*[x(xstart +(i-1))^3 x(xstart +(i-1))^2 x(xstart +(i-1)) 1]') ...
        - x(ystart +(i-1)) +x(vstart +i-1) * sin(x(Hestart +i-1))* dt;
    x(Hestart +i) = (x(psistart +i-1) - He) -x(vstart +i-1)/lf*x(deltastart)*dt;
    
    end
    
    %% Calculations
    for i = 0:N-1
        cost = cost + Wv*(x(vstart+i)-Vref)^2  + ...
            WPe*(x(Pestart+i))^2 + WHe*(x(Hestart+i))^2;
    end
    for i = 0:N-2
        if ( x(astart+i) < 0)
            Wa = 1;
        end
        cost = cost + Wa*(x(astart+i))^2 + Wd*(x(deltastart+i))^2;
    end
    for i = 1:N-2
        cost = cost + Wdr*(x(deltastart+i)-x(deltastart+(i-1)))^2 ...
            + War*(x(astart+i)-x(astart+(i-1)))^2;
    end
end













