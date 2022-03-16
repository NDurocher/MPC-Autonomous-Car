function [C,Ceq] = mycon(x,lf,dt,P,N)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
xstart = 1;
ystart = xstart + N;
psistart = ystart + N;
vstart = psistart + N;
Pestart = vstart + N; % Path error
Hestart = Pestart + N; % heading error
astart = Hestart + N;
deltastart = astart + N;

C = zeros(N*6,1);
C(xstart) = x(xstart);
C(ystart) = x(ystart);
C(psistart) = x(psistart);
C(vstart) = x(vstart);
C(Pestart) = x(Pestart);
C(Hestart) = x(Hestart);


Ceq = [];

for i = 1:N-1
    
      if (P == 0)
        He = atan(0);
    else
        He = atan(polyder(P)*[x(xstart+(i-1))^2 x(xstart+(i-1)) 1]');
      end
    
    px0 = x(xstart +(i-1)) + x(vstart +(i-1))*cos(x(psistart +(i-1)))*dt;
    py0 = x(ystart +(i-1)) + x(vstart +(i-1))*sin(x(psistart +(i-1)))*dt;
    psi0 = x(psistart +(i-1)) - x(vstart +(i-1))/lf*x(deltastart +(i-1))*dt;
    v0 = x(vstart +(i-1)) + x(astart +(i-1))*dt;
    Pe0 = polyval(P,x(xstart + (i-1))) ...
        - x(ystart +(i-1)) +x(vstart +i-1) * sin(x(Hestart +i-1))* dt;
    He0 = (x(psistart +i-1) - He) - x(vstart +i-1)/lf*x(deltastart)*dt;
    
    px1 = x(xstart +i);
    py1 = x(ystart +i);
    psi1 = x(psistart +i);
    v1 = x(vstart +i);
    Pe1 = x(Pestart +i);
    He1 = x(Hestart +i);
    
    C(xstart +i) = px1-px0;
    C(ystart +i) = py1-py0;
    C(psistart +i) = psi1-psi0;
    C(vstart +i) = v1-v0;
    C(Pestart +i) = Pe1-Pe0;
    C(Hestart +i) = He1-He0;
    
    x(xstart+i) = px0;
    x(ystart+i) = py0;
    x(psistart+i) = psi0;
    x(vstart+i) = v0;
    x(Pestart+i) = Pe0;
    x(Hestart+i) = He0;

    
    
end
end











