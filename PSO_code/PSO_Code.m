function D = PSO(Ipv, Vpv)

persistent u;
persistent dcurrent;
persistent pbest;
persistent p;   %power
persistent dc;  %duty cycle
persistent v;   %velocity
persistent counter;
persistent gbest;
%initialization
if(isempty(counter))
    counter = 0;
    dcurrent = 0.5; % current duty cycle
    gbest = 0.5;
    p=zeros(3,1);    %power
    v=zeros(3,1);     %voltage
    pbest = zeros(3,1);
    u=0;
    dc = zeros(3,1);   %duty cycle
    %initialize dc for each particle
    dc(1) = 0.2;
    dc(2) = 0.4;
    dc(3) = 0.7;
end
%At the first time, this isignored , counter = 0
%delay
if (counter>=0 && counter < 300)
    D = dcurrent;
    counter = counter+1;
    return;   %return control to the invoking function before it reaches the end of the function
end
counter = 0;  % reset the counter
%calculate the fitness function,power of each particle
%then compare the current value of the function with previous one
% Note: at the first time, this is ignored (u=0)

if(u>=1 && u<3)
    if((Vpv*Ipv)>p(u))
        p(u) = Vpv*Ipv;
        pbest(u) = dcurrent;
    end
end
u = u+1;

% At the first time this is ignored because it consist of the condition...
% u=0 (initial value at u)
if(u==5)
    u = 1;
end
if (u==1)
    D = dc(u);
    dcurrent = D;
    counter = 1;
    return;
elseif(u==2)
    D=dc(u);
    dcurrent = D;
    counter = 1;
    return;
elseif(u==3)
    D = dc(u);
    dcurrent=D;
    counter=1;
    return;
elseif(u==4)
    [m,i] = max(p); % finds the indices of the maximm values of p (max power) and returns them in output vector i
    gbest = pbest(i); % find the location (duty) of the particle which has max p
    D=gbest;
    dcurrent = D;
    counter =1;
    %update velocity and duty cycle
    v(1) = updatevelocity(v(1), pbest(1), dc(1), gbest);
    v(2) = updatevelocity(v(2), pbest(2), dc(2), gbest);
    v(3) = updatevelocity(v(3), pbest(3), dc(3), gbest);
    %update duty cycle
    dc(1) = updateduty(dc(1),v(1));
    dc(2) = updateduty(dc(2),v(2));
    dc(3) = updateduty(dc(3),v(3));
    return;
else %if u=0
    D = 0.1;
end 

end 

function vfinal = updatevelocity(velocity, pobest,d, gwbest)
%PSO parameters
w = 0.4;
c1 = 1.2; % personal learning coefficients
c2 = 2;   % global learning coefficients
vfinal = (w*velocity) + (c1*rand(1) * (pobest-d)) + (c2*rand(1)*(gwbest-d));
end
function dfinal = updateduty(d,velocity)
dup = d*velocity;
if(dup>1)
    dfinal =1;
elseif(dup<0)
    dfinal = 0;
else
    dfinal = dup;
end
end
    

