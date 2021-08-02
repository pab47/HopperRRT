%===================================================================
function [z,t,z_extra]=onestep(z0,robot,steps)  
%===================================================================
format long

flag = 1;
if nargin<2
    error('need more inputs to onestep');
elseif nargin<3
    flag = 0; %send only last state, for root finder and jacobian
    steps = 1;
    robot.ode = 0; %use dop853
end

if (nargin==3) %forward simulation should use ode113
    if (robot.ode==0) 
        disp('Changing integrator to ode113');
        robot.ode = 1; 
    end
end

x0 = robot.x0; x0dot = z0(1);  
y0 = z0(2); y0dot = 0;

E_kc0 = robot.E_kc; E_kt0 = robot.E_kt; E_Pt0 = robot.E_Pt; E_Pc0 = robot.E_Pc;
z0 = [x0 x0dot y0 y0dot E_kc0 E_kt0 E_Pc0 E_Pt0];

t0 = 0; 
dt = 1; %might need to be changed based on time taken for one step
time_stamps = 100;
t_ode = t0;
z_ode = [z0 ...
         x0+robot.l*sin(robot.control.theta) ...
         y0-robot.l*cos(robot.control.theta)];

for i=1:steps

    %%% apex to ground %%%
    options1 = odeset('Abstol',1e-13,'Reltol',1e-13,'Events',@contact);
    tspan = linspace(t0,t0+dt,time_stamps);
    [t_temp1,z_temp1]=ode113(@flight,tspan,z0,options1,robot);
   
    z_temp1 = [z_temp1 ...
               z_temp1(1:end,1)+robot.l*sin(robot.control.theta) ...
               z_temp1(1:end,3)-robot.l*cos(robot.control.theta) ...
              ];
    t0 = t_temp1(end);
    z0(1:4) = z_temp1(end,1:4);
    %z0(5:8) = z_temp1(end,5:8);
    x_com = z0(1); %save the x position for future
    z0(1) = -robot.l*sin(robot.control.theta); %relative distance wrt contact point because of non-holonomic nature of the system
    x_foot = x_com + robot.l*sin(robot.control.theta); 
    y_foot = robot.ground;
    
     
    %%% stance phase (compress) %%%
    tspan = linspace(t0,t0+1,time_stamps*10);
%     options2 = odeset('Abstol',1e-13,'Reltol',1e-13,'Events',@stance_mid);
%     [t_temp2,z_temp2]=ode113(@stance_compress,tspan,z0,options2,robot);
%     
    
    if (robot.ode == 1)
        options2 = odeset('Abstol',1e-13,'Reltol',1e-13,'Events',@stance_mid);
        [t_temp2,z_temp2]=ode113(@stance_compress,tspan,z0,options2,robot);
     else
        optionsdop = dopset('AbsTol',1e-13,'RelTol',1e-13,'Events',1,'EventTol',1e-13); %PAB
        params = [robot.m robot.g robot.l robot.control.kc robot.control.theta robot.control.Pc];
        %[t_temp2,z_temp2,te,ze,ie] = dop853(@stance_compressionMEX,tspan,z0',optionsdop,params');
        [t_temp2,z_temp2,te,ze,ie] = dop853_stance_compressionMEX(tspan,z0',optionsdop,params');
    end
    
    
    %z_temp2(:,1) = z_temp2(:,1) + x_com + robot.l*sin(robot.control.theta); %absolute x co-ordinate
    z_temp2 = [z_temp2, ...
          x_foot*ones(size(z_temp2,1),1) y_foot*ones(size(z_temp2,1),1)]; %the distal end of leg is 0 when touching the ground.
    t0 = t_temp2(end);
    z0(1:4) = z_temp2(end,1:4);
    %z0(5:8) = z_temp2(end,5:8);
    z_temp2(:,1) = z_temp2(:,1) + x_com + robot.l*sin(robot.control.theta);
    
    
    %%%%%%%%%%%%%%% maximum force %%%%%%%%
    l = z0(3); %y coordinate gives leg length
    Pc_max = robot.control.kc*(robot.l-l) + robot.control.Pc;
    Pt_max = robot.control.kt*(robot.l-l) + robot.control.Pt;
    
    
    %%% stance phase (tension) %%%
    tspan = linspace(t0,t0+1,time_stamps*10);
    %options2b = odeset('Abstol',1e-13,'Reltol',1e-13,'Events',@release);
    %[t_temp2b,z_temp2b]=ode113(@stance_tension,tspan,z0,options2b,robot);
   
    if (robot.ode == 1)
        options2b = odeset('Abstol',1e-13,'Reltol',1e-13,'Events',@release);
        [t_temp2b,z_temp2b]=ode113(@stance_tension,tspan,z0,options2b,robot);
     else
        optionsdop2 = dopset('AbsTol',1e-13,'RelTol',1e-13,'Events',1,'EventTol',1e-13); %PAB
        params = [robot.m robot.g robot.l robot.control.kt robot.control.theta robot.control.Pt];
        %[t_temp2b,z_temp2b,te,ze,ie] = dop853(@stance_tensionMEX,tspan,z0',optionsdop2,params');
        [t_temp2b,z_temp2b,te,ze,ie] = dop853_stance_tensionMEX(tspan,z0',optionsdop2,params');
    end

    
    z_temp2b(:,1) = z_temp2b(:,1) + x_com + robot.l*sin(robot.control.theta); %absolute x co-ordinate
    z_temp2b = [z_temp2b, ...
          x_foot*ones(size(z_temp2b,1),1) y_foot*ones(size(z_temp2b,1),1)]; %the distal end of leg is 0 when touching the ground.
    t0 = t_temp2b(end);
    z0(1:4) = z_temp2b(end,1:4);
    %z0(5:8) = z_temp2b(end,5:8);
    
    %%% ground to apex
    tspan = linspace(t0,t0+dt,time_stamps);
    options3 = odeset('Abstol',1e-13,'Reltol',1e-13,'Events',@apex);
    [t_temp3,z_temp3]=ode113(@flight,tspan,z0,options3,robot);
    
     z_temp3 = [z_temp3 ...
               z_temp3(1:end,1)+robot.l*sin(robot.control.theta) ...
               z_temp3(1:end,3)-robot.l*cos(robot.control.theta) ...
              ];
    t0 = t_temp3(end);
    z0(1:4) = z_temp3(end,1:4);
    %z0(5:8) = z_temp3(end,5:8);

    %%%%% Ignore time stamps for heelstrike and first integration point
    t_ode = [t_ode; t_temp1(2:end); t_temp2(2:end); t_temp2b(2:end);  t_temp3(2:end)];
    z_ode = [z_ode; z_temp1(2:end,:); z_temp2(2:end,:); z_temp2b(2:end,:); z_temp3(2:end,:)];
    
end

z = [z0(2) z0(3)];

if (nargout==3)
%     z = z';
%     t = t_ode(end);
    z=z_ode;
    t=t_ode;
    E_kc = z_temp1(end,5)+z_temp2(end,5)+z_temp2b(end,5)+z_temp3(end,5);
    E_kt = z_temp1(end,6)+z_temp2(end,6)+z_temp2b(end,6)+z_temp3(end,6);
    E_Pc = z_temp1(end,7)+z_temp2(end,7)+z_temp2b(end,7)+z_temp3(end,7);
    E_Pt = z_temp1(end,8)+z_temp2(end,8)+z_temp2b(end,8)+z_temp3(end,8); 
    %z_extra = [z_ode(end,5:8) z_ode(end,1)-z_ode(1,1)];
    z_extra = [E_kc E_kt E_Pc E_Pt z_ode(end,1)-z_ode(1,1) Pc_max Pt_max];
end

if flag==1
   z=z_ode;
   t=t_ode;
end

%===================================================================
function zdot=flight(t,z,robot)  
%===================================================================
% zdot = [z(2) 0 z(4) -robot.g]';
zdot = [z(2) 0 z(4) -robot.g 0 0 0 0]';

%===================================================================
function [gstop, isterminal,direction]=contact(t,z,robot)
%===================================================================
gstop = z(3) - robot.l*cos(robot.control.theta); %position is 0;
direction = -1; %negative direction goes from + to -
isterminal = 1;  %1 is stop the integration

%===================================================================
function zdot=stance_compress(t,z,robot)  
%===================================================================
x = z(1); y = z(3); %x & y position of com wrt ground
xdot = z(2); ydot = z(4);

l = sqrt(x^2+y^2);
F_spring = robot.control.Pc+robot.control.kc*(robot.l-l);
Fx_spring =  F_spring*(x/l);
Fy_spring = F_spring*(y/l);
Fy_gravity = robot.m*robot.g;
xddot = (1/robot.m)*(Fx_spring);
yddot = (1/robot.m)*(-Fy_gravity+Fy_spring);

eps = 0.01;
ldot = (x*xdot+y*ydot)/l;
F_kc = robot.control.kc*(robot.l-l);  
DE_kc = sqrt( (F_kc*ldot)^2 + eps^2 );
DE_kt = 0;
DE_Pc = sqrt( (robot.control.Pc*ldot)^2 + eps^2 );
DE_Pt = 0;

zdot = [z(2) xddot z(4) yddot DE_kc DE_kt DE_Pc DE_Pt]';

%===================================================================
function [gstop, isterminal,direction]=stance_mid(t,z,robot)
%===================================================================
gstop = z(4) - 0; %ydot is 0;
direction = 0; %negative direction goes from + to -
isterminal = 1;  %1 is stop the integration


%===================================================================
function zdot=stance_tension(t,z,robot)  
%===================================================================
x = z(1); y = z(3); %x & y position of com wrt ground
xdot = z(2); ydot = z(4);

l = sqrt(x^2+y^2);
F_spring = robot.control.Pt+robot.control.kt*(robot.l-l);
Fx_spring =  F_spring*(x/l);
Fy_spring = F_spring*(y/l);
Fy_gravity = robot.m*robot.g;
xddot = (1/robot.m)*(Fx_spring);
yddot = (1/robot.m)*(-Fy_gravity+Fy_spring);
%zdot = [z(2) xddot z(4) yddot]';

eps = 0.01;
ldot = (x*xdot+y*ydot)/l;
DE_kc = 0;
F_kt = robot.control.kt*(robot.l-l);  
DE_kt = sqrt( (F_kt*ldot)^2 + eps^2 );
DE_Pc = 0;
DE_Pt = sqrt( (robot.control.Pt*ldot)^2 + eps^2 );

zdot = [z(2) xddot z(4) yddot DE_kc DE_kt DE_Pc DE_Pt]';


%===================================================================
function [gstop, isterminal,direction]=release(t,z,robot)
%===================================================================
%gstop = z(3) - robot.l*cos(robot.control.theta); %incorrect
l = sqrt(z(1)^2+z(3)^2);
gstop = l-robot.l;
direction = 1; %negative direction goes from + to -
isterminal = 1;  %1 is stop the integration


%===================================================================
function [gstop, isterminal,direction]=apex(t,z,robot)
%===================================================================
gstop = z(4) - 0; %ydot is 0;
direction = 0; %negative direction goes from + to -
isterminal = 1;  %1 is stop the integration

