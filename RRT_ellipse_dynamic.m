% % An example of rapidly-exploring random trees and path planning in 2-D
% % Ref: "Rapidly-Exploring Random Trees: A New Tool for Path Planning",
% % Steven M. LaValle, 1998
%~~~~
% Code can also be converted to function with input format
% [tree, path] = RRT(K, xMin, xMax, yMin, yMax, xInit, yInit, xGoal, yGoal, thresh)
% K is the number of iterations desired.
% xMin and xMax are the minimum and maximum values of x
% yMin and yMax are the minimum and maximum values of y
% xInit and yInit is the starting point of the algorithm
% xGoal and yGoal are the desired endpoints
% thresh is the allowed threshold distance between a random point the the
% goal point.
% Output is the tree structure containing X and Y vertices and the path
% found obtained from Init to Goal
%~~~~ 
% Written by: Omkar Halbe, Technical University of Munich, 31.10.2015
% Downloaded from https://www.mathworks.com/matlabcentral/fileexchange/53772-rapidly-exploring-random-trees-algorithm

% Modified by: Pranav Bhounsule Apr, 13 2018 the code to resemble the pseudo-code posted by Steven LaValle on
% http://msl.cs.uiuc.edu/rrt/about.html
%~~~~ (
clc
clear all; close all;
load('control_policy.mat');

dynamic = 1; %dynamics = 1 uses dynamics (feedback motion planning), dynamics = 0 is static case, goes from fixed point to fixed point
 
PaperPosition = [-0.25 -0.1 8 6]; %location on printed page. rect = [left, bottom, width, height]
PaperSize = [7.25 5.8]; %[width height]
Fontsize = 12;
print_pdf = 0;

%rand('state', 4);
K=1000; %iterations
xMin=0; xMax=6; %velocity
yMin=1; yMax=2; %height

%http://msl.cs.uiuc.edu/rrt/about.html
%xInit=1; yInit=1.1; %initial point for planner (x = velocity, y = height)
%xGoal=5; yGoal=1.5; %goal for planner
 
% % %%%%%% illustration case 
% rand('state', 2);
% xInit=2; yInit=1.3; %initial point for planner (x = velocity, y = height)
% xGoal=4; yGoal=1.3; %goal for planner

%%%% good cases: for paper
%%%% test 1, increase speed only (good)
rand('state', 2); %5 steps
xInit=2; yInit=1.3; %initial point for planner (x = velocity, y = height)
xGoal=5; yGoal=1.3; %goal for planner

% % %%% test 2, Decrease speed only (good)
% rand('state', 2); %5 steps
% xInit=5; yInit=1.3; %initial point for planner (x = velocity, y = height)
% xGoal=2; yGoal=1.3; %goal for planner
% %   

% %%%% test 3, increase speed and height (good)
% rand('state', 2); %5 steps
% xInit=2; yInit=1.3; %initial point for planner (x = velocity, y = height)
% xGoal=4; yGoal=1.6; %goal for planner
%  
% %%% test 4, decrease speed and increase height (good)
% rand('state', 2); %6 steps
% xInit=5; yInit=1.3; %initial point for planner (x = velocity, y = height)
% xGoal=2; yGoal=1.6; %goal for planner
%  
% 
%%%%%% limitation cases
% %%% test 1a, increase speed and height
% xInit=2; yInit=1.3; %initial point for planner (x = velocity, y = height)
% xGoal=2; yGoal=1.4; %goal for planner
%  

% ellipse_area = pi*0.3;
% a = 1.2;
% b = 0.2;% ellipse_area/(pi*a);
P = [1 0; 0 11.11111111];
P1 = P(1,1); 
P2 = P(2,2);
a = 1/sqrt(P1);
b = 1/sqrt(P2);

    
thresh=min(a,b);  %termination criteria
delta_q = 1; %rate of increase in the direction of random point is the major axis of the ellipse

% Step 1. in http://msl.cs.uiuc.edu/rrt/about.html
tree.vertex(1).x = xInit;
tree.vertex(1).y = yInit;
tree.vertex(1).xPrev = xInit;
tree.vertex(1).yPrev = yInit;
tree.vertex(1).dist=0;
tree.vertex(1).ind = 1; tree.vertex(1).indPrev = 0;

xArray=xInit; yArray = yInit;

% h=figure(1); hold on; grid on;
% plot(xInit, yInit, 'ko', 'MarkerSize',10, 'MarkerFaceColor','k');
% plot(xGoal, yGoal, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');
% xlabel('Velocity (m/s)','Fontsize',Fontsize);
% ylabel('Height (m)','Fontsize',Fontsize);
% string = 'RRT_result';

if sqrt( (xInit-xGoal)^2 + (yInit-yGoal)^2 ) <= thresh 
      error('xinit is withing ROA of goal: no motion planning involved');
end
       
for iter = 2:K %Step 2 in http://msl.cs.uiuc.edu/rrt/about.html
    
    not_feasible = 1;
 
    while(not_feasible) %iterates till for only running solution
    %%%%%% Step 3 RAND_CONF() in http://msl.cs.uiuc.edu/rrt/about.html
    xRand = xMin+(xMax-xMin)*rand; 
    yRand = yMin+(yMax-yMin)*rand;

    %%%% Step 4 NEAREST_VERTEX() in http://msl.cs.uiuc.edu/rrt/about.html
    dist = Inf*ones(1,length(tree.vertex));
    for j = 1:length(tree.vertex)
        dist(j) = sqrt( (xRand-tree.vertex(j).x)^2 + (yRand-tree.vertex(j).y)^2 ); 
                        %find distance from (xRand,yRand) and all points in the vertex
    end
    [val, ind] = min(dist); %find index of point in the tree that is closest to (xRand,yRand)
       
    %%%% Step 5 NEW_CONF() in http://msl.cs.uiuc.edu/rrt/about.html
    x_nearest =  tree.vertex(ind).x; y_nearest =  tree.vertex(ind).y;
    mag = sqrt( (x_nearest-xRand)^2+ (y_nearest-yRand)^2);
    direction_x = (xRand-x_nearest)/mag; direction_y = (yRand-y_nearest)/mag;
    
    %phi_rand = atan(direction_y/direction_x);
    phi_rand = atan2(direction_y,direction_x);
    phi = 0;
    
    if (~dynamic)
     x_new = x_nearest + delta_q*a*cos(-phi_rand); y_new = y_nearest + delta_q*b*sin(-phi_rand);
    else
   %%%%%%% feedback motion planning is here %%%%%%%%%%
    x_fixed = x_nearest + delta_q*a*cos(-phi_rand); 
    y_fixed = y_nearest + delta_q*b*sin(-phi_rand);
    if (x_fixed<xMin || x_fixed > xMax)
        continue;
    end
    
    if (y_fixed<yMin || y_fixed > yMax)
        continue;
    end
    
    %[x_nearest y_nearest]
    %[x_fixed y_fixed]
    robot.zstar = [x_fixed y_fixed];
    z_k = [x_nearest y_nearest];
    X = z_k(1); Y = z_k(2);
    [control_fit_theta,control_fit_Pc,control_fit_Pt] = main_fit_data_neural_net_postprocess(net_theta,net_Pc,net_Pt,X,Y,robot.m,robot.g,robot.zstar,cluster_data);
    robot.control.theta = control_fit_theta; 
    robot.control.Pc = control_fit_Pc;  
    robot.control.Pt = control_fit_Pt; 
    [zz,tt,z_extra] = onestep(z_k,robot,1);
    x_new = zz(end,2); y_new = zz(end,3);
    %[x_new y_new]
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if (y_new>1.0)
        not_feasible = 0; %false
    end
    %plot(x_new,y_new,'bo', 'MarkerSize',10, 'MarkerFaceColor','b');
    end
    
    %mag2 = sqrt( (x_nearest-x_new)^2+ (y_nearest-y_new)^2)
    
    %%%% STEP 6: G.add_vertex(qnew);
    tree.vertex(iter).x =  x_new; tree.vertex(iter).y =  y_new; %add the point to the tree
    tree.vertex(iter).dist = val; %add the distance to the tree
        
    %%%% STEP 7: G.add_edge(qnear,qnew);
    tree.vertex(iter).xPrev = tree.vertex(ind).x; %add x_near, y_near to the tree
    tree.vertex(iter).yPrev = tree.vertex(ind).y;
    tree.vertex(iter).ind = iter; tree.vertex(iter).indPrev = ind;
    
    if (dynamic) %associate controller with 
        tree.vertex(iter).control_fit_theta = control_fit_theta;
        tree.vertex(iter).control_fit_Pc = control_fit_Pc;
        tree.vertex(iter).control_fit_Pt = control_fit_Pt;
        tree.vertex(iter).zstar = robot.zstar;
    end 
    
    P1 = 1/(a*a);
    P2 = 1/(b*b);
    tree.vertex(iter).P = [P1 P2];
    tree.vertex(iter).phi = phi;
    
    %%% terminate when x_nearest, y_nearest is sufficiently close to goal
%     if sqrt( (x_nearest-xGoal)^2 + (y_nearest-yGoal)^2 ) <= thresh 
    if sqrt( (x_new-xGoal)^2 + (y_new-yGoal)^2 ) <= thresh 
        plot([tree.vertex(iter).x; tree.vertex(ind).x],[tree.vertex(iter).y; tree.vertex(ind).y], 'rs-');
        break
    end
    
    plot([tree.vertex(iter).x; tree.vertex(ind).x],[tree.vertex(iter).y; tree.vertex(ind).y], 'rs-');
    
   
%     if (~dynamic)
%         generate_ellipse(x_new,y_new,P1,P2,phi);
%     else
%         generate_ellipse(x_fixed,y_fixed,P1,P2,phi);
%     end
    
    
    pause(0);
end

%%%%%%%% Post-process: Compute a path from goal to start using the tree
%%%%%%%% backwards
save_pathIndex = [];
if iter < K
    path.pos(1).x = xGoal; path.pos(1).y = yGoal;  
    path.pos(1).P = [P1 P2]; path.pos(1).phi = 0; %put the horizontal one because we are sure it will work because of the termination limits
    
    path.pos(2).x = tree.vertex(end).x; path.pos(2).y = tree.vertex(end).y;
    path.pos(2).P = tree.vertex(end).P; path.pos(2).phi = tree.vertex(end).phi;
    
    pathIndex = tree.vertex(end).ind;
    save_pathIndex = [save_pathIndex pathIndex];
    
    pathIndex = tree.vertex(end).indPrev;
    save_pathIndex = [save_pathIndex pathIndex];
    
    j=0;
    while 1
        path.pos(j+3).x = tree.vertex(pathIndex).x;
        path.pos(j+3).y = tree.vertex(pathIndex).y;
        
        path.pos(j+3).P = tree.vertex(pathIndex).P;
        path.pos(j+3).phi = tree.vertex(pathIndex).phi;
        
        pathIndex = tree.vertex(pathIndex).indPrev;
        save_pathIndex = [save_pathIndex pathIndex];
        
        if pathIndex == 1 %%reached the goal
            break
        end
        j=j+1;
    end
    path.pos(end+1).x = xInit; path.pos(end).y = yInit;
    
    for j = 1:length(path.pos)-2
          generate_ellipse(path.pos(j).x,path.pos(j).y,path.pos(j).P(1),path.pos(j).P(2),path.pos(j).phi,1);
    end
        
    for j=1:length(path.pos)-1
        plot(path.pos(j).x,path.pos(j).y,'ko-', 'LineWidth',2);
    end
    
    for j = 2:length(path.pos)
        
        plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'k-', 'Linewidth', 2);
         pause(0);
    end
    disp(['Minimum number of steps is ',num2str(length(path.pos)-1)]);
else
    disp('No path found. Increase number of iterations and retry.');
end
%axis('equal');
% ylim([0 3]);
% 
% set(gcf, 'PaperPosition', PaperPosition); %Position the plot further to the left and down. Extend the plot to fill entire paper.
% set(gcf, 'PaperSize', PaperSize); %Keep the same paper size
% if (print_pdf==1)
%     print(h,'-dpdf',string);
% end
  

%%% for debugging
% save_pathIndex
% for i=length(save_pathIndex)-1:-1:1 %length(save_pathIndex) should be ignored, repeated
%       ii = save_pathIndex(i);
%       disp([tree.vertex(ii).indPrev tree.vertex(ii).xPrev tree.vertex(ii).yPrev ...
%            tree.vertex(ii).ind tree.vertex(ii).x tree.vertex(ii).y ]) 
% end


%%%%% forward simulation %%%%%%
if (dynamic)
    X = xInit; Y = yInit;
    z_k = [X Y];
    save_z_k = z_k;
    save_t_k = 0;
    save_V_x_k = [];
    save_u_k = [];
    extra_steps = 10; %does simulation for extra steps, beyond the planned ones to show convergence
    for step=1:(length(path.pos)-2)+extra_steps 
        
        if step <= (length(path.pos)-2) %use save_pathIndex uptil length(path.pos)-2
            i = length(save_pathIndex)-step;             %length(save_pathIndex)-1:-1:1;
            ii = save_pathIndex(i);
            robot.zstar = tree.vertex(ii).zstar;    
        else %use last limit cycle for future steps
            robot.zstar = [xGoal yGoal];
        %    i = 1;
        end
        
        %step = step+1;
        
        
%         %%%% method 1: from saved zstar and controls (dont use if simulating more than (length(path.pos)-2) steps)
%         robot.zstar = tree.vertex(ii).zstar;
%         robot.control.theta = tree.vertex(ii).control_fit_theta; 
%         robot.control.Pc = tree.vertex(ii).control_fit_Pc; 
%         robot.control.Pt = tree.vertex(ii).control_fit_Pt; 
        
        %%% method 2: from save zstar, regenerate the control using zstar
        X = z_k(1); Y = z_k(2);
        
        [control_fit_theta,control_fit_Pc,control_fit_Pt] = main_fit_data_neural_net_postprocess(net_theta,net_Pc,net_Pt,X,Y,robot.m,robot.g,robot.zstar,cluster_data);
        robot.control.theta = control_fit_theta; 
        robot.control.Pc = control_fit_Pc;  
        robot.control.Pt = control_fit_Pt;
        save_u_k = [save_u_k; control_fit_theta control_fit_Pc control_fit_Pt];

        [zz,tt,z_extra] = onestep(z_k,robot,1);
        if (step==1)
            t_all = tt;
            z_all = zz;
            x_trans = zz(end,1);
            t_end = tt(end);
        else
            zz(:,1) = x_trans + zz(:,1);
            zz(:,9) = x_trans + zz(:,9);
            x_trans = zz(end,1);
            tt = tt + t_end;
            t_end = tt(end);
            t_all = [t_all; tt(2:end)];
            z_all = [z_all; zz(2:end,:)];
        end

        %%% zz = [x0 x0dot y0 y0dot E_kc0 E_kt0 E_Pc0 E_Pt0 xf yf];
        z_k(1) = zz(end,2); z_k(2) = zz(end,3);
        save_z_k = [save_z_k; z_k];
        save_t_k = [save_t_k; t_end];
        V_x_k = (z_k - robot.zstar)*P*(z_k' - robot.zstar');
        save_V_x_k = [save_V_x_k, V_x_k];
        X = z_k(1); Y = z_k(2);
        
        if (step==1)
            disp('z_k(1) z_k(2) zstar(1) zstar(2)');
        end
        if (step==(length(path.pos)-2)+1)
            disp('extra steps ...');
        end
        disp([z_k robot.zstar]) 
        sim(step).zstar = robot.zstar;
        sim(step).P = P;
        
        if ( (step>(length(path.pos)-2) && V_x_k < 0.11)) % || (step > (length(path.pos)-2) + 3) )
            break;
        end
        %disp(z_k)
    end
    
%     h=figure(2);
%     plot(save_z_k(2:end,1),save_z_k(2:end,2),'ro', 'MarkerSize',10, 'MarkerFaceColor','w'); hold on;
%     for i=1:length(save_z_k)-1
%         quiver(save_z_k(i,1),save_z_k(i,2),save_z_k(i+1,1)-save_z_k(i,1),save_z_k(i+1,2)-save_z_k(i,2),0,...
%                       'Color','red','Linewidth',2,'Autoscalefactor',0,'Autoscale','off','MaxHeadSize',0.2);
%         text(save_z_k(i+1,1),save_z_k(i+1,2)+0.1,num2str(i));
%     end
%   % plot(save_z_k(:,1),save_z_k(:,2),'r-');
%    plot(xInit,yInit,'kx', 'MarkerSize',10,'Linewidth',4);
%    plot(xGoal,yGoal,'ko', 'MarkerSize',10,'MarkerFaceColor','k');
%    axis('equal');
%    xlim([1.5 5.5]);
%    %ylim([1 2]);
%    xlabel('Apex horizontal velocity (m/s)','Fontsize',Fontsize); 
%    ylabel('Apex height (m)','Fontsize',Fontsize);
%    string = 'results/RRT';
%    set(gcf, 'PaperPosition', PaperPosition); %Position the plot further to the left and down. Extend the plot to fill entire paper.
%    set(gcf, 'PaperSize', PaperSize); %Keep the same paper size
%    if (print_pdf==1)
%         print(h,'-dpdf',string);
%    end
   
%     h=figure(3);
%     save_u_k
%     plot(save_u_k(:,1),'ro-','MarkerSize',10,'MarkerFaceColor','r','Linewidth',1); hold on
%     plot(save_u_k(:,2)/1000,'b*-','MarkerSize',10,'Linewidth',1); 
%     plot(save_u_k(:,3)/1000,'k^-','MarkerSize',10,'Linewidth',1,'MarkerFaceColor','k');
%     ylim([0 1.6]);
%     xlabel('Step number ','Fontsize',Fontsize); 
%     ylabel('Control action u_k ','Fontsize',Fontsize);
%     legend('theta','Pc/1000','Pt/1000','Location','Best');
%     string = 'results/control';
%     set(gcf, 'PaperPosition', PaperPosition); %Position the plot further to the left and down. Extend the plot to fill entire paper.
%     set(gcf, 'PaperSize', PaperSize); %Keep the same paper size
%     if (print_pdf==1)
%         print(h,'-dpdf',string);
%     end


   
   %save_V_x_k
  
%     fps = 20;
%     figure(4)
%     animate(t_all,z_all,robot,fps);
 
    robot.writeMovie = 1; %0 for no .avi
    disp('Next we will create animation as an avi file for saving.')
    disp('This will cause unusual figure windows, but let MATLAB finish')
    disp('Check the folder for two .avi file after MATLAB is done with animation');
    pause(2);
    fps = 30;
    t_k_save = save_t_k;
    %z_k_save = [z_source sim(current_limit_cycle).zstar];
    z_k_save = save_z_k;
    z_source = [xInit yInit];
    z_target = [xGoal yGoal];

    animate_w_text(t_all,z_all,robot,fps,t_k_save,z_k_save, sim, z_source, z_target);
    
end
