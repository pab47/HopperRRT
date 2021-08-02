% Modified from SlipRunner3 %%

%%% control is theta, Pc, and Pt %%
%%% works on single limit cycle
function main()

%if (nargin<1)
    clc
    clear all
    close all
    format long
% else 
%     close all
% end

%global history

PaperPosition = [-0.25 -0.1 8 6]; %location on printed page. rect = [left, bottom, width, height]
PaperSize = [7.25 5.8]; %[width height]
Fontsize = 12;
print_pdf = 0;
use_neural_net = 1;

%%%%%%%%%%%%%%%%%%% temp code %%%%%%%%%
% zstar = [5 1.4];
% robot = parms;
% theta = 0.347942564495251-0.05;
% kc = 3.200349281304472e+04;
% kt = kc;
% robot.control.theta = theta;
% robot.control.kc = kc;
% robot.control.kt = kt;
% steps = 1;
% [z,t] = onestep(zstar,robot,steps);
% fps = 10;
% animate(t,z,robot,fps);
%%%%%%%
% disp('Some plots...')
% figure(2)
% subplot(2,1,1)
% plot(t,z(:,1),'r',t,z(:,3),'b')
% xlabel('time'); ylabel('Angle (m)');
% legend('x','y');
% subplot(2,1,2)
% plot(t,z(:,2),'r',t,z(:,4),'b')
% xlabel('time'); ylabel('Velocity (m/s)');
% legend('vx','vy');
% 
% figure(3);
% subplot(2,1,1);
% plot(t,z(:,5),'r',t,z(:,6),'b');
% xlabel('time'); ylabel('energy');
% legend('E-kc','E-kt','Location','Best');
% subplot(2,1,2);
% plot(t,z(:,7),'r',t,z(:,8),'b');
% xlabel('time'); ylabel('energy');
% legend('E-Pc','E-Pt','Location','Best');
% 
% COT = (z(end,5)+z(end,6) + z(end,7)+z(end,8)) / (robot.m*robot.g*(z(end,1)-z(1,1)))

% figure(3)
% plot(z(:,1),z(:,3),'r'); %hold on
% xlabel('x'); ylabel('y');
% title('Trajectory y vs x');

%error(' ');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%% create the lyapunov function %%%%%

options = 32; %1 (not programmed) for finding if CLF exists for given lyapunov function
             %2 (not programmed) given a z_k does a forward simulation and an animation,
                  %21 does animation using data.mat created by running with option 2 
             %3 plots control strategy 
                %31 post processing (assumfing 3 is already run)
                %32 test control synergy (get new V_xk1_save for approximated control)
                     %writes the file data_****_fit.mat
                %33 post process 32          
           
if (options == 3 || options == 31 || options == 32 || options == 33 )
    %data_file = 'dataControl.mat';
    %data_file = 'data_lyap09.mat';
    %suffix = 'deacontrolcdbeat';
    
    if (options == 3 || options == 31)
        error('not programmed: try options = 32 first followed by 33');
    end
    
    %%%%%%%% case 1 %%%%%%%
    suffix = 'lyap1_09_'; %for [5 1.3]  
    %if (nargin<1)
        phi_deg = 0;
        %zstar = [3.5 1.3]; %first limit cycle
        %zstar = [2, 1.7]; %outside the domain
        zstar = [7, 1.3]; %outside the domain
    %else
    %    zstar = init_conditions(1:2);
    %    phi_deg = init_conditions(3);
    %end
    if (phi_deg<0)
        suffix = [suffix,num2str(10*zstar(1)),'_',num2str(10*zstar(2)),'_',num2str(abs(phi_deg)),'_n'];
    else %negative values
        suffix = [suffix,num2str(10*zstar(1)),'_',num2str(10*zstar(2)),'_',num2str(phi_deg)];
    end
    
    
%     if (zstar(2)<1.3)
%         error('zstar(2) or height should be >= 1.3');
%     end

    %%%%%% case 2 %%%%%%%%%
    %suffix = 'lyap2_09'; $for [5.65 1.3]
    %zstar = [5.65 1.3]; %second limit cycle

    
    data_file = ['data_',suffix,'.mat'];
    data_file_original = data_file;
    if (options == 31 )
        load(data_file);
    end
    %%%%% this is used for option 32 %%
   % data_file_train = {data_file}; %use this if you want to use same file for testing and training
%    data_file_train = {'data/data_lyap1_09_20_13_0.mat'};
    
%     data_file_train = {...
%         'data/data_lyap1_09_50_13_90.mat';...
%         'data/data_lyap1_09_20_13_90.mat'; ...
%         'data/data_lyap1_09_30_14_90.mat';...
%         'data/data_lyap1_09_40_16_90.mat';...
%         'data/data_lyap1_09_20_15_90.mat';...
%         'data/data_lyap1_09_50_14_90.mat'};

   data_file_train = {'data_lyap1_09_20_13.mat';...
                'data_lyap1_09_20_16.mat'; ...
                'data_lyap1_09_30_15.mat';...
                'data_lyap1_09_40_14.mat';...
                'data_lyap1_09_40_16.mat';...
                'data_lyap1_09_50_14.mat';...
                'data_lyap1_09_50_16.mat'};

     
    %data_file = 'data_deadbeat.mat';
%     if (robot.only_theta == 1)
%         data_file = 'data_theta.mat';
%     end
   if (options == 32 || options == 33)
       if (options == 32)
%           load(data_file_train);
            if (use_neural_net==1)
                %[control_fit_theta,control_fit_Pc,control_fit_Pt,net_theta,net_Pc,net_Pt] = main_fit_data_neural_net(data_file);
                [net_theta,net_Pc,net_Pt,cluster_data,robot] = main_fit_data_neural_net(data_file_train);
                robot.zstar = zstar; %write the zstar for training
                save('control_policy.mat','net_theta','net_Pc','net_Pt','cluster_data','robot');
                %error('error');
            else
                error('not programmed');
                %[gains_control_theta,gains_control_Pc, gains_control_Pt] = main_fit_data(data_file);
            end
           close all
       end
       data_file =  ['data_',suffix,'_fit.mat'];
       %load(data_file);
   end
end

if (options == 33)
    suffix = [suffix,'_fit']; %for plots
    load(data_file);
end

if (options == 3)
    
    %%%%%%% old code %%%%%%%%%%%%%
%     [x,F,INFO] = lyapmain(zstar);
%     p11 = x(1);
%     p12 = 0;
%     p22 = x(2);
%     P = [p11 p12; p12 p22]
%     robot = parms(P,zstar);  %set P and zstar
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%
%     robot = parms;
%     p11 = (robot.c)/((robot.a)^2);
%     p22 = (robot.c)/((robot.b)^2);
%     p12 = 0;
%     P = [p11 p12; p12 p22];
    
    phi_rad = (pi/180)*phi_deg;
    %P = [1 0; 0 11.11111111];
    %PP = diag([1,11.11111111,phi_rad]); %packing P and phi to prevent 
    PP = diag([100,11.1111,phi_rad]);
    robot = parms(PP,zstar);
    P = robot.P;
    phi_rad = robot.phi;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%%% compute limit cycle %%%%%%%%
    kc0 = 40*robot.m*robot.g;
    theta0  = 20*(pi/180); %10*(pi/180) %angle between leg and vertical
    [theta,kc,kt] = limit_cycle(kc0,theta0)
    theta0 = theta; kc0=kc; %set the nominal values for this gait.
    temp.kc = kc; temp.theta = theta;
    robot = parms(PP,zstar,temp);  % sets kc
    theta

    robot.control.theta = theta;
    J=partialder(@onestep,zstar,robot)
    eig(J)
    [~,~,z_extra] = onestep(zstar,robot);
    %%%z0 = [x0 x0dot y0 y0dot E_kc0 E_kt0 E_Pc0 E_Pt0];
    E_kc0 = z_extra(1); E_kt0 = z_extra(2); E_Pc0 = z_extra(3); E_Pt0 = z_extra(4); D = z_extra(5);
    disp('MCOT for nominal trajectory is');
    MCOT = (E_kc0+E_kt0+E_Pc0+E_Pt0)/(robot.m*robot.g*D)
    %error(' ');
    %[zz,tt] = onestep(zstar,robot,1);
    %animate(tt,zz,robot,30);
else
    P = robot.P;
    if (isfield(robot,'phi'))
        phi_rad = robot.phi;
    else
        phi_rad = 0;
    end
    zstar = robot.zstar;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%error(' ');

if (options ==3 || options == 31 || options == 32 || options == 33)

 if (options==3 || options == 32)   
    %test = 0;
    %if (test)
    %    save_X = level_set(robot.P,robot.c); %get values 
    %else
    %%%% create grid points %%%%
    if (options==3) %training set
      c = linspace(0,1.0,5); %11
     %c = [0.1 0.25];
      refine = 2; %put used 5x21 = 105 data points
    else      %test set
%      c = linspace(0,1.0,6); %11
%      refine = 3; %put 4 %451 points
     c = linspace(0,1.0,11); %11
     refine = 4; %put 4 %451 points
    end
     clear X Y
    for i=1:length(c)
        %phi_rad = robot.phi_rad;
        save_X = level_set(P,phi_rad,c(i),refine);
        X(i,:) = robot.zstar(1) + save_X(:,1)';
        Y(i,:) = robot.zstar(2) + save_X(:,2)';
    end

   
    
    if (options == 32)
        if (use_neural_net==0)
            error('not programmed');
%             TE = 0.5*robot.m*robot.zstar(1)*robot.zstar(1) + robot.m*robot.g*robot.zstar(2);
%             [~,control_fit_theta] = fn_control_theta(gains_control_theta,X,Y,control_theta,robot);
%             [~,control_fit_Pc] = fn_control_Pc(gains_control_Pc,X,Y,control_Pc,TE,robot);
%             [~,control_fit_Pt] = fn_control_Pt(gains_control_Pt,X,Y,control_Pt,TE,robot);
        else
            %size(X)
            [control_fit_theta,control_fit_Pc,control_fit_Pt] = main_fit_data_neural_net_postprocess(net_theta,net_Pc,net_Pt,X,Y,robot.m,robot.g,robot.zstar,cluster_data);
            %size(control_fit_theta)
        end
    end
    %end
    %%%%%%%%%% testing %%%%% 
    %          

       %control_fit_theta
       
%     x = -2:0.2:2;
%     y = -2:0.2:3;
%     [X,Y] = meshgrid(x,y);
%     Z = X.*exp(-X.^2-Y.^2);
 
% z_k_save = [];
% V_xk_save = [];
% E_xk_save = [];
% control_theta = []; control_Pc = []; control_Pt = [];
% exitflag = [];

%for i=1:length(save_X)
[mmm,nnn] = size(X);
run_time = 0;
tic;
runs = 0;
%all_history = [];
for i=1:mmm %loops over X
    for j=1:nnn %loops of Y
        disp('%%%%%%%%%%%%%%%%%%% ');
%         if (nargin==1)
%             disp(['[zstar phi_deg] = ',num2str(init_conditions)]);
%         end
        disp(['run = ',num2str(runs)]);
        if (options==3)
            pause(1);
        end
        disp(' ');
        runs = runs+1;
        
        %if (test)
        %    z_k =  [robot.zstar(1) + save_X(i,1); robot.zstar(2) + save_X(i,2)]' %test data point
        %else
            z_k = [X(i,j) Y(i,j)];
            z_k_save{i,j} = z_k;

       x_k = z_k' - robot.zstar';
       P = robot.P;
       if (isfield(robot,'phi'))    
           phi_rad = robot.phi;
       else
           phi_rad = 0;
       end
       P_rotated = get_P_rotated(P,phi_rad);
       
       V_xk = x_k'*P_rotated*x_k;
       V_xk_save{i,j} = V_xk;
    %      if (i==1)
       %V_xk_save = [V_xk_save; V_xk];
    %      end
        
    if (options == 3)
%         if (j==nnn)
%             %Z(i,n) = Z(i,1);
%             x(3) = control_theta(i,1);
%             x(4) = control_Pc(i,1);
%             x(5) = control_Pt(i,1);
%             flag = exitflag(i,1);
%         else
           % z_k = [5.154508497187473 1.542658477451406]; 
           if (z_k(2) > 1.0)
%                z_k = [4.911832212151721   1.704508497187474];
                [x,F,flag] = snoptmain(z_k);
%                 x
%                 F 
%                 flag
%                 hi
           else
               disp(' ');
               disp(['height = ',num2str(z_k(2)),'<1.0']);
               disp(' Skipping ...');
               pause(1);
           end
%        end
%
        
        flag
        %pause(1);
        %exitflag = [exitflag; flag];
        exitflag(i,j) = flag;
        %all_history = [all_history; history];
        
%         x
%         z_k
%         error(' ');
%        ddd
%         flag
%         z_k
%         x
%         disp('test');
    end
    
    if (z_k(2) > 1.0)
        if (options == 3)
            robot.control.theta=x(3); 
            robot.control.Pc = x(4); 
            robot.control.Pt = x(5);
            control_theta(i,j) = x(3);
            control_Pc(i,j) = x(4);
            control_Pt(i,j) = x(5);
        end
        if (options == 32)
            [control_fit_theta_temp,control_fit_Pc_temp,control_fit_Pt_temp] = main_fit_data_neural_net_postprocess(net_theta,net_Pc,net_Pt,X(i,j),Y(i,j),robot.m,robot.g,robot.zstar,cluster_data);
            robot.control.theta = control_fit_theta_temp;
            robot.control.Pc = control_fit_Pc_temp;
            robot.control.Pt = control_fit_Pt_temp;
            control_theta(i,j) = control_fit_theta_temp;
            control_Pc(i,j) = control_fit_Pc_temp;
            control_Pt(i,j) = control_fit_Pt_temp;
%             robot.control.theta = control_fit_theta(i,j); 
%             robot.control.Pc = control_fit_Pc(i,j);  
%             robot.control.Pt = control_fit_Pt(i,j); 
%             control_theta(i,j) = control_fit_theta(i,j);
%             control_Pc(i,j) = control_fit_Pc(i,j);
%             control_Pt(i,j) = control_fit_Pt(i,j);
            exitflag(i,j) = 0;
            theta0 = 0; kc0 = 0; %placeholder
            E_kc0=0; E_kt0=0; E_Pc0=0; E_Pt0=0; %placeholder 
        end
    else
            control_theta(i,j) = NaN;
            control_Pc(i,j) = NaN;
            control_Pt(i,j) = NaN;
            exitflag(i,j) = 0;
    end
  
                 
        
        %control_theta = [control_theta; x(3)];
        %control_Pc = [control_Pc; x(4)];
        %control_Pt = [control_Pt; x(5)];
        
     if (z_k(2) > 1.0)
        [zz,tt,z_extra] = onestep(z_k,robot,1);
        %robot.x0 = zz(end,1);
        %E_kc = z_extra(1)-E_kc0; E_kt = z_extra(2)-E_kt0; E_Pc = z_extra(3)-E_Pc0; E_Pt = z_extra(4)-E_Pt0;
        E_kc = z_extra(1); E_kt = z_extra(2); E_Pc = z_extra(3); E_Pt = z_extra(4);
        E_xk_save{i,j} = [E_kc E_kt E_Pc E_Pt];
        D_xk_save{i,j} = z_extra(5);
        

    %         robot.E_kc = zz(end,5);
    %         robot.E_kt = zz(end,6);
    %         robot.E_Pc = zz(end,7);
    %         robot.E_Pt = zz(end,8);
    %     if (i==1)
    %         z = [z;zz]; t = [t;tt];
    %     else
    %         tt = tt + t(end);
    %         z= [z;zz(2:end,:)]; t=[t;tt(2:end)];
    %     end

         z_k1 = [zz(end,2), zz(end,3)];
         %z_k_save = [z_k_save; z_k z_k1];
         z_k1_save{i,j} = z_k1;
         

         
         x_k1 = z_k1' - robot.zstar';
         P = robot.P;
         if (isfield(robot,'phi'))    
           phi_rad = robot.phi;
         else
           phi_rad = 0;
         end
         P_rotated = get_P_rotated(P,phi_rad);
         V_xk1 = x_k1'*P_rotated*x_k1;
         %V_xk_save = [V_xk_save; V_xk V_xk1];
         V_xk1_save{i,j} = V_xk1; 
         
     else
         z_k1_save{i,j} =  [NaN NaN NaN NaN];  
         E_xk_save{i,j} = NaN;
         D_xk_save{i,j} = NaN;
         V_xk1_save{i,j} = NaN;
     end
     
                
             

    %         figure(1)
    %          fps = 30;
    %         animate(tt,zz,robot,fps);

        %%%% simulate one step to get energy cost %%
    %end

        %%% do contour plot
    % 
    %     figure(1)
    %     contour(X,Y,Z,'ShowText','on')
    save('temp.mat','robot','V_xk_save', 'V_xk1_save','z_k_save','z_k1_save',...
                 'control_Pc','control_Pt','control_theta','E_xk_save','exitflag',...
                 'theta0','kc0','E_kc0','E_kt0','E_Pc0','E_Pt0','X','Y','D_xk_save');
    %save('temp2.mat','all_history');
    end
end
% poincare_file = ['poincare_',suffix,'.mat'];
% save(poincare_file,'all_history','robot');

run_time = toc 

%         save('dataControl.mat','robot','V_xk_save','z_k_save',...
%                  'control_Pc','control_Pt','control_theta','E_xk_save','exitflag',...
%                  'theta0','kc0','E_kc0','E_kt0','E_Pc0','E_Pt0');
        save(data_file,'robot','V_xk_save', 'V_xk1_save','z_k_save','z_k1_save',...
                 'control_Pc','control_Pt','control_theta','E_xk_save','exitflag',...
                 'theta0','kc0','E_kc0','E_kt0','E_Pc0','E_Pt0','X','Y','run_time','D_xk_save');
             
 end
     % load(data_file);
      
      disp('Run time in hrs');
      time_hrs = run_time/3600
      [mmm,nnn] = size(X);
      
      disp('sec per sim');
      average_time_sim = run_time / (mmm*nnn)
      
      %disp('exit flag should be 1');
      %exitflag
%       disp('V_xk1_save = 0.1*V_xk_save');
%       V_xk_save
%       V_xk1_save
%       disp('controls: theta, Pc, Pt');
%       control_theta
%       control_Pc
%       control_Pt
      
    if (1)
%      disp('Energy: kc kt Pc Pt');
      for i=1:mmm
          for j=1:nnn
            %disp(E_xk_save{i,j})
           if (exitflag(i,j) ==1000)
                E_xk_K(i,j) = NaN;
                E_xk_Pc(i,j) = NaN;
                E_xk_Pt(i,j) = NaN;
                control_Pc(i,j) = NaN;
                control_Pt(i,j) = NaN;
                control_theta(i,j) = NaN;
                  
           else
                temp = E_xk_save{i,j};
                if (length(temp)==1)
                    temp = [NaN   NaN   NaN   NaN];
                end
                E_xk_K(i,j) = temp(1)+temp(2);
                E_xk_Pc(i,j) = temp(3);
                E_xk_Pt(i,j) = temp(4);
                temp2 = D_xk_save{i,j};
                D_xk(i,j) = temp2;
            end
          end
      end
      
%       for i=1:mmm
%           for j=1:nnn
%               if (control_Pc(i,j)>5000 || control_Pt(i,j)>5000)
%                 E_xk_K(i,j) = NaN;
%                 E_xk_Pc(i,j) = NaN;
%                 E_xk_Pt(i,j) = NaN;
%                 control_Pc(i,j) = NaN;
%                 control_Pt(i,j) = NaN;
%                 control_theta(i,j) = NaN;
%               end
%           end
%       end

      %control_Pt
%       E_xk_K
%       E_xk_Pc
%       E_xk_Pt
      %E_xk_save2 = [E_xk_save(:,1)+E_xk_save(:,2) E_xk_save(:,3) E_xk_save(:,4) z_k_save(:,1:2)]
      %controls = [control_theta-theta0 control_Pc/1000 control_Pt/1000 z_k_save(:,1:2)]
      
      %%%%%%%% energy line  %%%%%%%%%%%%%%
      TE = 0.5*robot.m*robot.zstar(1)*robot.zstar(1) + robot.m*robot.g*robot.zstar(2);
      save_X = level_set(P,1,10);
      vx(:,1) = robot.zstar(1) + save_X(:,1)';
      %  Y(i,:) = robot.zstar(2) + save_X(:,2)';
      for i=1:length(vx)
        %vx(i,1) = z_k_save(i,1);
        h(i,1) = (TE - 0.5*robot.m*vx(i,1)*vx(i,1))/(robot.m*robot.g);
      end
   
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      %[m,n] = size(z_k_save);
     save_X = level_set(P,1,10);
     
     color_map = 'cool';
     color_map = 'gray';
     color_map = 'bone';
     color_map = 'spring';
     test_subplot = 0;
     
      hh=figure(1);
      if (test_subplot)
        subplot(3,1,1);
      end
      contourf(X,Y,control_theta-theta0,10,'ShowText','on')
      %colormap(jet)
      %colormap(color_map)
      colormap('cool');
      %colormap(flipud(gray))
      hold on;
%      clabel(CCC,'FontSize',12,'Color','black','EdgeColor','w')
      %clabel(CCC,hhh,'EdgeColor','r')
%       plot(z_k_save(:,1),z_k_save(:,2),'ko'); hold on;
%       for i=1:length(z_k_save)
%           text(z_k_save(i,1)+0.02,z_k_save(i,2)+0.02,num2str((control_theta(i,1)-theta0),3));
%       end
      plot(vx,h,'k-.','Linewidth',1);
      %plot(save_X(:,1)+zstar(1),save_X(:,2)+zstar(2),'k-.');
      plot(zstar(1),zstar(2),'k+','MarkerSize',10,'LineWidth',2);
      %if (zstar(1)==5)
          axis('equal');
          %ylim([zstar(2)-0.3 zstar(2)+0.3]);
      %end
      title('Control theta','Fontsize',Fontsize);
      xlabel('vx','Fontsize',Fontsize); ylabel('y','Fontsize',Fontsize);
      string = ['results/',suffix,'_theta'];
        set(gcf, 'PaperPosition', PaperPosition); %Position the plot further to the left and down. Extend the plot to fill entire paper.
        set(gcf, 'PaperSize', PaperSize); %Keep the same paper size
        if (print_pdf==1)
            print(hh,'-dpdf',string);
        end

%error(' ');
      hh=figure(2);
      if (test_subplot)
        hh = figure(1);
        subplot(3,1,2);
      end
      contourf(X,Y,control_Pc,'ShowText','on'); hold on; %Try 'LevelStep',100
      colormap(flipud(autumn));
%       plot(z_k_save(:,1),z_k_save(:,2),'bo'); hold on;
%       for i=1:length(z_k_save)
%           text(z_k_save(i,1)+0.02,z_k_save(i,2)+0.02,num2str(control_Pc(i,1)/1000,3));
%       end
      plot(vx,h,'k-.','Linewidth',1);
      %plot(save_X(:,1)+zstar(1),save_X(:,2)+zstar(2),'k-.'); 
      plot(zstar(1),zstar(2),'k+','MarkerSize',10,'LineWidth',2);
      %if (zstar(1)==5)
          axis('equal');
          %ylim([zstar(2)-0.3 zstar(2)+0.3]);
      %end
      title('Control Pc','Fontsize',Fontsize); 
      xlabel('vx','Fontsize',Fontsize); ylabel('y','Fontsize',Fontsize);
       %string = 'results/contour_Pc';
       string = ['results/',suffix,'_Pc'];
        set(gcf, 'PaperPosition', PaperPosition); %Position the plot further to the left and down. Extend the plot to fill entire paper.
        set(gcf, 'PaperSize', PaperSize); %Keep the same paper size
        if (print_pdf==1)
            print(hh,'-dpdf',string);
        end

%         for i=1:mmm
%           for j=1:nnn
%               if (exitflag(i,j)==1)
%                 plot(X(i,j),Y(i,j),'ro');
%               end
%           end
%         end

  
      hh=figure(3);
      if (test_subplot)
        hh = figure(1);
        subplot(3,1,3);
      end
      contourf(X,Y,control_Pt,'ShowText','on'); hold on;
      colormap(flipud(summer));
%       plot(z_k_save(:,1),z_k_save(:,2),'ro'); hold on;
%       for i=1:length(z_k_save)
%           text(z_k_save(i,1)+0.02,z_k_save(i,2)+0.02,num2str(control_Pt(i,1)/1000,3));
%       end
      plot(vx,h,'k-.','Linewidth',1);
      %plot(save_X(:,1)+zstar(1),save_X(:,2)+zstar(2),'k-.'); 
      plot(zstar(1),zstar(2),'k+','MarkerSize',10,'LineWidth',2);
      title('Control Pt','Fontsize',Fontsize); 
      %if (zstar(1)==5)
          axis('equal');
          %ylim([zstar(2)-0.3 zstar(2)+0.3]);
      %end
       xlabel('vx','Fontsize',Fontsize); ylabel('y','Fontsize',Fontsize);
        %string = 'results/contour_Pt';
        string = ['results/',suffix,'_Pt'];
        set(gcf, 'PaperPosition', PaperPosition); %Position the plot further to the left and down. Extend the plot to fill entire paper.
        set(gcf, 'PaperSize', PaperSize); %Keep the same paper size
        if (print_pdf==1)
            print(hh,'-dpdf',string);
        end

      %D_xk_save
      hh=figure(4);
      
 
      contourf(X,Y,E_xk_K./(robot.m*robot.g*D_xk),10,'ShowText','on'); hold on;
      colormap('cool');
      plot(vx,h,'k-.','Linewidth',1);
      %plot(save_X(:,1)+zstar(1),save_X(:,2)+zstar(2),'k-.'); 
      plot(zstar(1),zstar(2),'k+','MarkerSize',10,'LineWidth',2);
      title('Energy due to spring','Fontsize',Fontsize);
      %if (zstar(1)==5)
          axis('equal');
          %ylim([zstar(2)-0.3 zstar(2)+0.3]);
      %end
       xlabel('vx','Fontsize',Fontsize); ylabel('y','Fontsize',Fontsize);
        %string = 'results/contour_E_K';
        string = ['results/',suffix,'_E_K'];
        set(gcf, 'PaperPosition', PaperPosition); %Position the plot further to the left and down. Extend the plot to fill entire paper.
        set(gcf, 'PaperSize', PaperSize); %Keep the same paper size
        if (print_pdf==1)
            print(hh,'-dpdf',string);
        end
     
      
      hh=figure(5);
      contourf(X,Y,E_xk_Pc./(robot.m*robot.g*D_xk),'ShowText','on'); hold on;
      colormap(flipud(autumn));
      plot(vx,h,'k-.','Linewidth',1);
      %plot(save_X(:,1)+zstar(1),save_X(:,2)+zstar(2),'k-.'); 
      plot(zstar(1),zstar(2),'k+','MarkerSize',10,'LineWidth',2);
      title('Energy due to Pc','Fontsize',Fontsize); 
      xlabel('vx','Fontsize',Fontsize); ylabel('y','Fontsize',Fontsize);
      %if (zstar(1)==5)
          axis('equal');
          %ylim([zstar(2)-0.3 zstar(2)+0.3]);
      %end
       %string = 'results/contour_E_Pc';
       string = ['results/',suffix,'_E_Pc'];
        set(gcf, 'PaperPosition', PaperPosition); %Position the plot further to the left and down. Extend the plot to fill entire paper.
        set(gcf, 'PaperSize', PaperSize); %Keep the same paper size
        if (print_pdf==1)
                        print(hh,'-dpdf',string);
        end
      
      hh=figure(6);
      contourf(X,Y,E_xk_Pt./(robot.m*robot.g*D_xk),'ShowText','on'); hold on;
      colormap(flipud(summer));
      plot(vx,h,'k-.','Linewidth',1);
      %plot(save_X(:,1)+zstar(1),save_X(:,2)+zstar(2),'k-.'); 
      plot(zstar(1),zstar(2),'k+','MarkerSize',10,'LineWidth',2);
      title('Energy due to Pt','Fontsize',Fontsize); 
     %if (zstar(1)==5)
          axis('equal');
          %ylim([zstar(2)-0.3 zstar(2)+0.3]);
      %end
       xlabel('vx','Fontsize',Fontsize); ylabel('y','Fontsize',Fontsize);
     %string = 'results/contour_E_Pt';
     string = ['results/',suffix,'_E_Pt'];
        set(gcf, 'PaperPosition', PaperPosition); %Position the plot further to the left and down. Extend the plot to fill entire paper.
        set(gcf, 'PaperSize', PaperSize); %Keep the same paper size
        if (print_pdf==1)
                        print(hh,'-dpdf',string);
        end
        
        hh=figure(7);
      contourf(X,Y,(E_xk_K+E_xk_Pc+E_xk_Pt)./(robot.m*robot.g*D_xk),'ShowText','on'); hold on;
      colormap(flipud(gray));
      plot(vx,h,'k-.','Linewidth',1);
      %plot(save_X(:,1)+zstar(1),save_X(:,2)+zstar(2),'k-.'); 
      plot(zstar(1),zstar(2),'k+','MarkerSize',10,'LineWidth',2);
      title('Energy due to K Pc Pt','Fontsize',Fontsize); 
     %if (zstar(1)==5)
          axis('equal');
          %ylim([zstar(2)-0.3 zstar(2)+0.3]);
      %end
       xlabel('vx','Fontsize',Fontsize); ylabel('y','Fontsize',Fontsize);
     %string = 'results/contour_E_Pt';
     string = ['results/',suffix,'_E_all'];
        set(gcf, 'PaperPosition', PaperPosition); %Position the plot further to the left and down. Extend the plot to fill entire paper.
        set(gcf, 'PaperSize', PaperSize); %Keep the same paper size
        if (print_pdf==1)
                        print(hh,'-dpdf',string);
        end
    end
        if (options == 33)
           V_xk1_save = cell2mat(V_xk1_save);
           V_xk_save = cell2mat(V_xk_save);
           
           [m,n] = size(V_xk1_save);
           count1 = 0; count2 = 0; count3 = 0; count_rest = 0;
           bins = [];
           for i=1:m
               for j=1:n
                   if (isnan(V_xk1_save(i,j)))
                       continue;
                   end
                   if (V_xk1_save(i,j) <= 0.1)
                       count1 = count1+1;
                       %break;
                   elseif (V_xk1_save(i,j) <=0.2)
                       count2 = count2+1;
                       %break;
                   elseif (V_xk1_save(i,j) <=0.3)
                       count3 = count3+1;
                       %break
                   else
                       count_rest = count_rest + 1;
                       %warning('Found value not in any limit');
                       %pause(0.5);
                   end
               end
           end
           
           total = count1+count2+ count3+count_rest
           disp(['less than 0.1, % = ',num2str(100*count1/total)]);
           disp(['less than 0.2, % = ',num2str(100*count2/total)]);
           disp(['less than 0.3, % = ',num2str(100*count3/total)]);
           disp(['greater than 0.3, % = ',num2str(100*count_rest/total)]);
           disp(['worst case V_xk1 is = ',num2str(max(max(V_xk1_save)))]);
           
           temp_bin = [count1 count2 count3 count_rest]/(0.01*total);
           
           bins = [bins, temp_bin']; %each column should be V_xk for a step
           
           disp('init V,  final V, xdot, y'); 
           for i=1:m
               for j=1:n 
                   if (~isnan(V_xk1_save(i,j)) && V_xk1_save(i,j) > 1 )
                   disp([V_xk_save(i,j) V_xk1_save(i,j) X(i,j) Y(i,j)]);
                   end
               end
            end
           %[reshape(V_xk_save,[],1) reshape(V_xk1_save,[],1) reshape(X,[],1) reshape(Y,[],1)]
              
           ratio = V_xk_save./V_xk1_save; %original/new
           count = 0;
           for i=1:m
               for j=1:n
                   if (ratio(i,j) < 1 && V_xk_save(i,j)>0.1)
                       V_xk_save(i,j);
                       V_xk1_save(i,j);
                       count = count+1;
                   end
               end
           end
           disp(['The number of init conditions that did not decrease were = ',num2str(count)]);
           
            %%%%%%%%%%%% histogram plot 
           bins
            hh=figure(8);
            bar(bins);
            grid on;
            name_xaxis = {'0-0.1',...
                   '0.1-0.2',...
                   '0.2-0.3',...
                   '0.3-'
                   };
               [mn,nm] = size(bins);
               
%              string = 'legend(';
%              for jj=1:nm
%                  string = [string,'''step',num2str(jj),''''];
%                  if jj<nm
%                      string = [string,','];
%                  else
%                      string = [string,')'];
%                  end
%              end
%             eval(string);
              
            ylim([0 100]);
                set(gca,'xticklabel',name_xaxis,'Fontsize',14);
                %set(gca,'ylim',[0 45]);
                title(['Vxk1 zstar=\{',num2str(zstar(1)),',',num2str(zstar(2)),'\}']);
               string = ['results/',suffix,'_V_xk1'];
                set(gcf, 'PaperPosition', PaperPosition); %Position the plot further to the left and down. Extend the plot to fill entire paper.
                set(gcf, 'PaperSize', PaperSize); %Keep the same paper size
                if (print_pdf==1)
                        print(hh,'-dpdf',string);
                end
                
        end     
end

       
