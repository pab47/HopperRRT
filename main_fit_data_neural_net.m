%%%%% new: TE dividing line, one fit for each control %%%%%%%

%function [control_fit_theta,control_fit_Pc,control_fit_Pt,net_theta,net_Pc,net_Pt] = main_fit_data_neural_net(data_file,net_theta,net_Pc,net_Pt,XX,YY)
function [net_theta,net_Pc,net_Pt,cluster_data,robot] = main_fit_data_neural_net(data_file)

if (nargin==0)
    clc
    close all
    clear all
end

Fontsize = 12;
cluster_data = 1; %0 no clustering, 1 for clustering
fig_no = 1;
  
if (nargin==0)
    %clear variables
    % 1 file   
    %data_file = {'data_lyap1_09_50_14.mat'};
    %data_file = {'data_lyap1_09_50_14.mat';...
    %    'data_lyap1_09_30_15.mat'};
%     data_file = {'data_lyap1_09_50_14.mat';...
%         'data_lyap1_09_30_15.mat'; ...
%         'data_lyap1_09_20_13.mat';...
%         'data_lyap1_09_40_14.mat'};
   data_file = {'data_lyap1_09_20_13.mat';...
        'data_lyap1_09_20_16.mat'; ...
        'data_lyap1_09_30_15.mat';...
        'data_lyap1_09_40_14.mat';...
        'data_lyap1_09_40_16.mat';...
        'data_lyap1_09_50_14.mat';...
        'data_lyap1_09_50_16.mat'};
%    data_file = {'data/data_lyap1_09_30_14_90.mat'};
else
    %load(data_file);
end


for iiii=1:3

n1 = 1;
data_type = iiii;
rand('seed',0);

%%%%%%%%%%%%%% 1. Pull out the data from the files %%%%%%%%%%%%%%
for pppp=1:length(data_file)
%pppp=1;
load(data_file{pppp});

%%%%% needs to be rewritten (PAB47)
% if (nargin>1) %overwrite X and Y (do this later) 
%     X = XX;
%     Y = YY; 
% end

if (data_type==1)
    string_title = 'Control theta';
    data = control_theta;
    %all_data(pppp).control_theta = control_theta;
elseif (data_type ==2)
    string_title = 'Control Pc';
    data = control_Pc;
    %all_data(pppp).control_Pc = control_Pc;
elseif (data_type ==3)
    string_title = 'Control Pt';
    data = control_Pt;
    %all_data(pppp).control_Pt = control_Pt;
else
    error('data_type can be 1, 2, or 3');
end
all_data(pppp).data = data;
all_data(pppp).X = X;
all_data(pppp).Y = Y;
all_data(pppp).zstar = robot.zstar;
all_data(pppp).theta0 = theta0;

TE0 = 0.5*robot.m*robot.zstar(1)*robot.zstar(1) + robot.m*robot.g*robot.zstar(2);
all_data(pppp).TE0 = TE0;


[mmm,nnn] = size(X);
ppp = mmm*nnn;


   %if (data_type == 2 || data_type==3)
  % if (nargin<2) 
%     for i=1:mmm
%       for j=1:nnn
%           if (data(i,j)>8000)
%             data(i,j) = NaN;
%             %graphingZ1(i,j) = NaN;
%           end
%       end
%     end
 %  end
  
    k1 = 1;
    if (cluster_data == 1 && data_type ~= 1)
        for i=1:mmm
            for j=1:nnn
                vx_temp = X(i,j);
                h_temp = Y(i,j);
                TE_temp =  0.5*robot.m*vx_temp*vx_temp + robot.m*robot.g*h_temp;
                %del_vx = vx_temp - robot.zstar(1);
                %del_h = h_temp - robot.zstar(2);

                %if (TE>TE_temp)
                %if (TE_temp<controls(7))

                if (data_type == 2) %Pc
                    if (TE_temp>TE0) %only save TE > TE0 for fitting  
                         netrainX(n1,1) = X(i,j); %        F(i,j) = 0;
                         netrainX(n1,2) = Y(i,j);
                         netrainX(n1,3) = robot.zstar(1); %velocity
                         netrainX(n1,4) = robot.zstar(2); %apex height
                         
                         all_data(pppp).netrainX(k1,1) = X(i,j);
                         all_data(pppp).netrainX(k1,2) = Y(i,j);
                         all_data(pppp).netrainX(k1,3) = robot.zstar(1);
                         all_data(pppp).netrainX(k1,4) = robot.zstar(2);
                         
                         %if (nargin<2)
                         if (exitflag(i,j) == 1) % || exitflag(i,j) == 41)
                            netrainY1(n1,1) = data(i,j); 
                         else
                            netrainY1(n1,1) = NaN;
                         end
                         %end
                         all_data(pppp).index(k1,:) = [i,j];
                         k1 = k1+1;
                         index(n1,:) = [i,j];
                         n1 = n1+1;
                    end
                end
                if (data_type == 3) %Pt
                    if (TE_temp<TE0) %only save TE < TE0 for fitting
                        netrainX(n1,1) = X(i,j);
                        netrainX(n1,2) = Y(i,j);
                        netrainX(n1,3) = robot.zstar(1); %velocity
                        netrainX(n1,4) = robot.zstar(2); %apex height
                        
                        all_data(pppp).netrainX(k1,1) = X(i,j);
                        all_data(pppp).netrainX(k1,2) = Y(i,j);
                        all_data(pppp).netrainX(k1,3) = robot.zstar(1);
                        all_data(pppp).netrainX(k1,4) = robot.zstar(2);
                         
                        %if (nargin<2)
                        if (exitflag(i,j) == 1) % || exitflag(i,j) == 41)
                            netrainY1(n1,1) = data(i,j);
                        else
                            netrainY1(n1,1) = NaN;
                        end
                        %end
                        all_data(pppp).index(k1,:) = [i,j];
                        k1 = k1+1;
                        index(n1,:) = [i j];
                        n1 = n1+1;
                    end
                end
                
            end
        end
    else
        for i=1:mmm
            for j=1:nnn
                netrainX(n1,1) = X(i,j);
                netrainX(n1,2) = Y(i,j);
                netrainX(n1,3) = robot.zstar(1); %velocity
                netrainX(n1,4) = robot.zstar(2); %apex height
                
                all_data(pppp).netrainX(k1,1) = X(i,j);
                all_data(pppp).netrainX(k1,2) = Y(i,j);
                all_data(pppp).netrainX(k1,3) = robot.zstar(1);
                all_data(pppp).netrainX(k1,4) = robot.zstar(2);
                %if (nargin<2)
                if (exitflag(i,j) == 1) % || exitflag(i,j) == 41)
                    netrainY1(n1,1) = data(i,j);
                else
                    netrainY1(n1,1) = NaN;
                end
                %end
                k1 = k1+1;
                n1 = n1 + 1;
            end
        end
    end  
end
    
    %%%%%%%%% 2. Data fitting %%%%%%%%%%%%%%%%%%%%%%
    %net1 = fitnet([10,8,6]);
     %if (nargin<2)
        net1 = fitnet(12);
        %view(net1)
        net1 = train(net1,netrainX',netrainY1');
     %else
%          if (data_type == 1)
%              net1 = net_theta;
%          elseif (data_type == 2)
%              net1 = net_Pc;
%          else
%              net1 = net_Pt;
%          end
     %end

    %%%%%%%%%%% 3 a. Forecasting (errors) %%%%%%%%%%%%%%
    %mean_sq_error = mse(net1,netrainX',netrainY1')
    %mean_sq_error/ppp
    % forecast(1).Y = net1(netestX')';
    sum_ppp = 0;
    sum = 0;
    for pppp=1:length(data_file)
        %forecast(1).Y = net1(netrainX')';
        forecast(pppp).Y = net1(all_data(pppp).netrainX')';
        %size(all_data(pppp).netrainX)
        %if (nargin<2)
            mean = 0;
            if (data_type == 1) %for control_theta, mean is not zero
                mean = all_data(pppp).theta0;
            end
            for i=1:length(forecast(pppp).Y)
                sum = sum+(forecast(pppp).Y(i) - mean)^2;
            end
            sum_ppp = sum_ppp + ppp;
        %end
    end
    disp(string_title);
    disp('mse is');
    disp(sum/sum_ppp)

    
    %%%%%%% 3 b. Graphing the forecasts (results) %%%%%%%%

    indices = 1:1:length(data_file);
 if(data_type==2)
     indices = length(data_file):-1:1;
 end
    
%for pppp=1:length(data_file)
for pppp=indices
    nx=1;
    graphingX = all_data(pppp).X;
    graphingY = all_data(pppp).Y;
    forecastedData1 = zeros(mmm,nnn);
     %if (nargin<2)
        graphingZ1 = all_data(pppp).data; 
     %end
    if (cluster_data == 1 && data_type ~= 1)
        for px = 1:length(all_data(pppp).index)
            iii = all_data(pppp).index(px,1); jjj = all_data(pppp).index(px,2);
            if (forecast(pppp).Y(px,1)<0)
                forecastedData1(iii,jjj) = 0;
            else
                forecastedData1(iii,jjj) = forecast(pppp).Y(px,1);
            end
            %disp([graphingX(iii,jjj) graphingX(iii,jjj) graphingZ1(iii,jjj)/1000 data(iii,jjj)/1000]) 
        end
    else
        for px=1:mmm
            for qx=1:nnn
                    %graphingX(px,qx) = netrainX(nx,1);
                    %graphingY(px,qx) = netrainX(nx,2);
                    forecastedData1(px,qx) = forecast(pppp).Y(nx,1);
                    %graphingZ1(px,qx) = netrainY1(nx,1);
                    nx = nx + 1;
            end
        end
    end
    
    if (nargin==0)
        if (data_type==1)
            figure(1); 
        elseif (data_type==2)
            figure(2);
        else
            figure(3);
        end
        %fig_no = fig_no +1;
        subplot(2,1,2);
        contourf(graphingX,graphingY,forecastedData1,'ShowText','on'); hold on;
        %colormap(flipud(autumn));
        title([string_title,' predict'],'Fontsize',Fontsize);

        subplot(2,1,1);
        contourf(graphingX,graphingY,graphingZ1,'ShowText','on'); hold on;
        %colormap(flipud(autumn));
        title([string_title,' actual'],'Fontsize',Fontsize);
        xlabel('vx','Fontsize',Fontsize); ylabel('y','Fontsize',Fontsize);

        for iijj=1:2
            subplot(2,1,iijj)
            if(data_type==1)
                colormap('cool');
            elseif (data_type==2)
                colormap(flipud(autumn));
            else
                colormap(flipud(summer));
             end
        end
    end
    
    %%%%% needs to be rewritten (PAB47)
%     if (data_type==1)
%          control_fit_theta =  forecastedData1;
%     elseif (data_type == 2)
%         control_fit_Pc =  forecastedData1;
%     else
%         control_fit_Pt =  forecastedData1;
%     end
end

    if (data_type == 1)
        net_theta = net1;
    elseif (data_type == 2)
        net_Pc = net1;
    else
        net_Pt = net1;
    end
    clear net1
    clear netrainX netrainY1 forecast forecastedData1 index all_data
    
    rmfield(robot,{'zstar','control'});


end
