function save_X = level_set(P,phi, c,refine)

if (nargin<4)
    refine = 1;
end


% if (nargin==0) %testing 
%     %%% test set
%     clc
% 	clear all
% 	close all
%     refine = 1;% if (nargin==0) %testing 
%     %%% test set
%     clc
% 	clear all
% 	close all
%     refine = 1;
% 
%     zstar = [5 1.4]; %v, h
%     robot = parms;
%     TE = 0.5*robot.m*zstar(1)*zstar(1) + robot.m*robot.g*zstar(2);
%     [x,F,INFO] = lyapmain(zstar);
%     p11 = x(1);
%     p12 = 0;
%     p22 = x(2);
%     P = [p11 p12; p12 p22];
%     c = 1; %find level set X'*P*X - c = 0
% end

% 
%     zstar = [5 1.4]; %v, h
%     robot = parms;
%     TE = 0.5*robot.m*zstar(1)*zstar(1) + robot.m*robot.g*zstar(2);
%     [x,F,INFO] = lyapmain(zstar);
%     p11 = x(1);
%     p12 = 0;
%     p22 = x(2);
%     P = [p11 p12; p12 p22];
%     c = 1; %find level set X'*P*X - c = 0
% end


if (det(P)< 0)
    error('P is not positive definite');
end
% semi_major = norm(max(eig(inv(P))))
% semi_minor = norm(min(eig(inv(P))))
% area = pi*semi_major*semi_minor
% bound = sqrt(c)*semi_major

if (P(1,2)~=0)
    error('Formula for ellipse does not apply to non-diagonal P matrix');
end
a = sqrt(c/P(1,1));
b = sqrt(c/P(2,2));
%area = pi*a*b

%x = linspace(-a,a,round(a)*10*refine+1);

% if (nargin==0)
%     for i=1:length(x)
%         xxx(i,1) = x(i)+zstar(1);
%         x2= xxx(i);
%         yyy(i,1) = (TE - 0.5*robot.m*x2*x2)/(robot.m*robot.g);
%     end
% end

%save_X = [];
theta = linspace(0,2*pi,10*refine+1);
%theta = theta(1:end-1); %ignore last point (repeat of 0)
cc = cos(phi);
ss = sin(phi);
x = a*cc*cos(theta) - b*ss*sin(theta);
y = a*ss*cos(theta) + b*cc*sin(theta);

%x = zstar(1)  +  a*c*cos(theta) - b*s*sin(theta);
%y = zstar(2)  +  a*s*cos(theta) + b*c*sin(theta);

% x = a*cos(theta);
% y = b*sin(theta);
save_X = [x', y'];
% for j=1:2
%     for i=1:length(x)
%      if (i==1)
%             y0 = 0;
%     end
%         %[y,fval,exitflag] = fsolve(@curve,y0,options,x(i),P,c);
%         if (j==1) %positive y solution
%             lb = 0;
%             ub = [];
%         else %negative y solution
%             ub = 0;
%             lb = [];
%         end
%         %X = fmincon(FUN,X0,A,B,Aeq,Beq,LB,UB,NONLCON,OPTIONS) 
%         if (j==1)
%             xx = x(i);
%             [y,fval,exitflag] = fmincon(@dummy,y0,[],[],[],[],lb,ub,@curve,options,xx,P,c);
%         else
%             xx = x(length(x)-i+1);
%             [y,fval,exitflag] = fmincon(@dummy,y0,[],[],[],[],lb,ub,@curve,options,xx,P,c);
%         end
%         if (exitflag==1)
%             save_X = [save_X; xx y];
%         end
%         y0 = y;
%     end
% end
% 
% if (nargin==0)
%     %save_X
%     %[2*length(x) length(save_X)]
%     figure(1)
%     patch(save_X(:,1)+zstar(1),save_X(:,2)+zstar(2),'ro-'); hold on;
%     plot(xxx,yyy,'k','Linewidth',2);
%     %[xx' yy']
%     axis('equal');   
%     grid on
%     figure(2)
%     plot(save_X(:,1)+0*zstar(1),save_X(:,2)+0*zstar(2),'ko-'); hold on
%     %plot(x,yy,'k','Linewidth',2);
%     axis('equal');
%     grid on
%     figure(1)
% end


% function C = dummy(y,x,P,c)
% C = 1;
% 
% function [cin,ceq] = curve(y,x,P,c)
% X= [x y]';
% ceq = X'*P*X - c;
% cin = [];
