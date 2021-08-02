function h=generate_ellipse(x0,y0,P1,P2,phi,option)

if (nargin < 5)
    error('generate_ellipse needs more inputs');
elseif (nargin==5)
    option = 0;
end

c = cos(phi);
s = sin(phi);

P = [P1 0; 0 P2];

a = sqrt(1/P(1,1));
b = sqrt(1/P(2,2));

theta = linspace(0,2*pi);
x = x0 +  a*c*cos(theta) - b*s*sin(theta);
y = y0 +  a*s*cos(theta) + b*c*sin(theta);

if (option==0)
    h=plot(x,y,'r'); hold on
elseif (option==1)
    %h=patch(x,y,'cyan','Edgealpha',0.1,'Edgecolor','cyan','Facealpha',0.1);
    %154 104
    h=patch(x,y,[230 104 230]/255,'Edgealpha',0.2,'Edgecolor',[230 104 230]/255,'Facealpha',0.2);
else
    h=patch(x,y,[230 104 230]/255,'Edgealpha',0.5,'Edgecolor',[230 104 230]/255,'Facealpha',0.5);
end