%===================================================================
function animate_w_text(t_all,z_all,robot,fps,t_k_save,z_k_save, sim, z_source,z_target)
%%%% taken from snopt_agility (ACC paper)
%===================================================================

z_all = [z_all(:,1:4) z_all(:,9:end)]; %get rid of energy part

turn_on_ROA = 1;
% f=figure(9); hold on;
% movegui(f,'northwest');
% f=figure(10); hold on;
% movegui(f,'northeast');


if (robot.writeMovie)
    delete('animate_runner.avi');
    delete('animate_phase.avi');
    mov = VideoWriter('animate_runner.avi');
    %mov.Quality = 100%; 
    open(mov);
    if (turn_on_ROA)
        mov2 = VideoWriter('animate_phase.avi');
        open(mov2);
    end
end

%%% interpolate for animation %%
[m,n] = size(z_all);
tinterp = linspace(t_all(1),t_all(end),fps*(t_all(end)-t_all(1)));
for i=1:n
    z_interp(:,i) = interp1(t_all,z_all(:,i),tinterp);
end

[mm,nn] = size(z_interp);
min_xh = min(z_interp(:,1)); max_xh = max(z_interp(:,1)); 
dist_travelled = max_xh - min_xh;
camera_rate = dist_travelled/mm;

window_xmin = -5*robot.l; window_xmax = robot.l*5;
window_ymin = -0.1; window_ymax = 3*robot.l;

axis('equal')
axis([window_xmin window_xmax window_ymin window_ymax])
axis off
set(gcf,'Color',[1,1,1])

if (turn_on_ROA)
%%%%%%%%%%%%%%%%%%%%
figure(10); hold on
%movegui(f,'northeast');
%%%%%%% plot the level sets %%%%%%%%
    %colors = {'b','g','r','c','m','y'};
    %h = figure(9); hold on;
    for i=1:length(sim)
        %refine = 10;
        %%%%%%% save_X = level_set(P,phi, c,refine) %%%%%%%
        P = sim(i).P;
        P1 = P(1,1); P2 = P(2,2); phi = 0;
        generate_ellipse(sim(i).zstar(1),sim(i).zstar(2),P1,P2,phi,1);
        axis('equal');
        xlim([0 6.5]);
        ylim([0.5 2.5]);
        grid on
        %save_X = level_set(sim(i).P,0,robot.c,refine);   
        %patch(save_X(:,1)+sim(i).zstar(1),save_X(:,2)+sim(i).zstar(2),colors{i},...
        %          'FaceAlpha',0.4,'Edgecolor',colors{i}); 
        %axis('equal');
        %grid on
    end
    

    
    for i=1:length(sim)
        plot(sim(i).zstar(1),sim(i).zstar(2),'k+','Markersize',8,'MarkerEdgeColor','k','MarkerFaceColor','k'); 
    end
    plot(z_source(1), z_source(2),'kx','Markersize',10,'MarkerEdgeColor','k','MarkerFaceColor','k','LineWidth',3); 
end
    text_string1B = 'Start';
    text(z_source(1)-0.4,z_source(2)+0.1,text_string1B,'Fontsize',12);
    text_string1B = ', Step 0';

    htext = text(z_source(1)-0.1,z_source(2)+0.1,text_string1B,'Fontsize',12);
    
if(turn_on_ROA) 
    plot(z_target(1), z_target(2),'ko','Markersize',10,'MarkerEdgeColor','k','MarkerFaceColor','k'); 
    text_string1B = 'Goal';
    text(z_target(1)+0.2,z_target(2),text_string1B,'Fontsize',12);
    
    %axis('equal');
    %grid on
    xlabel('Apex velocity (m/s)','Fontsize',15);
    ylabel('Apex height (m)','Fontsize',15);
    set(gca,'fontsize',12)
end
%%%%%%%%%%%%%%%%%%%%%%%
counter = 1;

for i=1:length(tinterp)
   
    figure(9);
    %movegui(f,'northwest');
    %UTSA Blue: RGB values 12,35,64 and UTSA Orange: RGB values 241, 90, 34
    plot(z_interp(i,1),z_interp(i,3),'ro','MarkerEdgeColor',[241, 90, 34]/255, 'MarkerFaceColor',[241, 90, 34]/255,'MarkerSize',20); %com
    line([-2 max(z_interp(:,1))+10],[0 0],'Linewidth',2,'Color','black'); %ground
    line([z_interp(i,1) z_interp(i,5)],[z_interp(i,3) z_interp(i,6)],'Linewidth',4,'Color',[12,35,64]/255); %leg
    if (i==1)
        figure(10);
        P = sim(1).P;
        P1 = P(1,1); P2 = P(2,2); phi = 0;
        h1=generate_ellipse(sim(1).zstar(1),sim(1).zstar(2),P1,P2,phi,2);
    end
    figure(9)
    if (tinterp(i)>=t_k_save(counter+1))
        delete(h1);
%            if (i>2)
%                for ii=1:length(hh)
%                    delete(hh);
%                end
%            end
        %if (turn_on_ROA)
            delete(htext);
            figure(10)
            counter = counter + 1;
            if (turn_on_ROA)
                plot(z_k_save(counter,1),z_k_save(counter,2),'ro','Markersize',10,'MarkerEdgeColor','r','MarkerFaceColor','w')
                text_string1B = ['Step ',num2str(counter-1)];
                htext = text(z_k_save(counter,1)-0.05,z_k_save(counter,2)+0.25,text_string1B,'Fontsize',12);
                %for ii=1:length(sim)
                    %refine = 10;
                    %%%%%%% save_X = level_set(P,phi, c,refine) %%%%%%%
%                     P = sim(ii).P;
%                     P1 = P(1,1); P2 = P(2,2); phi = 0;
%                     if (ii==counter-1)
%                         hh(ii)=generate_ellipse(sim(counter-1).zstar(1),sim(counter-1).zstar(2),P1,P2,phi,1);
%                     else 
%                         hh(ii)=generate_ellipse(sim(ii).zstar(1),sim(ii).zstar(2),P1,P2,phi,2);
%                     end
%                     axis('equal');
%                     xlim([1.5 6.5]);
%                     grid on
          
%                end

                if (i<length(tinterp))
                P = sim(counter-1).P;
                P1 = P(1,1); P2 = P(2,2); phi = 0;
                    h1=generate_ellipse(sim(counter).zstar(1),sim(counter).zstar(2),P1,P2,phi,2);
                end
            end
            
        %end
        
        figure(9)
        
        %%%%%% test code %%%%%
%         if (counter == 5)
%             if (robot.writeMovie)
%                 close(mov);
%                 close(mov2);
%             end
%             error(' ');
%         end
    end 
    xx = mean([window_xmin window_xmax])-1.0;
    yy = window_ymax;
    text_string1 = ['Step No = ',num2str(counter-1)];
    text_string2 = ['Apex Velocity= ', num2str(round(z_k_save(counter,1),2))]; 
    text_string3 = ['Apex Height = ', num2str(round(z_k_save(counter,2),2))];
    displace = -1.;
    text(xx,yy+0.9+displace,text_string1,'Fontsize',15);
    text(xx,yy+0.5+displace,text_string2,'Fontsize',15);
    text(xx,yy+0.1+displace,text_string3,'Fontsize',15);
    
    window_xmin = window_xmin + camera_rate;
    window_xmax = window_xmax + camera_rate;
    axis('equal')
    axis off
    axis([window_xmin window_xmax window_ymin window_ymax])

    figure(9)
  if (robot.writeMovie)
        %axis off %does not show axis
        set(gcf,'Color',[1,1,1]) %set background to white
        frame = getframe(gcf);
        %writeVideo(mov,getframe);
        writeVideo(mov,frame);
        
  end
    pause(0.05);
    
if (turn_on_ROA)   
   figure(10)
   if (robot.writeMovie)
        %axis off %does not show axis
        %set(gcf,'Color',[1,1,1]) %set background to white
        %hh = get(gcf);
        %position = hh.Position;
        %getframe(gcf,position); %position = [left bottom width height])
        %writeVideo(mov2,getframe);
        frame = getframe(gcf);
        writeVideo(mov2,frame);
  end
    pause(0.05);
    %for i=1:length(sim)
    %    plot(z_source(1), z_source(2),'ro','Markersize',10,'MarkerEdgeColor','r','MarkerFaceColor','r'); 
    %    plot(z_k_save(:,1),z_k_save(:,2),'ro','Markersize',10,'MarkerEdgeColor','r','MarkerFaceColor','r'); 
    %end
end
end

if (robot.writeMovie)
    close(mov);
    if(turn_on_ROA)
    close(mov2);
    end
end
