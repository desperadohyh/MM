%% check grasping

if norm([Ax_current;0.05]-(target+t_marg(:,mode)))<0.02
    %%% Platform stop%%
    
    if mode == 1   % Ready to grasp and lift
    %g_current = 
        grip_ang = [linspace(g_current,g_open(5),10)]';
        ii = 0;
        %%%%% Subscribe gripper angle%%%
        v_grip = 0.2;
        
        while v_grip > 0.1 && ii<11
            grip_pub = grip_ang(ii);
            ii = ii+1;
            pause(0.1);
        end
        
        grasp = 1;     
    
        zB = [0;Ax_current(2);Ax_current(3)/2;Ax_current(4);Ax_current(5)];
        xref_lift = [linspace(Ax_current(1),zB(1),10);
                    linspace(Ax_current(2),zB(2),10);
                    linspace(Ax_current(3),zB(3),10);
                    linspace(Ax_current(4),zB(4),10);
                    linspace(Ax_current(5),zB(5),10);
                     g_current*ones(1,10)];

         for j = 1:10
             pub = xref_lift(:,j);
             

             if  fail == 1%sensor>threshold
                 % FAIL: Play back and Open griper 
                 for b = j-1:-1:1
                     pub = xref_lift(:,j);                     
                 end
                 grasp = 0;
    %              for j = 1:10
    %                  pub = xref_lift(:,j);
    %              end
                 mode = 2;  % need to restart
%            fail = 0;%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

             %%%%%%% lazy mode 2%%%%%%%%%%%%%%%%%%%%%%
             
             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                 break
             end
             mode = 3;  % Ready to move the object       
         end
         
    elseif mode == 3  % Ready to put down
        for b = 10-1:-1:1
            pub = xref_lift(:,j);            
        end
        grasp = 0;
        mode = 1;  % Ready for new task
    
     
    end
    
    
end