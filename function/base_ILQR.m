function [X_out, u] = base_ILQR(states_out,trajectory,NILQR,z0_,var)

%% ILQR 
    var.N = NILQR;
    traj=[];
    if isempty(states_out)==false
        nstate = var.nstate;
        traj(1,:) = states_out(1:nstate:end)';
        traj(2,:) = states_out(2:nstate:end)';
        traj(3,:) = states_out(6:nstate:end)';
        traj(4,:) = states_out(5:nstate:end)';
        traj(5,:) = states_out(7:nstate:end)';
        Tx_current = [z0_(1:2); z0_(6); z0_(5); z0_(7)];
    end
    
    if isempty(trajectory)==false
        traj(1,:) = trajectory(1:2:end);
        traj(2,:) = trajectory(2:2:end);
        traj(3:5,:) = z0_(3:5)*ones(1,size(trajectory,1)/2);
        Tx_current = z0_;
    end
        
    
    ILQRtraj = traj(:,1:NILQR);
    xref_= ILQRtraj;    
    
    
    L1 = zeros(7,NILQR);
    
    L2 = diag([1000 1000 0 0 0 1 1 ]);
    L2=reshape(repmat(L2,1,NILQR),7,7,NILQR);
    l=zeros(1,NILQR);
    
    u0 = zeros(2,NILQR-1);
    

    %xref_
    [ X_out, u, xbar, ubar, kk, K, sigsu, A, B ] = MaxEntILQR( L2, L1, l, Tx_current, u0, var, xref_ );

    
end