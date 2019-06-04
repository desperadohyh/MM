function recorded_data = record(recorded_data, Ax_current, Tx_current, g_current, dt)

recorded_data.theta_implement = [recorded_data.theta_implement [Ax_current; g_current ]];
recorded_data.MODE = [recorded_data.MODE mode];
end_effector = ee_plot(recorded_data.theta_implement, Tx_current(1:2),DH);
recorded_data.end_implement = [recorded_data.end_implement  end_effector];
recorded_data.traj_implement = [recorded_data.traj_implement Tx_current(1:2)'];
recorded_data.pla_implement = [recorded_data.pla_implement [0 0 0]'];
recorded_data.Dt = [Dt dt];

end