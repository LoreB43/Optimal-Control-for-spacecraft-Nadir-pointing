function cost = cost_function(PID,model)
% cost_function - Computes the cost for the given PID parameters based on simulation results.
%
% This function evaluates the cost of a set of PID controller parameters by simulating 
% the control system model. It calculates the cost based on the pointing error, angle 
% error, angular velocity error, and the control torque. The cost is penalized if the 
% torque exceeds specified limits, encouraging the optimization of PID parameters that 
% minimize the errors and keep the control torque within limits.
%
% Input:
%   PID   - A vector containing the PID controller parameters [kp, kd, ki].
%   model - The name or path of the Simulink model to simulate.
%
% Output:
%   cost  - A scalar value representing the total cost, incorporating the errors 
%           and penalties associated with the torque control.
%
% The function assigns the PID values to the workspace, runs the Simulink model, 
% and retrieves the simulation outputs, which include pointing error, angle error, 
% angular velocity error, and control torque. The cost is computed by summing up 
% squared error terms and adding a penalty for any violations of torque constraints.

    
    assignin('base', 'kp', PID(1));
    assignin('base', 'kd', PID(2));
    assignin('base', 'ki', PID(3));
    
    % Run the Simulink model
    simOut = sim(model);

    pointing_error = simOut.get('q_error');
    assignin('base','pointing_error',pointing_error);

    angle_error = simOut.get('angle_error');
    assignin('base','angle_error',angle_error);

    omega_error = simOut.get('w_error');
    assignin('base','omega_error',omega_error);

    torque_control = simOut.get('torque_control');
    assignin('base','torque_control',torque_control);
    
    cost = 0;
    penalty_factor = 1e2;  % Adjust the penalty factor if necessary
    
    for i = 1:size(pointing_error, 1)
        qe_vec = pointing_error(i, 1:3);  % Quaternion error vector
        we_vec = omega_error(i, :);  % Angular velocity error vector
    
        % Compute the squared norms for each error component
        qe_norm_sq = norm(qe_vec)^(2);  % Proportional error
        we_norm_sq = norm(we_vec)^(2) ; % Derivative error
    
        % Torque control input
        torque_control_vec = torque_control(i, :);
    
        % Compute cost terms
        cost = cost + 1e8*qe_norm_sq + we_norm_sq*1e4 ;
        
        % Add penalty for torque constraint violations
        penalty_torque = penalty_factor * norm(torque_control_vec)^(2);
        cost = cost + penalty_torque;
    end

    
end
