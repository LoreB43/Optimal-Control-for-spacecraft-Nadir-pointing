function q = euler2quaternion(eul)
    % eul2quat converts Euler angles to quaternion representation
    % Assumes the Euler angles are in the ZYX sequence (yaw-pitch-roll)
    % eul is a vector of three Euler angles [alpha, beta, gamma]

    % Extract individual Euler angles
    alpha = eul(1); % rotation around z-axis (yaw)
    beta = eul(2);  % rotation around y-axis (pitch)
    gamma = eul(3); % rotation around x-axis (roll)

    % Compute quaternion
    cy = cos(alpha * 0.5);
    sy = sin(alpha * 0.5);
    cp = cos(beta * 0.5);
    sp = sin(beta * 0.5);
    cr = cos(gamma * 0.5);
    sr = sin(gamma * 0.5);

    q = zeros(1, 4); % Preallocate quaternion
    q(4) = cr * cp * cy + sr * sp * sy; % w
    q(1) = sr * cp * cy - cr * sp * sy; % x
    q(2) = cr * sp * cy + sr * cp * sy; % y
    q(3) = cr * cp * sy - sr * sp * cy; % z

    q = q / vecnorm(q);
end

