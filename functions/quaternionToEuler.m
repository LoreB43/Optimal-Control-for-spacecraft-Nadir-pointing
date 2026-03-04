function [roll, pitch, yaw] = quaternionToEuler(q)
% quaternionToEuler - Converts a quaternion to Euler angles (roll, pitch, yaw).
%
% This function takes a quaternion representation of orientation and converts it to
% the corresponding Euler angles (roll, pitch, yaw). The quaternion is expected to
% have the format [q_x, q_y, q_z, q_w], where q_w is the scalar component, and 
% q_x, q_y, q_z are the vector components.
%
% Input:
%   q - A 4-element vector representing the quaternion [q_x, q_y, q_z, q_w].
%        The quaternion is assumed to be in the format [x, y, z, w] with 'w' being 
%        the scalar part.
%
% Output:
%   roll  - The roll angle (rotation about the x-axis) in radians.
%   pitch - The pitch angle (rotation about the y-axis) in radians.
%   yaw   - The yaw angle (rotation about the z-axis) in radians.
%
% The function calculates the Euler angles based on standard formulas for converting
% a quaternion to roll, pitch, and yaw using the quaternion's components.
% To avoid numerical issues, the pitch is clamped to the range [-1, 1] before applying
% the arcsine function.


    % Input: q = [q_x, q_y, q_z, q_w] (quaternion with scalar as last component)
    q_x = q(1);
    q_y = q(2);
    q_z = q(3);
    q_w = q(4);  % Scalar part

    % Roll (phi)
    roll = atan2(2 * (q_w * q_x + q_y * q_z), 1 - 2 * (q_x^2 + q_y^2));
    
    % Pitch (theta)
    sin_theta = 2 * (q_w * q_y - q_z * q_x);
    sin_theta = max(-1, min(1, sin_theta)); % Clamp to [-1, 1] to avoid numerical issues
    pitch = asin(sin_theta);
    
    % Yaw (psi)
    yaw = atan2(2 * (q_w * q_z + q_x * q_y), 1 - 2 * (q_y^2 + q_z^2));
end
