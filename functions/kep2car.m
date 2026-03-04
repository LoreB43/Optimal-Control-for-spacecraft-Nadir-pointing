function [r,v]=kep2car(a,e,i,omega,w,theta,mu)
% kep2car - Converts Keplerian orbital elements to Cartesian position and velocity vectors.
%
% This function takes the six classical Keplerian orbital elements (semi-major axis,
% eccentricity, inclination, longitude of ascending node, argument of periapsis, 
% and true anomaly) along with the gravitational parameter, and computes the 
% corresponding Cartesian position and velocity vectors in a 3D space.
%
% Input:
%   a      - Semi-major axis of the orbit (in meters).
%   e      - Eccentricity of the orbit (dimensionless).
%   i      - Inclination of the orbit (in radians).
%   omega  - Longitude of the ascending node (in radians).
%   w      - Argument of periapsis (in radians).
%   theta  - True anomaly (in radians).
%   mu     - Gravitational parameter (in m^3/s^2).
%
% Output:
%   r      - 3x1 Cartesian position vector [x; y; z] (in meters).
%   v      - 3x1 Cartesian velocity vector [vx; vy; vz] (in meters per second).
%
% The function first calculates the orbital parameters such as the radial distance and
% velocity in the perifocal frame. Then, it applies the appropriate rotation matrices 
% to transform the position and velocity vectors from the perifocal frame to the 
% reference frame defined by the orbital elements. The output vectors are in the 
% Cartesian coordinate system corresponding to the orbital frame.

 % Calculate the modulus of the eccentricity vector (e)
 mod_e=norm(e);

 % Semi-latus rectum (p) calculation
 p=a*(1-mod_e^2);

 % Radial distance (r) calculation using the true anomaly (theta)
 r=p/(1+mod_e*cos(theta));

 % Position vector in the perifocal frame (r_pf)
 r_pf=r*[cos(theta);sin(theta);0];

 % Velocity vector in the perifocal frame (v_pf)
 v_pf=sqrt(mu/p)*[-sin(theta);mod_e+cos(theta);0];

 % Rotation matrix for the longitude of the ascending node (omega)
 R_omega=[cos(omega) sin(omega) 0;
          -sin(omega) cos(omega) 0;
          0 0 1];

 % Rotation matrix for the inclination (i)
 R_i=[1 0 0;
     0  cos(i) sin(i);
     0 -sin(i) cos(i)];

 % Combined rotation matrix (R) for transforming vectors from perifocal frame to inertial frame
 R_w=[cos(w) sin(w) 0;
      -sin(w) cos(w) 0;
          0 0 1];
 R=R_w*R_i*R_omega;

  % Rotate the position vector (r_pf) and the velocity vector (v_pf) to the inertial frame
 r=R'*r_pf;
 v=R'*v_pf;
end


