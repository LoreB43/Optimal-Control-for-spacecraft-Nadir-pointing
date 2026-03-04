function[a,e,inc,omega,w,theta,h]=car2kep(r,v,mu)
% car2kep - Converts Cartesian position and velocity vectors to Keplerian orbital elements.
%
% This function takes the position vector (r) and velocity vector (v) of an orbiting 
% object in Cartesian coordinates and computes the corresponding Keplerian orbital elements. 
% The function also uses the gravitational parameter (mu) to make the necessary calculations 
% for the semi-major axis, eccentricity, inclination, longitude of ascending node, argument of 
% periapsis, and true anomaly.
%
% Input:
%   r     - 3x1 Cartesian position vector [x; y; z] (in meters).
%   v     - 3x1 Cartesian velocity vector [vx; vy; vz] (in meters per second).
%   mu    - Gravitational parameter (in m^3/s^2).
%
% Output:
%   a     - Semi-major axis of the orbit (in meters).
%   e     - Eccentricity vector (dimensionless).
%   inc   - Inclination of the orbit (in radians).
%   omega - Longitude of the ascending node (in radians).
%   w     - Argument of periapsis (in radians).
%   theta - True anomaly (in radians).
%   h     - Specific angular momentum vector (in kg*m^2/s).
%

    % Calculate the magnitudes of the position (r) and velocity (v) vectors
    mod_r=norm(r);
    mod_v=norm(v);
    
    % Specific angular momentum vector (h)
    h=cross(r,v);
    mod_h=norm(h);
    
    % Inclination (inc) of the orbit
    inc=acos(h(3)/mod_h);
    
    % Specific orbital energy (E) and semi-major axis (a) calculation 
    E=0.5*mod_v^2-mu/mod_r;
    a=-0.5*mu/E;
    
    % Calculating the node vector (N)
    if inc==0
        N=[1 0 0]';
        mod_N=1;
    else
    K=[0 0 1]';
    N=cross(K,h);
    mod_N=norm(N);
    end

    % Eccentricity vector (e)
    e=(1/mu)*(((mod_v^2-mu/mod_r)*r)-(dot(r,v)*v));
    mod_e=norm(e);

    % If eccentricity is very small (near circular orbit), set e to the node vector
    if mod_e <= 1e-3
        e=N;
    end
    
    % Longitude of ascending node (omega)
    if inc==0
        omega=0;
    elseif(N(2)>=0)
     omega=acos(N(1)/mod_N);
    else
     omega=2*pi-acos(N(1)/mod_N);   
    end
    
    % Argument of periapsis (w)
    if mod_e<=1e-3
        w=0;
    elseif(e(3)>=0)
        w=acos(dot(N,e)/(mod_e*mod_N));
    else
        w=2*pi-acos(dot(N,e)/(mod_e*mod_N));
    end
    
    % Radial velocity (v_r) - component of velocity along the radial direction
    v_r=dot(r,v)/mod_r;
    
     % True anomaly (theta) calculation
    if(v_r>=0)
     theta=acos(dot(e,r)/(mod_e*mod_r));
    else
     theta=2*pi-acos(dot(e,r)/(mod_e*mod_r));  
    end
    
    % Handle cases for circular orbits where theta can be problematic (i.e., should be between 0 and 2*pi)

end
























