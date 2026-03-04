%% Orbit and Environment parameter definition
TE      = 23.9344696 * 3600;      % Earth's sidereal day
ibody   = 3;                      % Earth
mjd2000 = 200;                    % Day of starting
J_e     = astroConstants(9);      % J2 effect    [-]
R_e     = astroConstants(23);     % Radius Earth [km]
mu_e    = astroConstants(13);     % gravitational parameter of Earth [km^3/s^2]

% Earth parameters
w_earth = 2*pi / TE;                                 % Earth angular velocity [rad/s]
epsilon = deg2rad(23.45);                            % Earth orbit inclination [rad]
rho_s   = [0.5 0.5 0.5 0.5 0.5 0.5 0.1 0.1 0.1 0.1]; % Specular reflection coefficients [-]
rho_d   = ones(1,10)*0.1;                            % Diffusion coefficients [-]
c       = astroConstants(5)*(10^3);                  % Speed of light [m/s]
G       = 6.67e-20;                                  % Gravitational constant [Nm^2/kg^2]
mt      = 5.9722e24;                                 % Earth mass [kg]
Fe_sun  = 1358;                                      % Direct radiation of Sun [W/m^2]


%% Kinematics parameters
kep_SSRGT(1) = 90000;                                % Orbit semi-major axis [km]                           
n            = sqrt( (mu_e)/ kep_SSRGT(1)^3 );       % Orbital angular velocity [rad/s]

w_0          = [0 0 n]';        % Initial spacecraft angular velocity [rad/s]
w_desired    = [0 0 n]';        % Desired angular velocity [rad/s]
q0           = randn(4,1);      % Initial random attitude quaternion
q0           = q0/norm(q0);     % Quaternion normalization

w_LN = [0 0 n];                 % angular velocity desired in LVLH frame [rad/s]

%% geometry parameters

[Satellite.I, Satellite.A, Satellite.N, Satellite.R] = V2_Geometry_Satellite;

% Inertia moments of spacecraft [kg m^2]
Ix = Satellite.I(1,1);
Iy = Satellite.I(2,2);
Iz = Satellite.I(3,3);


%% Orbit acquisition

%state vector of the earth wrt sun acquisition from uplanet
[kep, mu_sun]  = uplanet(mjd2000, ibody); 
[r_0ts, v_0ts] = kep2car(kep(1),kep(2),kep(3),kep(4),kep(5),kep(6),mu_sun);

%state vector of spacecraft wrt earth throught keplerian coordinates 
[x0, v0]       = kep2car(kep_SSRGT(1),kep_SSRGT(2),kep_SSRGT(3),kep_SSRGT(4),kep_SSRGT(5),kep_SSRGT(6),mu_e);
