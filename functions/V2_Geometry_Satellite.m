function [I,A,N,R]=V2_Geometry_Satellite
% This function calculates the geometry-related properties of a satellite.
% It returns the inertia matrix (I), area matrix (A), normal vectors (N), and relative positions (R).


 V_pannelli_solari=0.010*0.200*0.300;
 V_parallelepipedo = 0.200*0.200*0.300;
 V_tot=V_parallelepipedo+2*V_pannelli_solari;

 m_tot=12*1.33;

 rho=m_tot/V_tot;
 m_parallelepipedo=rho*V_parallelepipedo;  
 m_pannelli_solari=rho*V_pannelli_solari;
 
 Lz=0.300;
 Ly=0.200;
 Lx=0.200;
 
 I_x_parallelepipedo=m_parallelepipedo*(Ly^2+Lz^2)/12;
 I_y_parallelepipedo=m_parallelepipedo*(Lx^2+Lz^2)/12;
 I_z_parallelepipedo=m_parallelepipedo*(Ly^2+Lx^2)/12;

 Lz=0.01;

 I_x_pannelli_solari=m_pannelli_solari*(Ly^2+Lz^2)/12;
 I_y_pannelli_solari=m_pannelli_solari*(Lx^2+Lz^2)/12;
 I_z_pannelli_solari=m_pannelli_solari*(Ly^2+Lx^2)/12;

 x_G_pannello_solare_1=[0,0.200,0.1450]; 
 x_G_pannello_solare_2=[0,-0.200,0.1450];
 x_G_parallelepipedo=[0, 0 ,0 ];

 x_G_satellite = (m_parallelepipedo*x_G_parallelepipedo + m_pannelli_solari*x_G_pannello_solare_1+m_pannelli_solari*x_G_pannello_solare_2)/m_tot;

 x_G_parallelepipedo = x_G_parallelepipedo - x_G_satellite;
 x_G_pannello_solare_1 = x_G_pannello_solare_1 - x_G_satellite;
 x_G_pannello_solare_2 = x_G_pannello_solare_2 - x_G_satellite;

 I_x_satellite=I_x_pannelli_solari*2 + I_x_parallelepipedo +m_pannelli_solari*(x_G_pannello_solare_1(2)^2+x_G_pannello_solare_1(3)^2)+m_pannelli_solari*(x_G_pannello_solare_2(2)^2+x_G_pannello_solare_2(3)^2);
 I_y_satellite=I_y_pannelli_solari*2 + I_y_parallelepipedo +m_pannelli_solari*(x_G_pannello_solare_1(1)^2+x_G_pannello_solare_1(3)^2)+m_pannelli_solari*(x_G_pannello_solare_2(1)^2+x_G_pannello_solare_2(3)^2);
 I_z_satellite=I_z_pannelli_solari*2 + I_z_parallelepipedo +m_pannelli_solari*(x_G_pannello_solare_1(1)^2+x_G_pannello_solare_1(2)^2)+m_pannelli_solari*(x_G_pannello_solare_2(1)^2+x_G_pannello_solare_2(2)^2);

 I=[I_x_satellite 0 0
    0 I_y_satellite 0
    0 0 I_z_satellite];

 R_x_positiva=[0.100 0 0]-x_G_satellite;
 R_x_negativa=[-0.100 0 0]-x_G_satellite;
 R_y_positiva=[0 -0.100 0]-x_G_satellite;
 R_y_negativa=[0 +0.100 0]-x_G_satellite;
 R_z_positiva=[0 0 0.150]-x_G_satellite;
 R_z_negativa=[0 0 -0.150]-x_G_satellite;

 R_pannello_solare_1 =  x_G_pannello_solare_1;
 R_pannello_solare_2 =  x_G_pannello_solare_2;
 
R=[ R_x_positiva
 R_x_negativa
 R_y_positiva
 R_y_negativa
 R_z_positiva
 R_z_negativa
 R_pannello_solare_1
 R_pannello_solare_1
 R_pannello_solare_2
 R_pannello_solare_2];

 A_z = 0.200*0.200;
 A_y= 0.200*0.300;
 A_x = 0.200*0.300;

 A_pannello_solare = 0.200*0.300;

 A=[A_x; A_x; A_y; A_y; A_z; A_z; A_pannello_solare; A_pannello_solare; A_pannello_solare;A_pannello_solare ];


 N_x_positiva = [1 0 0];
 N_x_negativa = [-1 0 0];
 N_y_positiva = [0 1 0];
 N_y_negativa = [0 -1 0];
 N_z_positiva = [0 0 1];
 N_z_negativa = [0 0 -1];

 N_pannello_solare_1_z_pos = [ 0 0 1];
 N_pannello_solare_2_z_pos = [ 0 0 1];
 N_pannello_solare_1_z_neg = [ 0 0 -1];
 N_pannello_solare_2_z_neg = [ 0 0 -1];

 N=[ N_x_positiva ;  N_x_negativa; N_y_positiva ;...
     N_y_negativa; N_z_positiva ;  N_z_negativa; ...
     N_pannello_solare_1_z_pos ;  N_pannello_solare_1_z_neg; N_pannello_solare_2_z_pos; ...
     N_pannello_solare_2_z_neg]';

end
