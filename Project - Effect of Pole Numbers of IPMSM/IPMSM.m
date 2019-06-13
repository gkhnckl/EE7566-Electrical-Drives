% this script calculates torque for ipmsm

Imax = 400;  %A, maximum current
Load_angle = 45;  %deg, load angle

iq = Imax*cosd(Load_angle);  %A, iq current
id = -Imax*sind(Load_angle);  %A, id current


%% motor specific paremeters
Vdc = 500;
pp = 5;
mean_L_AB = -348e-6 ;  %H, mean of inductance L_AB
pp_L_AB = 678e-6 ;  %H, peak to peak inductance of L_AB

fi = 0.125;  %Wb, peak flux linkage of a phase due to PMs when maxium current is zero

%%

M = mean_L_AB; %H, mutual inductance
L = M * (-2);   %H, self inductance
M2 = pp_L_AB / (-2);  %H, position dependent inductance

Ld = 1.5 * (L+M2);  %H, Ld inductance
Lq = 1.5 * (L-M2);  %H, Lq inductance

Ld = 0.45e-3;
Lq = 1.23e-3; 


Torque = 1.5*pp*(fi-(Lq-Ld)*id)*iq;


SpeedEl = Vdc / sqrt(3) / sqrt((Lq*iq)^2 + (Ld*id + fi)^2) 
SpeedMec = SpeedEl / pp
