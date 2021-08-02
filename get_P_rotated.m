function P_rotated = get_P_rotated(P,phi)

a = sqrt(1/P(1,1)); %major axis
b = sqrt(1/P(2,2)); %minor axis
p11 = (cos(phi)^2)/(a^2) + (sin(phi)^2)/(b^2);
p22 = (cos(phi)^2)/(b^2) + (sin(phi)^2)/(a^2);
p12 = (-(1/b^2) + (1/a^2))*cos(phi)*sin(phi);
P_rotated = [p11 p12; p12 p22];