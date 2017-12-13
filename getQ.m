function [q] = getQ(x,y,z)

L1 = 3*25.4;          %base height (in mm)
L2 = 5.75*25.4;       %shoulder to elbow length (in mm)
L3 = 7.375*25.4;      %elbow to wrist length (in mm)
L4 = 1.75*25.4;       %Wrist1 to Wrist2 (in mm)
L5 = 1.25*25.4;       %wrist2 to base of gripper (in mm)
L6 = 0;             %Distance from Wrist Center to Gripper Center

zdist = L6;        
xyDist = L4+L5; 

%angle away the center
th1 = atan2(y,x); 

%Calculates the wrist center location
xw = x - xyDist * cos(th1);
yw = y - xyDist * sin(th1);
zw = z + zdist; 

%Parameters needed for calculating theta 2 and theta 3
r = sqrt(xw^2 + yw^2);
s = abs(zw - L1);
D = (r^2 + s^2 - L2^2 - L3^2)/(-2*L2*L3);

%Calculates theta 3
th3 = atan2(D,sqrt(1-D^2));

%Uses theta 3 to calculate theta 2
th2 = atan2(r,s) - atan2(L3*cos(th3),L2-L3*sin(th3));

%Calculates theta 4
zdist = z - zw;
xydist = sqrt((x - xw)^2 + (y - yw)^2);
th4 = -atan2(zdist, xydist) - th2 - th3;

q = [th1, th2, th3, th4, 0, 0];

end

