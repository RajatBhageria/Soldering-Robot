function [q] = getQ(x,y,z)

L1 = 3*25.4;          %base height (in mm)
L2 = 5.75*25.4;       %shoulder to elbow length (in mm)
L3 = 7.375*25.4;      %elbow to wrist length (in mm)
L4 = 1.75*25.4;       %Wrist1 to Wrist2 (in mm)
L5 = 1.25*25.4;       %wrist2 to base of gripper (in mm)
L6 = 100; 
zdist = L6;    %Distance from Wrist Center to Gripper Center
PI = pi();            %Create PI constant
xyDist = L4+L5; 

%angle away the center
th1 = 

%Calculates the wrist center location
xw = x - zdist - xyDist * cos();
yw = y - zdist - xyDist * sin();
zw = y - zdist; 
%disp ([xw yw, zw]);

%Calculates the theta 1 needed
th1 = atan2(yw,xw);

end

