function [] = testRobot(pinLetters, pinNumbers)
%pinLettersNumbers is a nx2 matrix where n is the number of pins to solder
%on the perfboard. The first column is the pin Letter to solder to 
%the second column is the pin Number to solder to. 

%how many different pins to solder 
[n,~] = size(pinLetters);

%xyz distance fo the corner of the perfboard closest to the robot from the
%base of the robot
xPerfToBase = 175;%mm
yPerfToBase = 0;%mm
zPerfToBase = 128;%mm

%distance from the left corner to the closest pin
xCornerToPin = 5;%mm
yCornerToPin = 9.23;%mm

%distance between the centers of respective pins 
distBtwPins = 2; 

alphabet = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ';

pause on; 

lynxServo([0,0,0,0,0,0]);
pause(1); 

% define the size of the square perfboard on which to solder
s = 48;%mm

% draw the perfboard
% define the points for the perfboard 
p5 = [xPerfToBase,-s/2+yPerfToBase, zPerfToBase];
p6 = [xPerfToBase,s/2+yPerfToBase, zPerfToBase];
p7 = [s+xPerfToBase,s/2+yPerfToBase, zPerfToBase];
p8 = [s+xPerfToBase,-s/2+yPerfToBase, zPerfToBase];

% draw the perfboard on the simulation 
xFloor = [p5(1) p6(1) p7(1) p8(1)];
yFloor = [p5(2) p6(2) p7(2) p8(2)];
zFloor = [p5(3) p6(3) p7(3) p8(3)];
fill3(xFloor, yFloor, zFloor, 'y');
hold on; 

for solder = 1:n
    %get the pin for the current solder 
    pinLetter = pinLetters(solder,1); 
    pinNumber = pinNumbers(solder,1);
    
    disp(pinLetter);
    disp(pinNumber); 
    
    %convert the pinLetter to a number 
    alphaNumber = strfind(alphabet,pinLetter); 
    
    %convert the pinNumber along y to be positive or negative 
    pinNumber = pinNumber - 10;
    
    %get the goal xyz position to actuate the tip of the robot to
    xGoal = alphaNumber * distBtwPins + xCornerToPin + xPerfToBase; 
    yGoal = pinNumber * distBtwPins + yCornerToPin + yPerfToBase;
    zGoal = zPerfToBase;

    %draw the goal on the perfboard 
    scatter3(xGoal,yGoal,zGoal,100,'r','filled');

    %find the goal orientation of the end effector 
    goalOrientation = [ 0   0   1;
                        0   1   0;
                       -1   0   0];
                   
    %find qEnd using IK using the goal EE position and also the goal orientation 
    T = [[goalOrientation;[0,0,0]],[xGoal;yGoal;zGoal;1]];
    [qEnd,~] = IK_lynx_sol(T); 

    %actuate to the end position  
    lynxServo(qEnd); 

    %wait a little to solder 
    pause(2); 
    
    %go up for the next pin
    qUp = qEnd; 
    qUp(2) = qUp(2) - .1;
    qUp(3) = qUp(3) - .2;
    lynxServo(qUp);
    pause(2); 
end

%go back to original position
lynxServo([0,0,0,0,0,0]);
pause(1); 

pause off;

end

