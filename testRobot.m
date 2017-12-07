function [] = testRobot(pinLetter, pinNumber)
%Note that the pinLetter has to be a capital letter between 'A' and 'Q'

%xyz distance fo the corner of the perfboard closest to the robot from the
%base of the robot
xPerfToBase = 10;%mm
yPerfToBase = 20;%mm
zPerfToBase = 20;%mm

%distance from the left corner to the closest pin
xCornerToPin = 5;%mm
yCornerToPin = 9.23;%mm

%distance between the centers of respective pins 
distBtwPins = 2.11; 

%convert the pinLetter to a number 
alphabet = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ';
alphaNumber = strfind(alphabet,pinLetter); 

%get the goal xyz position to actuate the tip of the robot to
xGoal = alphaNumber * distBtwPins + xCornerToPin + xPerfToBase; 
yGoal = alphaNumber * distBtwPins + yCornerToPin + yPerfToBase;
zGoal = zPerfToBase;

%starting position is initial q=zero vector
qStart = [0,0,0,0,0,0];
%find the goal orientation of the end effector 
goalOrientation = eye(3); %PUT IN REAL ORIENTATION! 
%find qEnd using IK using the goal EE position and also the goal
%orientation 
T = [[goalOrientation;[0,0,0]],[xGoal;yGoal;zGoal;0]];
qEnd = IK_lynx_sol(T); 

%run the robot using a potential fields planner 
potentialFieldPlanner(qStart,qEnd); 

end

