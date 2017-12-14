function [] = potentialFieldPlanner(qStart,qEnd)
%% @param: posOfObstacles a mx3 matrix with the xyz positions of each of the m obstacles 
% @param: ri: a mx1 matrix of radii for the m obstacles 
% @param: qStart: a 6x1 matrix of the joint configurations of starting pos
% @param: qEnd: a 6x1 matrix of the joint configurations of ending pos

%start the lynx
%lynxStart();        

% set the epsilon for what a reasonable distance to stop is 
epsilon = .5; 

%assign qCurr to be qStart. qCurr will be updated throughout the while loop
qCurr = qStart; 

%loop while the distance between the configuration spaces have not reached
%the goal configuration 
while any((qEnd(1,1:5) - qCurr(1,1:5)) > epsilon)
    %% Calculate the attractive force
    [XCurr,~] = updateQ(qCurr); 
    [XEnd, ~ ] = updateQ(qEnd); 
    
    %% Calculate the attractive force, a 6x1 vector 

    %initialize taua
    tauaTotal = zeros(6,1); 
    
    %Find all the J matrices using qCurr 
    JAll = [];
    
    syms q1 q2 q3 q4 q5 q6
    for joint = 1:5
        %% Calculate the attractive joint effort for joint i 
        %Find the positions of the current position and the end goal of the
        % joint i. 
        currPosJointI = XCurr(joint,:);
        endPosJointI = XEnd(joint,:);
        
        %instantiate Fa 
        Fa = zeros(1,3); 
        
        %if case over whether the current pos of joint i is at it's goal 
        if( norm(currPosJointI - endPosJointI) ~= 0)
            %if joint i is not at its goal position, then calculate Fa
            Fa = - (currPosJointI - endPosJointI) / norm(currPosJointI - endPosJointI);
        else
            Fa = zeros(1,3); 
        end 
        
        %find the jacobian for joint i 
        J = findJacobian(qCurr,joint);
        
        %substitute values of qCur into symbolic J 
        J = subs(J, q1, qCurr(1));
        J = subs(J, q2, qCurr(2));
        J = subs(J, q3, qCurr(3));
        J = subs(J, q4, qCurr(4));
        J = subs(J, q5, qCurr(5));
        J = subs(J, q6, qCurr(6));
        
        % Convert sym matrix back to numeric
        J = double(J);
        
        %Only take the first joint columns 
        Jnew = J(:,1:joint); 
        
        %%add zeros if Jnew is not a 6x5
        colsOfZerosToAdd = 6 - joint;
        Jnew = [Jnew, zeros(6,colsOfZerosToAdd)];
        
        %calculate the taua for this joint 
        Jv = Jnew(1:3,:);
        taua = Jv'*Fa';
        %6x1   =    [6x3][3x1]
            
        %find the total Taua by adding the taua for this joint 
        tauaTotal = tauaTotal + taua; %final result of 5x6 matrix
        
    end 

    %set the tau = to the tauatotal + the taurtotal
    tau = tauaTotal;
    
    %set the step rate
    alpha = 0.01;
    
    %ensure there is not
    if (norm(tau)~=0)
        qCurr = qCurr' + alpha * (tau / norm(tau));
    else
        qCurr = qCurr';
    end
    
    %change qCurr to be 1x6 not 6x1 to match with input
    qCurr = qCurr' ;
    disp(qCurr);
    lynxServo(qCurr);
    scatter3(XCurr(6,1),XCurr(6,2),XCurr(6,3)); hold on;
end
end