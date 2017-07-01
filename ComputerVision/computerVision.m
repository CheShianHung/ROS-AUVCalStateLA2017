clear;
clc;

%% Useful commands
% hostname -I
% rosinit('10.85.43.217');
% rosshutdown;
% !synclient HorizEdgeScroll=0 HorizTwoFingerScroll=0
% roboticsAddons
% folderpath = '/home/diego/catkin_ws/src';
% rosgenmsg(folderpath);

%% Initialize subscriber and publishers
fcdPub = rospublisher('/front_cam_distance','auv_cal_state_la_2017/FrontCamDistance');
bcdPub = rospublisher('/bottom_cam_distance','auv_cal_state_la_2017/BottomCamDistance');
tiPub = rospublisher('/target_info','auv_cal_state_la_2017/TargetInfo');
fcdMsg = rosmessage('auv_cal_state_la_2017/FrontCamDistance');
bcdMsg = rosmessage('auv_cal_state_la_2017/BottomCamDistance');
tiMsg = rosmessage('auv_cal_state_la_2017/TargetInfo');

cviSub = rossubscriber('/cv_info');

%% Initializa boolean variables
frontCam = false;
bottomCam = false;

%% Rate of loop (10Hz)
rate = rosrate(10);

while 1
    %% Default data
    fcdMsg.FrontCamForwardDistance = 0;
    fcdMsg.FrontCamHorizontalDistance = 0;
    fcdMsg.FrontCamVerticalDistance = 0;
    bcdMsg.BottomCamForwardDistance = 0;
    bcdMsg.BottomCamHorizontalDistance = 0;  
    bcdMsg.BottomCamVerticalDistance = 0;
    tiMsg.State = 0;
    tiMsg.Angle = 0;
    tiMsg.Height = 0;
    tiMsg.Direction = 0;
    
    %% Receive Msg
    cviMsg = receive(cviSub) ;
    
    %% Evaluate inputs
    if cviMsg.CameraNumber == 1 && ~frontCam
        %camera = cv.VideoCapture();
        frontCam = true; 
    elseif cviMsg.CameraNumber == 2 && ~bottomCam
        %camera = cv.VideoCapture();
        bottomCam = true;
    elseif ~cviMsg.CameraNumber == 1 && ~cviMsg.CameraNumber == 2 && (frontCam || bottomCam)
        frontCam = false;
        bottomCam = false;
    end
            
    
    %% Run camera
    if frontCam
        FrontCamera(cviMsg.TaskNumber, cviMsg.GivenColor, cviMsg.GivenShape, cviMsg.GivenLength, cviMsg.GivenDistance);
    end
    
    if bottomCam
        BottomCamera(cviMsg.TaskNumber, cviMsg.GivenColor, cviMsg.GivenShape, cviMsg.GivenLength, cviMsg.GivenDistance);
    end
    
    %% Send Msg
    send(fcdPub, fcdMsg);
    send(bcdPub, bcdMsg);
    send(tiPub, tiMsg);
    
    %% Loop rate (10Hz)
    waitfor(rate);
end

%% Front Camera
function FrontCamera(taskNum, givenC, givenS, givenL, givenD)
    fprintf('taskNum: %d ,givenC: %d ,givenS: %d ,givenL: %.2f ,givenD: %.2f', taskNum, givenC, givenS, givenL, givenD); 
end

%% Bottom Camera
function BottomCamera(taskNum, givenC, givenS, givenL, givenD)
    frintf('taskNum: %d ,givenC: %d ,givenS: %d ,givenL: %.2f ,givenD: %.2f', taskNum, givenC, givenS, givenL, givenD);  
end