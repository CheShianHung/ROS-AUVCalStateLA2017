clear;
clc;
addpath('~/cv/mexopencv');
addpath('~/cv/mexopencv/opencv_contrib');

%% Useful commands
% hostname -I
% rosinit('10.85.43.217');
% rosshutdown;
% !synclient HorizEdgeScroll=0 HorizTwoFingerScroll=0
% roboticsAddons
% folderpath = '/home/auv/catkin_ws/src';
% rosgenmsg(folderpath);

%% Initialization
fcdPub = rospublisher('/front_cam_distance','auv_cal_state_la_2017/FrontCamDistance');
bcdPub = rospublisher('/bottom_cam_distance','auv_cal_state_la_2017/BottomCamDistance');
tiPub = rospublisher('/target_info','auv_cal_state_la_2017/TargetInfo');

global tiMsg;
global fcdMsg;
global bcdMsg;
global camera;
global found;
global testTimer;
global tiMsgTemp;

tiMsg = rosmessage('auv_cal_state_la_2017/TargetInfo');
fcdMsg = rosmessage('auv_cal_state_la_2017/FrontCamDistance');
bcdMsg = rosmessage('auv_cal_state_la_2017/BottomCamDistance');
tiMsgTemp = tiMsg;

cviSub = rossubscriber('/cv_info');

%% Initializa variables
frontCam = false;
bottomCam = false;
found = false;
testTimer = 0;

%% Rate of loop (100Hz)
rate = rosrate(100);

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

    %% Receive cviMsg
    % TaskNumber, GivenColor, GivenShape, GivenLength, GivenDistance
    cviMsg = receive(cviSub) ;

    %% Evaluate inputs
    if cviMsg.CameraNumber == 1 && ~frontCam
        camera = videoinput('linuxvideo',1,'RGB24_744x480');
        frontCam = true;
        bottomCam = false;
    elseif cviMsg.CameraNumber == 2 && ~bottomCam
        camera = videoinput('linuxvideo',2,'RGB24_744x480');
        frontCam = false;
        bottomCam = true;
    elseif cviMsg.CameraNumber == 0 && (frontCam || bottomCam)
        frontCam = false;
        bottomCam = false;
        found = false;
        testTimer = 0;
        disp("task done");
    end

    %% Run camera
    if frontCam
        FrontCamera(cviMsg);
    end

    if bottomCam
        BottomCamera(cviMsg);
    end

    %% Send Msg
    send(fcdPub, fcdMsg);
    send(bcdPub, bcdMsg);
    send(tiPub, tiMsg);

    %% Loop rate (10Hz)
    waitfor(rate);
end

%% FRONTCAMERA RoboSub computer vision tasks for front camera
%   FrontCamera(cviMsg)
%   cviMsg is a struct that contains
%       cviMsg.TaskNumber
%           1 = BuoyDetection
%           2 = PathMarkerDetection, etc...
%       cviMsg.GivenColor
%       cviMsg.GivenShape
%       cviMsg.GivenLength
%       cviMsg.GivenDistance

function FrontCamera(msg)

    %% Initialize outputs
    global camera;
    global fcdMsg;
    global tiMsg;
    global found;
    meandelta_h = zeros(1,15);
    meandelta_x = zeros(1,15);
    theta = zeros(1,15);
    meandistance = zeros(1,15);

    %% Given Constants
    % cviMsg.GivenLength = 8*49/8;
    given_radius = msg.GivenLength;
    % cviMsg.GivenDistance = 60;
    given_distance = msg.GivenDistance;
    % cviMsg.GivenColor = 2;                     % integer; colors listed below
    color_choice = msg.GivenColor;
    camdevice = 'usb';                      % 'webcam' 'image' 'usb'
    videofeed = false;                      % shows results
    satthresh = 150;                         % threshold sensitivity for saturation channel (0-255)
    huethresh = 50;                         % threshold sensitivity for hue channel (0-255)
    scale = 4;                              % image processing scaling
    display = 1;                            % display image scaling
    corners = true;                        % display shape corners

    %% Colors
    colors_list = { 'red',[255,0,0];        % 1
    'green',[25,123,76];                    % 2
    'yellow',[199,204,120]                  % 3
    'pink',[255,102,102]
    'bouy',[101,240,127]};

    %% Tasks
    switch msg.TaskNumber
        case 1      % Buoy detection
            %% Initialize Color
            color = uint8([]);
            color(1,1,:) = colors_list{color_choice,2}; % pick color from RGB choices
            colorHSV = rgb2hsv(color);      % convert color choice to LAB colorspace
            color = colorHSV;
            huethresh = huethresh/255;
            satthresh = satthresh/255;
            colorthresh = zeros(1,2,2);
            colorthresh(:,:,1) = [color(:,:,1)-huethresh,color(:,:,1)+huethresh];
            colorthresh(:,:,2) = [color(:,:,2)-satthresh,color(:,:,2)+satthresh];
            colorthresh(:,:,1) = mod(colorthresh(:,:,1),1);
            colorthresh = uint8(colorthresh*255);

            lowerb = colorthresh(1,1,:);    % lower bound
            upperb = colorthresh(1,2,:);    % upper bound

            %% Camera initialization

            switch camdevice
                case 'webcam'
                    img = camera.read();
                case 'image'
                    img = which('buoy.png');
                    img = cv.imread(img, 'Flags',1);
                otherwise
                    img = getsnapshot(camera);
                    img = img(31:400,71:654,:);
            end

            l = size(img,1); % length
            w = size(img,2); % width
            %%
            origin = [l/2,w/2];             % Sets the origin coordinates

            m = 1;  % total attempts
            n = 1;  % successful attempts
            while m < 60 && n < 30 && (60-m > 30-n) % take at most 60 frames
                %% Processing
                tic;
                blur = imresize(cv.medianBlur(img,'KSize',5),1/scale); % blur color image
                HSV = rgb2hsv(blur);        % convert color image to LAB colorspace
                HSV = uint8(HSV*255);


                %% Color Threshold - filter out all unwanted color
                if lowerb(:,:,1) > upperb(:,:,1)
                    mask = (HSV(:,:,2) > lowerb(:,:,2)) &...
                        (HSV(:,:,2) < upperb(:,:,2)) & ((HSV(:,:,1) > lowerb(:,:,1))...
                        | (HSV(:,:,1) < upperb(:,:,1))); % does the same thing as cv.inRange()
                else
                    mask = (HSV(:,:,2) > lowerb(:,:,2)) &...
                        (HSV(:,:,2) < upperb(:,:,2)) & (HSV(:,:,1) > lowerb(:,:,1))...
                        & (HSV(:,:,1) < upperb(:,:,1));
                end

                %output = uint8(cv.bitwise_and(blur,blur,'Mask',mask)); % apply the mask
                %output = cv.cvtColor(output,'RGB2GRAY'); % grayscale
                %output = cv.threshold(output,60,'MaxValue',255,'Type','Binary'); % threshold

                cnts = cv.findContours(mask,'Mode','External','Method','Simple'); % detect all contours

                %% Arrange contours from largest to smallest
                numcnts = numel(cnts);
                A = zeros(numcnts,2);
                circles = false;
                if numcnts > 0
                    A(1:numcnts,2) = (1:numcnts);
                    for i = 1:numcnts
                        A(i,1) = cv.contourArea(cnts{i});
                    end
                    A = sortrows(A,'descend');
                    k = 1;


                    if ~isnan(A(1,2))
                        %% Calculate the shape of the detected contour
                        while ~circles && k < 5 && k <= length(A(:,1)) && A(k,1) > 10
                            c = A(k,2);             % index of contour with largest area
                            cnt = cnts{c};
                            M = cv.moments(cnt);
                            cX = int16(M.m10/M.m00);
                            cY = int16(M.m01/M.m00);
                            peri = cv.arcLength(cnt,'Closed',1);
                            approx = cv.approxPolyDP(cnt,'Epsilon',...
                                0.04*peri,'Closed',1); % approximate the corners of the shape
                            if length(approx) > 3
                                circles = true;
                                n = n + 1;
                                [~,radius] =  cv.minEnclosingCircle(cnt);
                                if videofeed
                                    if corners
                                        for i = 1:length(approx)
                                            img = cv.circle(img,scale*approx{i},3,'Color',[0,0,255],...
                                                'Thickness',-1); % draws corners of shape
                                        end
                                    end

                                    img = cv.circle(img,scale.*[cX,cY],7,'Color',[255,255,255],...
                                        'Thickness',-1); % draws center of shape

                                    img = cv.circle(img,scale.*[cX,cY], scale.*radius, 'Color',[0,0,255], ...
                                        'Thickness',2, 'LineType','AA'); % draw the circle outline
                                end
                                center = [cX,cY];
                                delta_h = (origin(2)-center(2))./10;
                                meandelta_h(n) = delta_h;
                                fcdMsg.FrontCamVerticalDistance = delta_h;
                                delta_x = (origin(1)-center(1))./10;
                                meandelta_x(n) = delta_x;
                                fcdMsg.FrontCamHorizontalDistance = delta_x;
                                distance = given_distance*given_radius/radius;
                                meandistance(n) = distance;
                                fcdMsg.FrontCamForwardDistance = distance;
                                theta(n) = atand(double(distance/delta_x));
                                fprintf('Height:%3.2f Angle:%3.2f Distance:%3.2f\n',delta_h,delta_x,distance); % print the calculated height and amount needed to turn
                            end
                            k = k+1;
                        end
                    end
                end


                if videofeed
                    imshow(imresize(img,1/display));
                end
                switch camdevice
                    case 'webcam'
                        img = camera.read();        % initialize camera image for next loop
                    case 'image'
                        break
                    otherwise
                        img = getsnapshot(camera);
                        img = img(31:400,71:654,:);
                end
                m = m + 1;
            end
            if n == 30 || found
                if ~found
                    found = true;
                end
                tiMsg.State = 1;
                tiMsg.Angle = mean(meandelta_x(15:end));
                tiMsg.Height = mean(meandelta_h(15:end));
                %fcdMsg.FrontCamHorizontalDistance = mean(theta(15:end));
                %fcdMsg.FrontCamForwardDistance = mean(meandistance(15:end));
                fprintf('FOUND\nAVERAGE: Height:%3.2f Angle:%3.2f\n',tiMsg.Angle,tiMsg.Direction);
            else
                tiMsg.State = 0;
                %imshow(img);
                fprintf('Finding...\n')
            end
    end
    %global testTimer;
    %testTimer = testTimer + 0.1;
    %if testTimer >= 5
    %    tiMsg.State = 1;
    %    tiMsg.Angle = 45;
    %    tiMsg.Height = -4;
    %    tiMsg.Direction = 1;
    %    disp('Object found. Sending data to master...');
    %else
    %    disp('Finding object...');
    %    disp(testTimer);
    %end
end

%% Bottom Camera
function BottomCamera(msg)
    %frintf('taskNum: %d ,givenC: %d ,givenS: %d ,givenL: %.2f ,givenD: %.2f', msg, givenC, givenS, givenL, givenD);
end
