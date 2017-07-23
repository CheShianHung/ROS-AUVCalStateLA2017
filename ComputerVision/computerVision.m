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
global fcdPub bcdPub tiPub tiMsg fcdMsg bcdMsg camera found tiMsgTemp cviSub;
fcdPub = rospublisher('/front_cam_distance','auv_cal_state_la_2017/FrontCamDistance');
bcdPub = rospublisher('/bottom_cam_distance','auv_cal_state_la_2017/BottomCamDistance');
tiPub = rospublisher('/target_info','auv_cal_state_la_2017/TargetInfo');

tiMsg = rosmessage('auv_cal_state_la_2017/TargetInfo');
fcdMsg = rosmessage('auv_cal_state_la_2017/FrontCamDistance');
bcdMsg = rosmessage('auv_cal_state_la_2017/BottomCamDistance');
tiMsgTemp = tiMsg;

cviSub = rossubscriber('/cv_info');

%% Initializa variables
%camera.Name = "";
frontCam = false;
bottomCam = false;
found = false;
%global testTimer;
%testTimer = 0;

%% Rate of loop (100Hz)
rate = rosrate(100);

while 1
    %% Default data
    fcdMsg.FrontCamForwardDistance = 999;
    fcdMsg.FrontCamHorizontalDistance = 999;
    fcdMsg.FrontCamVerticalDistance = 999;
    bcdMsg.BottomCamForwardDistance = 999;
    bcdMsg.BottomCamHorizontalDistance = 999;
    bcdMsg.BottomCamVerticalDistance = 999;
    tiMsg.State = 0;
    tiMsg.Angle = 0;
    tiMsg.Height = 0;
    tiMsg.Direction = 0;
    
    %% Receive cviMsg
    % TaskNumber, GivenColor, GivenShape, GivenLength, GivenDistance
    cviMsg = receive(cviSub) ;
    
    %% Evaluate inputs
    if cviMsg.CameraNumber == 1 && ~frontCam
        delete(imaqfind);
        camera = videoinput('linuxvideo',1,'RGB24_744x480');
        %camera = videoinput('linuxvideo',1,'RGB24_1280x720');
        triggerconfig(camera,'manual');     % speeds up image acquisition for videoinput
        start(camera);
        %         if ~strcmp(camera.Name,'DFK 22AUC03')
        %             camera = webcam(1);
        %         end
        %camera = webcam(1);
        frontCam = true;
        bottomCam = false;
    elseif cviMsg.CameraNumber == 2 && ~bottomCam
        delete(imaqfind);
        camera = videoinput('linuxvideo',2,'RGB24_744x480');
        triggerconfig(camera,'manual');
        start(camera);
        frontCam = false;
        bottomCam = true;
    elseif cviMsg.CameraNumber == 0 && (frontCam || bottomCam)
        stop(camera);
        frontCam = false;
        bottomCam = false;
        found = false;
        %testTimer = 0;
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
global camera fcdPub fcdMsg tiMsg found cviSub;
meandelta_h = zeros(1,15);
meandelta_x = zeros(1,15);
meantheta = zeros(1,15);
meandistance = zeros(1,15);

%% Given Constants
% cviMsg.GivenLength = 8*49/8;
given_radius = msg.GivenLength;
% cviMsg.GivenDistance = 60;
given_distance = msg.GivenDistance;
% cviMsg.GivenColor = 2;                     % integer; colors listed below
color_choice = msg.GivenColor;
camdevice = 'usb';                      % 'webcam' 'image' 'usb'
videofeed = true;                      % shows results
% threshold sensitivity for hue channel (0-255)
scale = 4;                              % image processing scaling
display = 1;                            % display image scaling
corners = true;                        % display shape corners
sigma = 0.33;
contours = true;
linethresh = 10;
topthresh = 2;
xthresh = 40;
d = 50;

%% Brightness
% use the
%       v4l2-ctl
% command inside of a terminal to adjust the exposure and brightness

%% Colors
colors_list = { 'red',[255,0,0]        % 1
    'green',[25,123,76]                    % 2
    'yellow',[199,204,120]                  % 3
    'pink',[255,102,102]
    'bouy',[101,240,127]
    'gate',[145,59,35]};

thresh_list = {[50,150]
    [50,150]
    [50,150]
    [0,0]
    [0,0]
    [6,52]};

%% Tasks
switch msg.TaskNumber
    case 1      % Initialize Buoy detection
        %% Initialize Color
        color = uint8([]);
        color(1,1,:) = colors_list{color_choice,2}; % pick color from RGB choices
        colorHSV = rgb2hsv(color);      % convert color choice to LAB colorspace
        color = colorHSV;
        huethresh = thresh_list{color_choice}(1);
        satthresh = thresh_list{color_choice}(2);
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
                %img = camera.read();
                img = snapshot(camera);
            case 'image'
                img = which('buoy.png');
                img = cv.imread(img, 'Flags',1);
            otherwise
                img = getsnapshot(camera);
                %                 img = snapshot(camera);
                img = img(1:480,163:652,:);
        end
        
        l = size(img,1); % length
        w = size(img,2); % width
        %%
        origin = [l/2,w/2];             % Sets the origin coordinates
        
        
        %%%while ~found || strcmp(msgReceiveFromMaster,'keepFinding')
        m = 1;  % total attempts
        n = 1;  % successful attempts
        while m < 60 && n < 30 && (60-m > 30-n) % take at most 60 frames
            %% Processing
            img = imrotate(img,180);
            blur = imresize(cv.medianBlur(img,'KSize',5),1/scale); % blur color image
            HSV = rgb2hsv(blur);        % convert color image to HSV colorspace
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
                            center = scale.*[cX,cY];
                            radius = scale.*radius;
                            delta_h = (origin(2)-center(2));
                            meandelta_h(n) = delta_h;
                            %                             fcdMsg.FrontCamVerticalDistance = delta_h;
                            delta_x = (center(1)-origin(1));
                            meandelta_x(n) = delta_x;
                            %                             fcdMsg.FrontCamHorizontalDistance = delta_x;
                            distance = given_distance*given_radius/radius;
                            meandistance(n) = distance;
                            %                             fcdMsg.FrontCamForwardDistance = distance;
                            distance = double(distance);
                            %                             delta_x = (origin(1) - center(1))./10;
                            meantheta(n) = atand(double(distance/delta_x));
                            fprintf('Vertical:%3.2f Angle:%3.2f Distance:%3.2f\n',delta_h,meantheta(n),distance); % print the calculated height and amount needed to turn
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
                    %img = camera.read();        % initialize camera image for next loop
                    img = snapshot(camera);
                case 'image'
                    break
                otherwise
                    img = getsnapshot(camera);
                    %                         img = snapshot(camera);
                    img = img(1:480,163:652,:);
            end
            m = m + 1;
        end
        if n == 30 || found
            if ~found
                found = true;
            end
            tiMsg.State = 1;
            tiMsg.Angle = mean(meantheta(15:end));
            tiMsg.Height = mean(meandelta_h(15:end));
            
            %             send(fcdPub, fcdMsg);
            %     send(bcdPub, bcdMsg);
            %%%send(tiPub, tiMsg);
            
            %fcdMsg.FrontCamHorizontalDistance = mean(theta(15:end));
            %fcdMsg.FrontCamForwardDistance = mean(meandistance(15:end));
            fprintf('FOUND\nAVERAGE: Angle:%3.2f Height:%3.2f\n',tiMsg.Angle,tiMsg.Height);
            
        else
            tiMsg.State = 0;
            %imshow(img);
            %%%send(tiPub, tiMsg);
            fprintf('Finding...\n')
        end
        %%%end
        
        
        
        
        
    case 2 % Running buoy detection
        %% Initialize Color
        color = uint8([]);
        color(1,1,:) = colors_list{color_choice,2}; % pick color from RGB choices
        colorHSV = rgb2hsv(color);      % convert color choice to LAB colorspace
        color = colorHSV;
        huethresh = thresh_list{color_choice}(1);
        satthresh = thresh_list{color_choice}(2);
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
                img = img(1:480,163:652,:);
        end
        
        l = size(img,1); % length
        w = size(img,2); % width
        %%
        origin = [l/2,w/2];             % Sets the origin coordinates
        
        %% while loop
        cviMsg.CameraNumber = 1;
        while cviMsg.CameraNumber == 1
        
        %% Processing
        img = imrotate(img,180);
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
        
        
        
            cnts = cv.findContours(mask,'Mode','External','Method','Simple'); % detect all contours
        
            %% Arrange contours from largest to smallest
            numcnts = numel(cnts);
            A = zeros(numcnts,2);false
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
                            center = scale.*[cX,cY];
                            radius = scale.*radius;
                            delta_h = (origin(2)-center(2));
                            fcdMsg.FrontCamVerticalDistance = delta_h;
                            delta_x = (center(1)-origin(1));
                            fcdMsg.FrontCamHorizontalDistance = delta_x;
                            distance = given_distance*given_radius/radius;
                            fcdMsg.FrontCamForwardDistance = distance;
                            distance = double(distance);
                            delta_x = double(distance);
                            fprintf('Height:%3.2f Angle:%3.2f Distance:%3.2f\n',delta_h,delta_x,distance); % print the calculated height and amount needed to turn
                        end
                        k = k+1;
                    end
                end
            else
                fcdMsg.FrontCamVerticalDistance = 999;
                fcdMsg.FrontCamHorizontalDistance = 999;
                fcdMsg.FrontCamForwardDistance = 999;
                fprintf('OBJECT LOST\n');
            end
            if videofeed
                imshow(imresize(img,1/display));
            end
            switch camdevice
                case 'webcam'
                    img = camera.read(); % initialize camera image for next loop
                case 'image'
                    break
                otherwise
                    img = getsnapshot(camera);
                    img = img(1:480,161:650,:);
            end
            send(fcdPub, fcdMsg);
            cviMsg = receive(cviSub) ;
        end
        
        
        
        
        
    case 3 % Gate detection initialization
        scale=1;
        %% Initialize Color
        color = uint8([]);
        color(1,1,:) = colors_list{color_choice,2}; % pick color from RGB choices
        colorHSV = rgb2hsv(color);     % convert color choice to LAB colorspace
        color = colorHSV;
        huethresh = thresh_list{color_choice}(1);
        satthresh = thresh_list{color_choice}(2);
        huethresh = huethresh/255;
        satthresh = satthresh/255;
        colorthresh = zeros(1,2,2);
        colorthresh(:,:,1) = [color(:,:,1)-huethresh,color(:,:,1)+huethresh];
        colorthresh(:,:,2) = [color(:,:,2)-satthresh,color(:,:,2)+satthresh];
        colorthresh(:,:,1) = mod(colorthresh(:,:,1),1);
        colorthresh = uint8(colorthresh*255);
        
        lowerb = colorthresh(1,1,:); % lower bound
        upperb = colorthresh(1,2,:); % upper bound
        
        %% Camera initialization
        
        switch camdevice
            case 'webcam'
                camera = cv.VideoCapture();
                pause(2);
                img = camera.read();
            case 'image'
                img = which('gate.png');
                img = cv.imread(img, 'Flags',1);
            otherwise
                camera = videoinput('tisimaq_r2013',1,'RGB24 (744x480)');
                pause(2);
                img = getsnapshot(camera);
                %                 img = img(31:400,71:654,:);
        end
        
        l = size(img,1); % length
        w = size(img,2); % width
        %%
        
        origin = [l/2,w/2];    % Sets the origin coordinates
        mask = logical([]);
        
        m = 1;
        n = 1;
        while m < 60 && n < 30 && (60-m > 30-n)
            img = imrotate(img,180);
            gate = false;
            topline = false;
            %% Processing
            blur = imresize(cv.medianBlur(img,'KSize',5),1/scale);    % blur color image
            HSV = rgb2hsv(blur);                     % convert color image to LAB colorspace
            HSV = uint8(HSV*255);
            gray = cv.cvtColor(blur,'RGB2GRAY');
            v = median(median(gray));
            lowerv = uint8((1-sigma)*v);
            upperv = uint8((1+sigma)*v);
            
            %% Color Threshold
            % filter out all unwanted color
            
            
            if lowerb(:,:,1) > upperb(:,:,1)
                mask = (HSV(:,:,2) > lowerb(:,:,2)) &...
                    (HSV(:,:,2) < upperb(:,:,2)) & ((HSV(:,:,1) > lowerb(:,:,1))...
                    | (HSV(:,:,1) < upperb(:,:,1)));                        % does the same thing as cv.inRange()
            else
                mask = (HSV(:,:,2) > lowerb(:,:,2)) &...
                    (HSV(:,:,2) < upperb(:,:,2)) & (HSV(:,:,1) > lowerb(:,:,1))...
                    & (HSV(:,:,1) < upperb(:,:,1));
            end
            
            output = uint8(cv.bitwise_and(blur,blur,'Mask',mask)); % apply the mask
            output = cv.cvtColor(output,'RGB2GRAY'); % grayscale
            edged = cv.Canny(output,[lowerv,upperv]);
            
            lines = cv.HoughLinesP(edged, 'Rho',1, 'Theta',pi/180, 'Threshold',80, ...
                'MinLineLength',50, 'MaxLineGap',20);
            A = zeros(numel(lines),6);
            a=0;b=0;c=0;
            X = zeros(numel(lines),1);
            M = zeros(numel(lines),1);
            Y = zeros(numel(lines),1);
            left=zeros(1,2);right=zeros(1,2);top=zeros(1,2);
            for i = 1:numel(lines)
                if videofeed
                    img = cv.line(img,lines{i}(1:2),lines{i}(3:4),'Color',[255,0,0],'Thickness',3,'LineType','AA');
                end
                X(i) = (lines{i}(3)-lines{i}(1))/(lines{i}(4)-lines{i}(2))*(-lines{i}(2))...
                    + lines{i}(1);
                M(i) = (lines{i}(4)-lines{i}(2))/(lines{i}(3)-lines{i}(1));
                Y(i) = (lines{i}(4)-lines{i}(2))/(lines{i}(3)-lines{i}(1))*(-lines{i}(1))...
                    + lines{i}(2);
            end
            C = [M,X,Y];
            C = sortrows(C,'descend');
            B = zeros(numel(lines),2);
            for i = 1:numel(lines)
                if abs(C(i,1)) < topthresh
                    c = c + 1;
                    A(c,5:6) = C(i,[1,3]);
                else
                    a = a + 1;
                    B(a,:) = C(i,1:2);
                end
            end
            a = 0;
            B = sortrows(B,2,'descend');
            for i = 1:numel(lines)
                if ~A(1,1) || (abs(B(i,2)-A(a,2)) < xthresh)
                    a = a + 1;
                    A(a,1:2) = B(i,1:2);
                elseif ~A(1,3) || (abs(B(i,2)-A(b,4)) < xthresh)
                    b = b + 1;
                    A(b,3:4) = B(i,1:2);
                end
            end
            if ~isempty(nonzeros(A(:,1))) && ~isempty(nonzeros(A(:,3)))
                left = [mean(A(1:a,1)),mean(A(1:a,2))];
                right = [mean(A(1:b,3)),mean(A(1:b,4))];
                if ~isempty(nonzeros(A(:,5)))
                    top = [mean(A(1:c,5)),mean(A(1:c,6))];
                end
            end
            if left(2) > right(2)
                temp = left;
                left = right;
                right = temp;
            end
            if (~isnan(left(1)) && ~isnan(right(1))) &&...
                    logical(left(1)*right(1))
                n = n+1;
                gate = true;
                middle = [(left(1)+right(1))/2,(left(2)+right(2))/2];
                if ~isnan(top(1)) && logical(top(1))
                    topline = true;
                    tY = ((top(2)/top(1))+middle(2))/(inv(top(1))-inv(middle(1)));
                    if isinf(middle(1))
                        dY = -d;
                    else
                        dY = -abs(d*sin(atan(middle(1))));
                    end
                    cY = tY - dY;
                    cX = (cY/middle(1))+middle(2);
                    if videofeed
                        img = cv.line(img,scale.*[cX,0],scale.*[cX,l/scale]);
                        img = cv.circle(img,scale.*[cX,cY],4,'Color',[255,255,255],...
                            'Thickness',-1);
                    end
                end
                
                
            end
            
            
            
            
            if gate
                if topline
                    center = [cX,cY];
                    delta_h = (origin(2)-center(2))./10;
                    meandelta_h(n) = delta_h;
                    %                         fcdMsg.FrontCamVerticalDistance = delta_h;
                    delta_x = (origin(1)-center(1))./10;
                    meandelta_x(n) = delta_x;
                    %                         fcdMsg.FrontCamHorizontalDistance = delta_x;
                    %                         distance = given_distance*given_radius/radius;
                    %                         meandistance(n) = distance;
                    %                         fcdMsg.FrontCamForwardDistance = distance;
                    %                         distance = double(distance);
                    %                         delta_x = double(distance);
                    %                         meantheta(n) = atand(double(distance/delta_x));
                    %             fprintf('Height:%3.2f Vertical:%3.2f\n',delta_h,delta_x); % print the calculated height and amount needed to turn
                    
                    %                 n = n + 1;
                    %     %     else
                    %     %         fprintf('fps:%2.2f\n',1/toc);
                else
                    cX = middle(2);
                    if videofeed
                        img = cv.line(img,scale.*[cX,0],scale.*[cX,l/scale],'Color',[255,255,255]);
                    end
                    delta_x = (origin(1)-cX)./10;
                    meandelta_x(n) = delta_x;
                    fprintf('Vertical:%3.2f Distance:%3.2f\n',delta_x); % print the calculated height and amount needed to turn
                end
                n = n+1;
            end
            
            if videofeed
                imshow(imresize(img,1/display));
            end
            
            switch camdevice
                case 'webcam'
                    img = camera.read(); % initialize camera image for next loop
                case 'image'
                    break
                otherwise
                    img = getsnapshot(camera);
                    %                     img = img(31:400,71:654,:);
            end
            m = m+1;
            
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
        
        
        
        
        
    case 4 % Running gate detection
        scale=1;
        %% Initialize Color
        color = uint8([]);
        color(1,1,:) = colors_list{color_choice,2}; % pick color from RGB choices
        colorHSV = rgb2hsv(color);     % convert color choice to LAB colorspace
        color = colorHSV;
        huethresh = thresh_list{color_choice}(1);
        satthresh = thresh_list{color_choice}(2);
        huethresh = huethresh/255;
        satthresh = satthresh/255;
        colorthresh = zeros(1,2,2);
        colorthresh(:,:,1) = [color(:,:,1)-huethresh,color(:,:,1)+huethresh];
        colorthresh(:,:,2) = [color(:,:,2)-satthresh,color(:,:,2)+satthresh];
        colorthresh(:,:,1) = mod(colorthresh(:,:,1),1);
        colorthresh = uint8(colorthresh*255);
        
        lowerb = colorthresh(1,1,:); % lower bound
        upperb = colorthresh(1,2,:); % upper bound
        
        %% Camera initialization
        
        switch camdevice
            case 'webcam'
                %                 camera = cv.VideoCapture();
                %                 pause(2);
                img = camera.read();
            case 'image'
                img = which('gate.png');
                img = cv.imread(img, 'Flags',1);
            otherwise
                %camera = videoinput('tisimaq_r2013',1,'RGB24 (744x480)');
                %                 pause(2);
                img = getsnapshot(camera);
                img = img(1:480,161:650,:);
        end
        
        l = size(img,1); % length
        w = size(img,2); % width
        %%
        
        origin = [l/2,w/2];    % Sets the origin coordinates
        
        m = 1;
        n = 1;
        %% while loop
        cviMsg.CameraNumber = 1;
        while cviMsg.CameraNumber == 1
            img = imrotate(img,180);
            gate = false;
            topline = false;
            
            %% Processing
            blur = imresize(cv.medianBlur(img,'KSize',5),1/scale);    % blur color image
            HSV = rgb2hsv(blur);                     % convert color image to LAB colorspace
            HSV = uint8(HSV*255);
            gray = cv.cvtColor(blur,'RGB2GRAY');
            v = median(median(gray));
            lowerv = uint8((1-sigma)*v);
            upperv = uint8((1+sigma)*v);
            
            %% Color Threshold
            % filter out all unwanted color
            
            
            if lowerb(:,:,1) > upperb(:,:,1)
                mask = (HSV(:,:,2) > lowerb(:,:,2)) &...
                    (HSV(:,:,2) < upperb(:,:,2)) & ((HSV(:,:,1) > lowerb(:,:,1))...
                    | (HSV(:,:,1) < upperb(:,:,1)));                        % does the same thing as cv.inRange()
            else
                mask = (HSV(:,:,2) > lowerb(:,:,2)) &...
                    (HSV(:,:,2) < upperb(:,:,2)) & (HSV(:,:,1) > lowerb(:,:,1))...
                    & (HSV(:,:,1) < upperb(:,:,1));
            end
            
            output = uint8(cv.bitwise_and(blur,blur,'Mask',mask)); % apply the mask
            output = cv.cvtColor(output,'RGB2GRAY'); % grayscale
            edged = cv.Canny(output,[lowerv,upperv]);
            
            lines = cv.HoughLinesP(edged, 'Rho',1, 'Theta',pi/180, 'Threshold',35, ...
                'MinLineLength',1, 'MaxLineGap',40);   % adjust Threshold, MinLineLength, MaxLineGap
            A = zeros(numel(lines),6);
            a=0;b=0;c=0;
            X = zeros(numel(lines),1);
            M = zeros(numel(lines),1);
            Y = zeros(numel(lines),1);
            left=zeros(1,2);right=zeros(1,2);top=zeros(1,2);
            for i = 1:numel(lines)
                if videofeed
                    img = cv.line(img,lines{i}(1:2),lines{i}(3:4),'Color',[255,0,0],'Thickness',3,'LineType','AA');
                end
                X(i) = (lines{i}(1)+lines{i}(3))/2;
                %                 X(i) = (lines{i}(3)-lines{i}(1))/(lines{i}(4)-lines{i}(2))*(-lines{i}(2))...
                %                     + lines{i}(1);
                M(i) = (lines{i}(4)-lines{i}(2))/(lines{i}(3)-lines{i}(1));
                Y(i) = (lines{i}(2)+lines{i}(4))/2;
                %                 Y(i) = (lines{i}(4)-lines{i}(2))/(lines{i}(3)-lines{i}(1))*(-lines{i}(1))...
                %                     + lines{i}(2);
            end
            C = [M,X,Y];
            C = sortrows(C,'descend');
            B = zeros(numel(lines),2);
            for i = 1:numel(lines)
                if abs(C(i,1)) < topthresh
                    c = c + 1;
                    A(c,5:6) = C(i,[1,3]);
                else
                    a = a + 1;
                    B(a,:) = C(i,1:2);
                end
            end
            a = 0;
            B = sortrows(B,2,'descend');
            for i = 1:numel(lines)
                if isnan(B(i,1))
                elseif ~A(1,1) || (abs(B(i,2)-A(a,2)) < xthresh)
                    a = a + 1;
                    A(a,1:2) = B(i,1:2);
                elseif ~A(1,3) || (abs(B(i,2)-A(b,4)) < xthresh)
                    b = b + 1;
                    A(b,3:4) = B(i,1:2);
                end
            end
            if ~isempty(nonzeros(A(:,1))) && ~isempty(nonzeros(A(:,3)))
                left = [mean(A(1:a,1)),mean(A(1:a,2))];
                right = [mean(A(1:b,3)),mean(A(1:b,4))];
                if ~isempty(nonzeros(A(:,5)))
                    top = [mean(A(1:c,5)),mean(A(1:c,6))];
                end
            end
            if left(2) > right(2)
                temp = left;
                left = right;
                right = temp;
            end
            if (~isnan(left(1)) && ~isnan(right(1))) &&...
                    logical(left(1)) && logical(right(1))
                n = n+1;
                gate = true;
                middle = [(left(1)+right(1))/2,(left(2)+right(2))/2];
                if ~isnan(top(1)) && logical(top(1))
                    topline = true;
                    tY = top(2);
                    %                     tY = (top(2)+middle(2)*top(1))*middle(1)/(middle(1)-top(1));
                    %                     tY = ((top(2)/top(1))+middle(2))/(inv(top(1))-inv(middle(1)));
                    %                     if isinf(middle(1))
                    dY = -d;
                    %                     else
                    %                         dY = -abs(d*sin(atan(middle(1))));
                    %                     end
                    cY = tY - dY;
                    cX = middle(2);
                    %                     cX = (cY/middle(1))+middle(2);
                    if videofeed
                        img = cv.line(img,scale.*[cX,0],scale.*[cX,l/scale]);
                        img = cv.circle(img,scale.*[cX,cY],4,'Color',[255,255,255],...
                            'Thickness',-1);
                    end
                end
            end
            
            if gate
                if topline
                    if ~isnan(cX) && cX<=490 && cX>=1
                        lengthL = max(left(:,2))-min(left(:,2));
                        lengthR = max(right(:,2))-min(right(:,2));
                        center = [cX,cY];
                        delta_h = (origin(2)-center(2));
                        meandelta_h(n) = delta_h;
                        fcdMsg.FrontCamVerticalDistance = delta_h;
                        delta_x = (center(1)-origin(1));
                        meandelta_x(n) = delta_x;
                        fcdMsg.FrontCamHorizontalDistance = delta_x;
                        %                         distance = given_distance*given_radius/radius;
                        %                         meandistance(n) = distance;
                        %                         fcdMsg.FrontCamForwardDistance = distance;
                        %                         distance = double(distance);
                        %                         delta_x = double(distance);
                        %                         meantheta(n) = atand(double(distance/delta_x));
                        fprintf('Vertical:%3.2f Horizontal:%3.2f Left:%3.2f Right:%3.2f\n',delta_h,delta_x,lengthL,lengthR); % print the calculated height and amount needed to turn
                        
                        %                 n = n + 1;
                        %     %     else
                        %     %         fprintf('fps:%2.2f\n',1/toc);
                    end
                else
                    if ~isnan(middle(2)) && middle(2)<=490 && middle(2)>=1
                        lengthL = max(left(:,2))-min(left(:,2));
                        lengthR = max(right(:,2))-min(right(:,2));
                        cX = middle(2);
                        if videofeed
                            img = cv.line(img,scale.*[cX,0],scale.*[cX,l/scale],'Color',[255,255,255]);
                        end
                        delta_x = (origin(1)-cX);
                        meandelta_x(n) = delta_x;
                        fprintf('Horizontal:%3.2f Left:%3.2f Right:%3.2f\n',delta_x,lengthL,lengthR); % print the calculated height and amount needed to turn
                    end
                end
%%%                 n = n+1;
            end
            
            if videofeed
                imshow(imresize(img,1/display));
            end
            
            switch camdevice
                case 'webcam'
                    img = camera.read(); % initialize camera image for next loop
                case 'image'
                    break
                otherwise
                    img = getsnapshot(camera);
                    img = img(1:480,161:650,:);
            end
%             m = m+1;
            send(fcdPub, fcdMsg);
            cviMsg = receive(cviSub) ;
        end
%         if n == 30 || found
%             if ~found
%                 found = true;
%             end
%             tiMsg.State = 1;
%             tiMsg.Angle = mean(meandelta_x(15:end));
%             tiMsg.Height = mean(meandelta_h(15:end));
%             %fcdMsg.FrontCamHorizontalDistance = mean(theta(15:end));
%             %fcdMsg.FrontCamForwardDistance = mean(meandistance(15:end));
%             fprintf('FOUND\nAVERAGE: Height:%3.2f Angle:%3.2f\n',tiMsg.Angle,tiMsg.Direction);
%         else
%             tiMsg.State = 0;
%             %imshow(img);
%             fprintf('Finding...\n')
%         end
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
