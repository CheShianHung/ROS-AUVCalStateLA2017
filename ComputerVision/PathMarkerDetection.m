clear;
clc;
%% Given Constants

thresh = 20;
color_choice = 4;   % integer from 1-4; colors listed below
camdevice = 'image';   % 'webcam' 'image' 'usb'
videofeed = 1; % shows results

%% Colors

colors_list = {'green',[0,100,0]; % 1
    'orange',[255,128,0]; %2
    'black',[0,0,0]; % 3
    'red',[187,49,36]; % etc
    'yellow',[204,149,46];
    'blue',[23,58,122];
    'card',[62,143,200];
    'block',[97,108,159]};


%% Initialize OpenCV

if ispc
    addpath('C:\dev\mexopencv');
    addpath('C:\dev\mexopencv\opencv_contrib');
else
    addpath('~/cv/mexopencv');
    addpath('~/cv/mexopencv/opencv_contrib');
end

%% Initialize Color
color = single([]); % cv.cvtColor() needs values between 0-1
color(1,1,:) = colors_list{color_choice,2}; % pick color from RGB choices
color = cv.cvtColor(color./255,'RGB2Lab');     % convert color choice to LAB colorspace
color = int8(color); % Lab values are between -128:128
color = [color-thresh,color+thresh]; % make a color threshold range

%% Camera initialization

if strcmp(camdevice,'webcam')
    camera = cv.VideoCapture();
    pause(2);
    img = single(camera.read());
elseif strcmp(camdevice,'image')
    img = which('blue.jpg');
    img = single(cv.imread(img, 'Flags',1));
else
    camera = videoinput('tisimaq_r2013',1,'RGB24 (744x480)');
    pause(2);
    img = single(getsnapshot(camera));
    img = img(31:400,71:654,:);
end

l = size(img,1); % length
w = size(img,2); % width

%%
origin = [l/2,w/2];    % Sets the origin coordinates
rectangles = 0;

while 1
    %% Processing
    
    blur = cv.medianBlur(img,'KSize',5);    % blur color image
    Lab = cv.cvtColor(blur./255, 'RGB2Lab');
    
    lowerb = color(1,1,:); % lower bound
    upperb = color(1,2,:); % upper bound
    
    %% lab
    mask(:,:,1) = logical((Lab(:,:,2) > lowerb(:,:,2)).*...
        (Lab(:,:,2) < upperb(:,:,2)).*(Lab(:,:,3) > lowerb(:,:,3))...
        .*(Lab(:,:,3) < upperb(:,:,3))); % does the same thing as cv.inRange()
    
    output = uint8(cv.bitwise_and(blur,blur,'Mask',mask)); % apply the mask
    output = cv.cvtColor(output,'RGB2GRAY'); % grayscale
    output = cv.threshold(output,60,'MaxValue',255,'Type','Binary'); % threshold
    
    %% Contour Detection
    
    cnts = cv.findContours(output,'Mode','External','Method','Simple'); % detect all contours
    
    %     A = zeros(1,numel(cnts));
    maxArea = [0,NaN];
    A = [];
    if numel(cnts) > 0
        for i = 1:numel(cnts)
            A = cv.contourArea(cnts{i});
            if A > maxArea(1)
                maxArea = [A,i];
            end
        end
        c = maxArea(2); % index of contour with largest area
        
        if ~isnan(c)
            
            
            %% Calculate the shape of the detected contour
            
            cnt = cnts(c);
            M = cv.moments(cnt{1,1});
            cX = int16(M.m10/M.m00);
            cY = int16(M.m01/M.m00);
            peri = cv.arcLength(cnt{1,1},'Closed',1);
            approx = cv.approxPolyDP(cnt{1,1},'Epsilon',...
                0.04*peri,'Closed',1); % approximate the corners of the shape
            if length(approx) == 4
                rectangles = 1;
                if videofeed
                    for i = 1:4
                        img = cv.circle(img,approx{i},3,'Color',[0,0,255],...
                            'Thickness',-1); % draws corners of shape
                    end
                end
                img = cv.circle(img,[cX,cY],7,'Color',[255,255,255],...
                    'Thickness',-1); % draws center of shape
            end
        end
    end
    
    %% Calculate the angle of orientation
    
    if rectangles
        minDist = [inf,NaN];
        dist = [];
        center = single([cX,cY]);
        P = zeros(4,2);
        for i = 1:2
            dist = sqrt((approx{i}(1)-approx{i+1}(1))^2+(approx{i}(2)-approx{i+1}(2))^2);
            if dist < minDist(1)
                minDist = [dist,i];
            end
        end
        c = minDist(2);
        edge = (approx{c}+approx{c+1})/2;
        edge = edge - center;
        theta = -atan2d(edge(1),edge(2));
        
        [~,radius] =  cv.minEnclosingCircle(cnt{1});
        %         delta_h = (origin(2)-center(2))./10;
        %         delta_x = double((origin(1)-center(1))./10);
        
        fprintf('Angle:%2.1f \n',theta); % print the calculated height and amount needed to turn
    end
    if videofeed
        imshow(uint8(img));
    end
    if strcmp(camdevice,'webcam')
        img = single(camera.read()); % initialize camera image for next loop
    elseif strcmp(camdevice,'image')
        break
    else
        img = single(getsnapshot(camera));
        img = img(31:400,71:654,:);
    end
    rectangles = 0;
end