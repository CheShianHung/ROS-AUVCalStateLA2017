clear;
clc;
%% Given Constants

given_radius = 8*49/8;
given_distance = 60;
color_choice = 3;       % integer; colors listed below
camdevice = 'usb';   % 'webcam' 'image' 'usb'
videofeed = false;      % shows results
thresh = 15;            % threshold sensitivity
scale = 4;              % image processing scaling
display = 1;            % display image scaling
corners = false;        % display shape corners

%% Colors

colors_list = { 'red',[255,0,0];        % 1
    'green',[25,123,76];      % 2
    'yellow',[199,204,120]      %3
    'pink',[255,102,102]
    'bouy',[141,253,170]};

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

switch camdevice
    case 'webcam'
        camera = cv.VideoCapture();
        pause(2);
        img = single(camera.read());
    case 'image'
        img = which('buoy.png');
        img = single(cv.imread(img, 'Flags',1));
    otherwise
        camera = videoinput('tisimaq_r2013',1,'RGB24 (744x480)');
        pause(2);
        img = single(getsnapshot(camera));
        img = img(31:400,71:654,:);
end

l = size(img,1); % length
w = size(img,2); % width
%%

origin = [l/2,w/2];    % Sets the origin coordinates
mask = logical([]);

while 1
    %% Processing
    tic;
    blur = imresize(cv.medianBlur(img,'KSize',5),1/scale);    % blur color image
    Lab = cv.cvtColor(blur./255, 'RGB2Lab');                     % convert color image to LAB colorspace
    
    
    lowerb = color(1,1,:); % lower bound
    upperb = color(1,2,:); % upper bound
    %% Color Threshold
    % filter out all unwanted color
    
    mask = ((Lab(:,:,2) > lowerb(:,:,2)).*...
        (Lab(:,:,2) < upperb(:,:,2)).*(Lab(:,:,3) > lowerb(:,:,3))...
        .*(Lab(:,:,3) < upperb(:,:,3))) > 0;                        % does the same thing as cv.inRange()
    
    output = uint8(cv.bitwise_and(blur,blur,'Mask',mask)); % apply the mask
    output = cv.cvtColor(output,'RGB2GRAY'); % grayscale
    output = cv.threshold(output,60,'MaxValue',255,'Type','Binary'); % threshold
    
    cnts = cv.findContours(output,'Mode','External','Method','Simple'); % detect all contours
    
    %% Arrange contours from largest to smallest
    numcnts = numel(cnts);
    maxArea = [0,NaN];
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
            
            while ~circles && k < 2 && k <= length(A(:,1)) && A(k,1) > 100
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
                                img = cv.circle(img,4.*approx{i},3,'Color',[0,0,255],...
                                    'Thickness',-1); % draws corners of shape
                            end
                        end
                        
                        img = cv.circle(img,scale.*[cX,cY],7,'Color',[255,255,255],...
                            'Thickness',-1); % draws center of shape
                        
                        img = cv.circle(img,scale.*[cX,cY], scale.*radius, 'Color',[0,0,255], ...
                            'Thickness',2, 'LineType','AA');        % draw the circle outline
                    end
                end
                k = k+1;
            end
        end
    end
    
    
    if videofeed
        imshow(uint8(imresize(img,1/display)));
    end
    switch camdevice
        case 'webcam'
            img = single(camera.read()); % initialize camera image for next loop
        case 'image'
            break
        otherwise
            img = single(getsnapshot(camera));
            img = img(31:400,71:654,:);
    end
    t = toc;
    if 1/t > 10
        pause(.1-t);
    end
    if circles
        center = [cX,cY];
        delta_h = (origin(2)-center(2))./10;
        delta_x = (origin(1)-center(1))./10;
        distance = given_distance*given_radius/radius;
        theta = atand(double(distance/delta_x));
        fprintf('Height:%3.2f   Angle:%2.1f     Distance:%3.2f  fps:%2.2f\n',delta_h,theta,distance,1/toc); % print the calculated height and amount needed to turn
    else
        fprintf('fps:%2.2f\n',1/toc);
    end
end