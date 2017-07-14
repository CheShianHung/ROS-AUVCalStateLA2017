clear;
clc;
%% Given Constants

given_radius = 8*49/8;
given_distance = 60;
color_choice = 2;       % integer; colors listed below
camdevice = 'usb';   % 'webcam' 'image' 'usb'
videofeed = true;      % shows results
satthresh = 150;        % threshold sensitivity for saturation channel (0-255)
huethresh = 50;        % threshold sensitivity for hue channel (0-255)
scale = 4;              % image processing scaling
display = 1;            % display image scaling
corners = true;        % display shape corners
global camera;


%% Colors

colors_list = { 'red',[255,0,0];        % 1
    'green',[25,123,76];      % 2
    'yellow',[199,204,120]      %3
    'pink',[255,102,102]
    'bouy',[101,240,127]};

%% Initialize OpenCV

if ispc
    addpath('C:\dev\mexopencv');
    addpath('C:\dev\mexopencv\opencv_contrib');
else
    addpath('~/cv/mexopencv');
    addpath('~/cv/mexopencv/opencv_contrib');
end

%% Initialize Color
color = uint8([]);
color(1,1,:) = colors_list{color_choice,2}; % pick color from RGB choices
colorHSV = rgb2hsv(color);     % convert color choice to LAB colorspace
color = colorHSV;
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
%         camera = cv.VideoCapture(0);
%         pause(2);
        img = camera.read();
    case 'image'
        img = which('buoy.png');
        img = cv.imread(img, 'Flags',1);
    otherwise
        camera = videoinput('linuxvideo',1,'RGB24_744x480');
        pause(2);
        img = getsnapshot(camera);
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
    HSV = rgb2hsv(blur);                     % convert color image to LAB colorspace
    HSV = uint8(HSV*255);
    
    

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
    output = cv.threshold(output,60,'MaxValue',255,'Type','Binary'); % threshold
    
    cnts = cv.findContours(mask,'Mode','External','Method','Simple'); % detect all contours
    
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
            
            while ~circles && k < 10 && k <= length(A(:,1)) && A(k,1) > 25
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
        imshow(imresize(img,1/display));
    end
    switch camdevice
        case 'webcam'
            img = camera.read(); % initialize camera image for next loop
        case 'image'
            break
        otherwise
            img = getsnapshot(camera);
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