clear;
clc;
%% Given Constants

given_radius = 8*49/8;
given_distance = 60;
color_choice = 2;   % integer from 1-3; colors listed below
camdevice = 'webcam';   % 'webcam' 'image' 'usb'
videofeed = false; % shows results
thresh = 30;

%% Colors

colors_list = { 'red',[255,0,0];        % 1
    'green',[0,100,0];      % 2
    'yellow',[255,255,0]
    'pink',[255,102,102]
    'bouy',[141,253,170]};  % 3

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
    tic
    %     gray = cv.cvtColor(img, 'RGB2GRAY');    % convert to grayscale
    %     gray = cv.medianBlur(gray, 'KSize',5);  % blur grayscaled image
    blur = imresize(cv.medianBlur(img,'KSize',5),.25);    % blur color image
    Lab = cv.cvtColor(blur./255, 'RGB2Lab');                     % convert color image to LAB colorspace
    
    
    lowerb = color(1,1,:); % lower bound
    upperb = color(1,2,:); % upper bound
    %% HoughCircles
    % detect all circles in the image
    
    mask = ((Lab(:,:,2) > lowerb(:,:,2)).*...
        (Lab(:,:,2) < upperb(:,:,2)).*(Lab(:,:,3) > lowerb(:,:,3))...
        .*(Lab(:,:,3) < upperb(:,:,3))) > 0; % does the same thing as cv.inRange()
    
    
    %     tic;
    %     mask = (double(Lab(:,:,2) > lowerb(:,:,2)).*...
    %         double(Lab(:,:,2) < upperb(:,:,2)).*double(Lab(:,:,3) > lowerb(:,:,3))...
    %         .*double(Lab(:,:,3) < upperb(:,:,3))) > 0; % does the same thing as cv.inRange()
    %     toc
    
    output = uint8(cv.bitwise_and(blur,blur,'Mask',mask)); % apply the mask
    
    output = cv.cvtColor(output,'RGB2GRAY'); % grayscale
    output = cv.threshold(output,60,'MaxValue',255,'Type','Binary'); % threshold
    
    cnts = cv.findContours(output,'Mode','External','Method','Simple'); % detect all contours
    
    numcnts = numel(cnts);
    maxArea = [0,NaN];
    A = zeros(numcnts,2);
    if numcnts > 0
        A(1:numcnts,2) = (1:numcnts);
        for i = 1:numcnts
            A(i,1) = cv.contourArea(cnts{i});
        end
        A = sortrows(A,'descend');
        k = 1;
        circles = false;
        
        
        if ~isnan(A(1,2))
            %% Calculate the shape of the detected contour
            
            while ~circles && k < 5 && k <= length(A(:,1)) && A(k,1) > 100
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
                    if videofeed
                        for i = 1:length(approx)
                            img = cv.circle(img,4.*approx{i},3,'Color',[0,0,255],...
                                'Thickness',-1); % draws corners of shape
                        end
                        
                        img = cv.circle(img,4.*[cX,cY],7,'Color',[255,255,255],...
                            'Thickness',-1); % draws center of shape
                    end
                end
                k = k+1;
            end
        end
    end
    
    %     circles = cv.HoughCircles(output, 'Method','Gradient', 'DP',2, ...
    %         'MinDist',size(gray,1)/8, 'Param1',200, 'Param2',100, ...
    %         'MinRadius',0, 'MaxRadius',0);
    %     if numel(circles) < 20
    
    %% Create an image mask to only analyze color within the circles
    
    %         mask_template = single(zeros(length(img(:,1)),length(img(1,:,:)),3));    % initialize as an unsigned 8bit matrix
    %         minDist = [inf,NaN];    % initialize the distance vector
    %         detected_color = single(zeros(1,3));  % initialize the mean distance vector
    %
    %         for i=1:numel(circles)
    %             center = round(circles{i}(1:2));
    %             radius = round(circles{i}(3));
    %             mask = cv.circle(mask_template, center, radius, 'Color',[0,0,255],'Thickness',-1,'LineType','AA');
    %             mask = cv.erode(mask,'Iterations',2);
    %             mask = single(round(mask(:,:,3)./255));
    %             %         mask = logical(mask(:,:,3) > 1); % make the mask binary
    %             labarea = Lab.*mask; % apply the mask to the colored image
    %             for j = 1:3
    %                 temp = labarea(:,:,j);
    %                 detected_color(i,j) = mean(nonzeros(temp)); % average of each LAB color value within the area
    %             end
    %             d = sqrt(sum((colorlab-detected_color(i,:)).^2)); % Euclidian distance between detected color and desired color
    %             if d < minDist(1)
    %                 minDist = [d, i]; % detected object with the closest desired color
    %             end
    %         end
    %         if numel(circles) ~= 0 && minDist(1) < 100
    %             center = round(circles{minDist(2)}(1:2));
    %             radius = round(circles{minDist(2)}(3));
    %
    %             %%    Draw Circles
    %             if videofeed
    %             img = cv.circle(img, center, radius, 'Color',[0,0,255], ...
    %                 'Thickness',2, 'LineType','AA');        % draw the circle outline
    %             img = cv.circle(img, center, 3, 'Color',[255,0,0], ...
    %                 'Thickness','Filled', 'LineType','AA'); % draw the circle center
    %             imshow(uint8(img));    % print the image
    %             end
    %
    %             %%  Calculate Height, Angle, and Distance
    %
    %             delta_h = (origin(2)-center(2))./10;
    %             delta_x = (origin(1)-center(1))./10;
    %             distance = given_distance*given_radius/radius;
    %             theta = atand(distance/delta_x);
    %             fprintf('Height:%3.2f   Angle:%2.1f     Distance:%3.2f\n',delta_h,theta,distance); % print the calculated height and amount needed to turn
    %         else
    %             if videofeed
    %                 imshow(uint8(img));
    %             end
    %         end
    %
    %
    %     end
    
    if videofeed
        imshow(uint8(img));
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
        fprintf('fps: %2.2f x: %3.0f y: %3.0f\n',1/toc,cX,cY);
    else
    disp(1/toc);
    end
end