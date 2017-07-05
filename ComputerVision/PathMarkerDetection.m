clear;
clc;
%% Given Constants

given_radius = 8*49/8;
given_distance = 60;
color_choice = 6;   % integer from 1-4; colors listed below

%% Colors

colors_list = { 'red',[255,0,0];    %1
    'green',[0,100,0];      % 2
    'yellow',[255,255,0]    % 3
    'orange',[255,128,0]    % 4
    'blue',[0,0,255]        % 5
    'black',[0,0,0]};       % 6

%% Initialize OpenCV

if ispc
    addpath('C:\dev\mexopencv');
    addpath('C:\dev\mexopencv\opencv_contrib');
else
    addpath('~/cv/mexopencv');
    addpath('~/cv/mexopencv/opencv_contrib');
end

%% Initialize Color
color = single([]);
color(1,1,:) = colors_list{color_choice,2};
colorlab = cv.cvtColor(color./255,'RGB2Lab');     % convert color to LAB colorspace
colorlab = single(colorlab);
colorlab = permute(colorlab,[3 2 1])';

%% Camera initialization

% camera = cv.VideoCapture();
% pause(2);
% img = single(camera.read());

% img = which('shapes_and_colors.jpg');
% img = single(cv.imread(img, 'Flags',1));

camera = videoinput('tisimaq_r2013',1,'RGB24 (744x480)');
pause(2);
img = single(getsnapshot(camera));
img = img(30:400,70:654,:);

img = cv.resize(img,[300,350]);

%%

origin = [size(img,1)/2,size(img,2)/2];    % Sets the origin coordinates
mask_template = single(zeros(length(img(:,1)),length(img(1,:,:)),3));    % initialize as an unsigned 8bit matrix

while 1
    %% Processing
    
    %     tic;
    gray = cv.cvtColor(img, 'RGB2GRAY');    % convert to grayscale
    gray = cv.medianBlur(gray, 'KSize',5);  % blur grayscaled image
    thresh = cv.threshold(gray, 60,'MaxValue',255,'Type','Binary');
    lab = cv.medianBlur(img,'KSize',5);    % blur color image
    lab2 = cv.cvtColor(lab./255, 'RGB2Lab');
    % convert color image to LAB colorspace
    
    %% Contour Detection
    
    cnts = cv.findContours(thresh,'Mode','External','Method','Simple');
    
    %% Create an image mask to only analyze color within the rectangles
    
    
    minDist = [inf,NaN];    % initialize the distance vector
    detected_color = zeros(1,3);  % initialize the mean distance vector
    rectangles = 0;
    if numel(cnts) < 20
    for i=1:numel(cnts)
        cnt = cnts(i);
        M = cv.moments(cnt{1,1});
        cX = int16(M.m10/M.m00);
        cY = int16(M.m01/M.m00);
        peri = cv.arcLength(cnt{1,1},'Closed',1);
        approx = cv.approxPolyDP(cnt{1,1},'Epsilon',0.04*peri,'Closed',1);
        
        if length(approx) == 4 && numel(cnt{1}) > 20
            rectangles = 1;
            mask = cv.fillConvexPoly(mask_template,cnt{1,1},'Color',[0,0,255]);
            mask = cv.erode(mask,'Iterations',2);
            mask = round(mask(:,:,3)./255); % make the mask binary
            labarea = lab2.*mask; % apply the mask to the colored image
            for j = 1:3
                temp = labarea(:,:,j);
                detected_color(i,j) = mean(nonzeros(temp)); % average of each LAB color value within the area
            end
            d = sqrt(sum((colorlab-detected_color(i,:)).^2)); % Euclidian distance between detected color and desired color
            if d < minDist(1)
                minDist = [d, i]; % detected object with the closest desired color
            end
            
        end
    end
    end
    if rectangles
        cnt = cnts(minDist(2));
        M = cv.moments(cnt{1,1});
        center = [int16(M.m10/M.m00),int16(M.m01/M.m00)];
        
        %%    Draw
        
        % Draw the outline
        img = cv.fillConvexPoly(img,cnt{1,1},'Color',[255,255,255]);
        
        % Draw the center
        img = cv.circle(img, center, 3, 'Color',[255,0,0], ...
            'Thickness','Filled', 'LineType','AA');
        imshow(uint8(img));    % print the image
        
        
        
        %%  Calculate Height, Angle, and Distance
        
        delta_h = (origin(2)-center(2))./10;
        delta_x = double((origin(1)-center(1))./10);
        peri = cv.arcLength(cnt{1,1},'Closed',1);
        approx = cv.approxPolyDP(cnt{1,1},'Epsilon',0.04*peri,'Closed',1);
        radius = 0;
        for i = 1:4
            for j = 1:4
                dist = sqrt(approx{i}(1,1)^2+approx{j}(1,2)^2)/2;
                if dist > radius
                    radius = dist;
                end
            end
        end
                    
                distance = given_distance*given_radius/radius;
                theta = atand(distance/delta_x);
        fprintf('Height:%3.2f   Angle:%2.1f Distance:%2.1f\n',delta_h,theta,distance); % print the calculated height and amount needed to turn
    else
        imshow(uint8(img));
    end
    %     toc;
%     img = single(camera.read()); % initialize camera image for next loop

img = single(getsnapshot(camera));
img = img(30:400,70:654,:);

img = cv.resize(img,[300,350]);
    
end