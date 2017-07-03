clear;
clc;
%% Given Constants

given_radius = 8*49/8;
given_distance = 60;
color_choice = 4;   % integer from 1-3; colors listed below

%% Colors

colors_list = { 'red',[255,0,0];        % 1
                'green',[0,100,0];      % 2
                'yellow',[255,255,0]
                'pink',[255,102,102]};  % 3

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
colorlab =cv.cvtColor(color./255,'RGB2Lab');     % convert color to LAB colorspace
colorlab = permute(colorlab,[3 2 1])';

%% Camera initialization
% camera = cv.VideoCapture(0);
% pause(2);
% img = single(camera.read());

camera = videoinput('tisimaq_r2013',1,'RGB24 (744x480)');
pause(2);
img = single(getsnapshot(camera));
img = img(30:400,70:654,:);

img = cv.resize(img,[300,350]);
%%

origin = [size(img,1)/2,size(img,2)/2];    % Sets the origin coordinates

while 1
    %% Processing
    
    gray = cv.cvtColor(img, 'RGB2GRAY');    % convert to grayscale
    gray = cv.medianBlur(gray, 'KSize',5);  % blur grayscaled image
    lab = cv.medianBlur(img,'KSize',5);    % blur color image
    lab2 = cv.cvtColor(lab./255, 'RGB2Lab');                     % convert color image to LAB colorspace
    
    %% HoughCircles
    % detect all circles in the image
    
    circles = cv.HoughCircles(gray, 'Method','Gradient', 'DP',2, ...
        'MinDist',size(gray,1)/8, 'Param1',200, 'Param2',100, ...
        'MinRadius',0, 'MaxRadius',0);
    if numel(circles) < 20
    
    %% Create an image mask to only analyze color within the circles
    
    mask_template = single(zeros(length(img(:,1)),length(img(1,:,:)),3));    % initialize as an unsigned 8bit matrix
    minDist = [inf,NaN];    % initialize the distance vector
    detected_color = single(zeros(1,3));  % initialize the mean distance vector
    for i=1:numel(circles)
        center = round(circles{i}(1:2));
        radius = round(circles{i}(3));
        mask = cv.circle(mask_template, center, radius, 'Color',[0,0,255],'Thickness',-1,'LineType','AA');
        mask = cv.erode(mask,'Iterations',2);
        mask = single(round(mask(:,:,3)./255));
%         mask = logical(mask(:,:,3) > 1); % make the mask binary
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
    if numel(circles) ~= 0 && minDist(1) < 100
        center = round(circles{minDist(2)}(1:2));
        radius = round(circles{minDist(2)}(3));
        
        %%    Draw Circles
        
                img = cv.circle(img, center, radius, 'Color',[0,0,255], ...
                    'Thickness',2, 'LineType','AA');        % draw the circle outline
                img = cv.circle(img, center, 3, 'Color',[255,0,0], ...
                    'Thickness','Filled', 'LineType','AA'); % draw the circle center
                imshow(uint8(img));    % print the image
        
        %%  Calculate Height, Angle, and Distance
        
        delta_h = (origin(2)-center(2))./10;
        delta_x = (origin(1)-center(1))./10;
        distance = given_distance*given_radius/radius;
        theta = atand(distance/delta_x);
        fprintf('Height:%3.2f   Angle:%2.1f     Distance:%3.2f\n',delta_h,theta,distance); % print the calculated height and amount needed to turn
            else
                imshow(uint8(img));
    end
        
%        pause(0.1) 
%     img = single(camera.read()); % initialize camera image for next loop
% tic;
    end
img = single(getsnapshot(camera)); 
img = img(30:400,70:654,:);

img = cv.resize(img,[300,350]);
% toc;

end