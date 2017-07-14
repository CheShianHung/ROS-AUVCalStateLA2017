clear;
clc;
%% Given Constants

given_radius = 8*49/8;
given_distance = 60;
color_choice = 6;       % integer; colors listed below
camdevice = 'image';   % 'webcam' 'image' 'usb'
videofeed = true;      % shows results
satthresh = 80;        % threshold sensitivity for saturation channel (0-255)
huethresh = 15;        % threshold sensitivity for hue channel (0-255)
scale = 6;              % image processing scaling
display = 1;            % display image scaling
corners = false;        % display shape corners
sigma = 0.33;

%% Initialize outputs
delta_h = zeros(1,15);
theta = zeros(1,15);
distance = zeros(1,15);


%% Colors

colors_list = { 'red',[255,0,0];        % 1
    'green',[25,123,76];      % 2
    'yellow',[199,204,120]      %3
    'pink',[255,102,102]
    'bouy',[101,240,127]
    'gate',[250,66,15]};

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
        img = img(31:400,71:654,:);
end

l = size(img,1); % length
w = size(img,2); % width
%%

origin = [l/2,w/2];    % Sets the origin coordinates
mask = logical([]);

m = 1;
n = 1;
while m < 60 && n < 30 && (60-m > 30-n)
    %% Processing
    tic;
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
    
    lines = cv.HoughLinesP(edged, 'Rho',1, 'Theta',pi/180, 'Threshold',50, ...
        'MinLineLength',50, 'MaxLineGap',20);
    
    %     output = cv.threshold(output,60,'MaxValue',255,'Type','Binary'); % threshold
    %
    %     cnts = cv.findContours(output,'Mode','External','Method','Simple'); % detect all contours
    %
    %     %% Arrange contours from largest to smallest
    numlines = numel(lines);
    %     maxArea = [0,NaN];
    A = zeros(numlines,2);
    linespresent = false;
    if numlines >= 4
        linespresent = true;
        A(1:numlines,2) = (1:numlines);
        for i = 1:numlines
            A(i,1) = sqrt((lines{i}(1)-lines{i}(3))^2+(lines{i}(2)-lines{i}(4))^2);
        end
        A = sortrows(A,'descend');
        x11 = lines{1}(1);
        y11 = lines{1}(2);
        x21 = lines{1}(3);
        y21 = lines{1}(4);
        x12 = lines{2}(1);
        y12 = lines{2}(2);
        x22 = lines{2}(3);
        y22 = lines{2}(4);
        %         center(1,1) = ((y22*x11+y11*x11)/(x22-x11)-(y12*x21+y21*x21)/(x12-x21)...
        %             +y21-y11)/((y22-y11)/(x22-x11)-(y12-y21)/(x12-x21));
        %         center(1,2) = (y22-y11)/(x22-x11)*(center(1,1)-x11)+y11;
        midpoint(1,1) = (x11+x21)/2;
        midpoint(1,2) = (y11+y21)/2;
        x11 = lines{3}(1);
        y11 = lines{3}(2);
        x21 = lines{3}(3);
        y21 = lines{3}(4);
        x12 = lines{4}(1);
        y12 = lines{4}(2);
        x22 = lines{4}(3);
        y22 = lines{4}(4);
        %         center(2,1) = ((y22*x11+y11*x11)/(x22-x11)-(y12*x21+y21*x21)/(x12-x21)...
        %             +y21-y11)/((y22-y11)/(x22-x11)-(y12-y21)/(x12-x21));
        %         center(2,2) = (y22-y11)/(x22-x11)*(center(2,1)-x11)+y11;
        midpoint(2,1) = (x11+21)/2;
        midpoint(2,2) = (y11+y21)/2;
        center(1,1) = (midpoint(1,1)+midpoint(2,1))/2;
        center(1,2) = (midpoint(1,2)+midpoint(2,2))/2;
        
        
   
    %
    %
    %        
                        if videofeed
                            for i = 1:4
                                for j = 0:2:2
                                    img = cv.circle(img,scale.*lines{i}(1+j:2+j),3,'Color',[0,0,255],...
                                        'Thickness',-1); % draws corners of shape
                                end
                            end
    %
                            img = cv.circle(img,scale.*center,7,'Color',[255,255,255],...
                                'Thickness',-1); % draws center of shape
    %
    %                         img = cv.circle(img,scale.*[cX,cY], scale.*radius, 'Color',[0,0,255], ...
    %                             'Thickness',2, 'LineType','AA');        % draw the circle outline
                        end
    %                 end
    %                 k = k+1;
    %             end
    %         end
        end
    %
    %
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
    %     t = toc;
    %     if 1/t > 10
    %         pause(.1-t);
    %     end
    %     if circles
    %         center = [cX,cY];
    %         delta_h(n) = (origin(2)-center(2))./10;
    %         delta_x = (origin(1)-center(1))./10;
    %         distance(n) = given_distance*given_radius/radius;
    %         theta(n) = atand(double(distance(n)/delta_x));
    % %         fprintf('Height:%3.2f   Angle:%2.1f     Distance:%3.2f  fps:%2.2f\n',delta_h(n),theta(n),distance(n),1/toc); % print the calculated height and amount needed to turn
    %         n = n + 1;
    %     else
    %         fprintf('fps:%2.2f\n',1/toc);
    %     end
    %     m = m + 1;
    % end
    % if n == 30
    %     fcdMsg.FrontCamVerticalDistance = mean(delta_h);
    %     fcdMsg.FrontCamHorizontalDistance = mean(theta);
    %     fcdMsg.FrontCamForwardDistance = mean(distance);
    %     fprintf('Height:%3.2f  Angle:%2.1f   Distance:%3.2f\n',fcdMsg.FrontCamVerticalDistance,fcdMsg.FrontCamHorizontalDistance,...
    %         fcdMsg.FrontCamForwardDistance);
    %     found = true;
    % else
    %     found = false;
    %     fprintf('Not found')
end
