clear;
clc;
%% Given Constants

given_radius = 8*49/8;
given_distance = 60;
color_choice = 6;       % integer; colors listed below
camdevice = 'webcam';   % 'webcam' 'image' 'usb'
videofeed = true;      % shows results
satthresh = 65;        % threshold sensitivity for saturation channel (0-255)
huethresh = 20;        % threshold sensitivity for hue channel (0-255)
scale = 1;              % image processing scaling
display = 1;            % display image scaling
corners = true;        % display shape corners
sigma = 0.33;
contours = true;
linethresh = 10;
topthresh = 1;
xthresh = 90;
d = 100;

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
    'gate',[42,84,66]
    'bluue',[56,86,109]
    'greeen',[81,162,135]};

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
    %     blur = cv.medianBlur(edged,'KSize',5);
    
    
    lines = cv.HoughLinesP(edged, 'Rho',1, 'Theta',pi/180, 'Threshold',50, ...
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
    if length(nonzeros(A(:,1))) > 1 && length(nonzeros(A(:,3))) > 1 &&...
            length(nonzeros(A(:,5))) > 1
        left = [mean(A(1:a,1)),mean(A(1:a,2))];
        right = [mean(A(1:b,3)),mean(A(1:b,4))];
        top = [mean(A(1:c,5)),mean(A(1:c,6))];
    end
    if left(2) > right(2)
        temp = left;
        left = right;
        right = temp;
    end
    if (~isnan(left(1)) && ~isnan(right(1)) && ~isnan(top(1))) &&...
            logical(left(1)*right(1)*top(1))
        middle = [(left(1)+right(1))/2,(left(2)+right(2))/2];
        %         tX = (middle(1)*middle(2)+top(2))/(middle(1)-top(1));
        %         dX = d*cos(atan(middle(1)));
        tY = ((top(2)/top(1))+middle(2))/(inv(top(1))-inv(middle(1)));
        if isinf(middle(1))
            dY = -d;
        else
            dY = -abs(d*sin(atan(middle(1))));
        end
        cY = tY - dY;
        cX = (cY/middle(1))+middle(2);
        %         cX = tX-dX;
        %         cY = middle(1)*(cX-middle(2));
        img = cv.circle(img,scale.*[cX,cY],4,'Color',[255,255,255],...
            'Thickness',-1);
    end
    
    
    %     end
    %     imshow(img);
    %     [cnts,hierarchy] = cv.findContours(edged,'Mode','CComp','Method','None'); % detect all contours
    
    %     output = cv.threshold(output,60,'MaxValue',255,'Type','Binary'); % threshold
    %
    %     cnts = cv.findContours(output,'Mode','External','Method','Simple'); % detect all contours
    %
    %     %% Arrange contours from largest to smallest
    %% Arrange contours from largest to smallest
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    %
    %
    %
    %     numcnts = numel(cnts);
    %     %     maxArea = [0,NaN];
    %     A = zeros(numcnts,2);
    %     linesdetected = 0;
    %     if numcnts > 0
    %         A(1:numcnts,2) = (1:numcnts);
    %         for i = 1:numcnts
    %             A(i,1) = cv.contourArea(cnts{i});
    %         end
    %         A = sortrows(A,'descend');
    %         k = 1;
    %
    %
    %         if ~isnan(A(1,2))
    %             %% Calculate the shape of the detected contour
    %
    %             while k <= 1 %length(A(:,1))
    %                 c = A(k,2);             % index of contour with largest area
    %                 cnt = cnts{c};
    %                 M = cv.moments(cnt);
    %                 cX = int16(M.m10/M.m00);
    %                 cY = int16(M.m01/M.m00);
    %                 peri = cv.arcLength(cnt,'Closed',1);
    %                 approx = cv.approxPolyDP(cnt,'Epsilon',...
    %                     0.04*peri,'Closed',1); % approximate the corners of the shape
    %                 if true%length(approx) == 2
    % %                     linesdetected = linesdetected + 1;
    %                     %                     [~,radius] =  cv.minEnclosingCircle(cnt);
    %                     if videofeed
    %                         if corners
    %                             for i = 1:length(approx)
    %                                 img = cv.circle(img,scale.*approx{i},3,'Color',[0,0,255],...
    %                                     'Thickness',-1); % draws corners of shape
    %                             end
    %                         end
    %                         if contours
    %                             for i = 1:length(cnt)
    %                                 img = cv.circle(img,scale.*cnt{i},1,'Color',[255,0,0],...
    %                                     'Thickness',-1);
    %                             end
    %                         end
    %
    %                         img = cv.circle(img,scale.*[cX,cY],7,'Color',[255,255,255],...
    %                             'Thickness',-1); % draws center of shape
    %
    %                         %                         img = cv.circle(img,scale.*[cX,cY], scale.*radius, 'Color',[0,0,255], ...
    %                         %                             'Thickness',2, 'LineType','AA');        % draw the circle outline
    %                     end
    %                 end
    %                 k = k+1;
    %             end
    %         end
    %     end
    %
    %
    %
    %
    %
    %
    if videofeed
        imshow(imresize(img,1/display));
    end
    
    
%         pause(2);
    
    
    
    switch camdevice
        case 'webcam'
            img = camera.read(); % initialize camera image for next loop
        case 'image'
            break
        otherwise
            img = getsnapshot(camera);
            img = img(31:400,71:654,:);
    end
    %     %     t = toc;
    %     %     if 1/t > 10
    %     %         pause(.1-t);
    %     %     end
    %     %     if circles
    %     %         center = [cX,cY];
    %     %         delta_h(n) = (origin(2)-center(2))./10;
    %     %         delta_x = (origin(1)-center(1))./10;
    %     %         distance(n) = given_distance*given_radius/radius;
    %     %         theta(n) = atand(double(distance(n)/delta_x));
    %     % %         fprintf('Height:%3.2f   Angle:%2.1f     Distance:%3.2f  fps:%2.2f\n',delta_h(n),theta(n),distance(n),1/toc); % print the calculated height and amount needed to turn
    %     %         n = n + 1;
    %     %     else
    %     %         fprintf('fps:%2.2f\n',1/toc);
    %     %     end
    % %         m = m + 1;
    %     % end
    %     % if n == 30
    %     %     fcdMsg.FrontCamVerticalDistance = mean(delta_h);
    %     %     fcdMsg.FrontCamHorizontalDistance = mean(theta);
    %     %     fcdMsg.FrontCamForwardDistance = mean(distance);
    %     %     fprintf('Height:%3.2f  Angle:%2.1f   Distance:%3.2f\n',fcdMsg.FrontCamVerticalDistance,fcdMsg.FrontCamHorizontalDistance,...
    %     %         fcdMsg.FrontCamForwardDistance);
    %     %     found = true;
    %     % else
    %     %     found = false;
    %     %     fprintf('Not found')
end
