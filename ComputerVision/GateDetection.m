clear;
clc;
%% Given Constants

given_radius = 8*49/8;
given_distance = 60;
color_choice = 6;       % integer; colors listed below
camdevice = 'image';   % 'webcam' 'image' 'usb'
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

colors_list = { 'red',[255,101,255]        % 1
    'green',[25,123,76]                    % 2
    'yellow',[199,204,120]                  % 3
    'pink',[255,102,102]
    'bouy',[101,240,127]
    '1st_gate',[173,83,31]
    '2nd_gate',[0,0,0]};

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
        img = which('gate2.jpg');
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
                        found = true;
                        %                         distance = given_distance*given_radius/radius;
                        %                         meandistance(n) = distance;
                        %                         fcdMsg.FrontCamForwardDistance = distance;
                        %                         distance = double(distance);
                        %                         delta_x = double(distance);
                        %                         meantheta(n) = atand(double(distance/delta_x));
%                         fprintf('Vertical:%3.2f Horizontal:%3.2f Left:%3.2f Right:%3.2f\n',delta_h,delta_x,lengthL,lengthR); % print the calculated height and amount needed to turn
                        
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
                        fcdMsg.FrontCamVerticalDistance = 999;

                        fcdMsg.FrontCamHorizontalDistance = delta_x;
                        found = true;

                        meandelta_x(n) = delta_x;
%                         fprintf('Horizontal:%3.2f Left:%3.2f Right:%3.2f\n',delta_x,lengthL,lengthR); % print the calculated height and amount needed to turn
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
%                     start(camera);
                    img = getsnapshot(camera);
%                     stop(camera);
                    hsv = rgb2hsv(img);
                    i = mean(mean(hsv(:,:,3)));
                    if i < threshL
                        while i < threshL
                            e1 = e1 + 1;
                            exposure = sprintf('v4l2-ctl -d /dev/video0 -c exposure_absolute=%d',e1);
                            system(exposure);
                            start(camera);
                            img = getsnapshot(camera);
                            stop(camera);
                            
                            hsv = rgb2hsv(img);
                            i = mean(mean(hsv(:,:,3)));
                        end
                    elseif i > threshH
                        while e1 > threshH
                            e1 = e1 - 1;
                            exposure = sprintf('v4l2-ctl -d /dev/video0 -c exposure_absolute=%d',e1);
                            system(exposure);
                            start(camera);
                            img = getsnapshot(camera);
                            stop(camera);
                            hsv = rgb2hsv(img);
                            i = mean(mean(hsv(:,:,3)));
                        end
                    end
                    img = img(1:480,161:650,:);
            end
            %             m = m+1;
            if ~found
                fcdMsg.FrontCamVerticalDistance = 999;
                fcdMsg.FrontCamHorizontalDistance = 999;
                fcdMsg.FrontCamForwardDistance = 999;
                fprintf('OBJECT LOST\n');
            end
            disp(fcdMsg.FrontCamHorizontalDistance);
            send(fcdPub, fcdMsg);
            cviMsg = receive(cviSub) ;
        end