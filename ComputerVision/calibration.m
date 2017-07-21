%% Video Capture demo
% We learn how to capture live stream from camera and display it, while
% adjusting basic video color properties.
%
% <http://docs.opencv.org/3.2.0/dd/d43/tutorial_py_video_display.html>
% <https://github.com/opencv/opencv/blob/3.2.0/samples/cpp/videocapture_starter.cpp>
%

function varargout = calibration()
% global h
delete(imaqfind);
color_choice = 6;
global scale;
scale = 1;              % image processing scaling ratio


colors_list = { 'red',[255,0,0];        % 1
    'green',[25,123,76];      % 2
    'yellow',[199,204,120];      %3
    'pink',[255,102,102];
    'bouy',[101,240,127];
    'gate',[185,181,57]};

color = uint8([]);
color(1,1,:) = colors_list{color_choice,2}; % pick color from RGB choices
colorHSV = rgb2hsv(color);     % convert color choice to LAB colorspace
color = colorHSV;


% create the UI
h = buildGUI(color);
if nargout > 1, varargout{1} = h; end

% main loop
% counter = 0;
% tID = tic();
while ishghandle(h.fig)
    % get frame-by-frame
%     tic;
    frame = getsnapshot(h.cap);
    if isempty(frame), break; end
    frame = frame(1:480,163:652,:);
    frame = imrotate(frame,180);
    
    
    
    
    %% Processing
    blur = imresize(cv.medianBlur(frame,'KSize',5),1/scale);    % blur color image
    HSV = rgb2hsv(blur);                     % convert color image to LAB colorspace
    HSV = uint8(HSV*255);
    
    colorthresh = zeros(1,2,2);
    colorthresh(:,:,1) = [h.slid(3).Value-(h.slid(1).Value/255),h.slid(3).Value+(h.slid(1).Value/255)];
    colorthresh(:,:,2) = [h.slid(4).Value-(h.slid(2).Value/255),h.slid(4).Value+(h.slid(2).Value/255)];
    colorthresh(:,:,1) = mod(colorthresh(:,:,1),1);
    colorthresh = uint8(colorthresh*255);
    
    lowerb = colorthresh(1,1,:); % lower bound
    upperb = colorthresh(1,2,:); % upper bound
    
    
    
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
    %%
    mask = logical(mask);
    
%     % FPS
%     counter = counter + 1;
%     mask = cv.putText(mask, sprintf('FPS = %.2f', counter/toc(tID)), ...
%         [10 20], 'FontFace','HersheyPlain', 'Color',[0 255 0]);
    
    % display
    set(h.img, 'CData',mask);
%     disp(1/toc);
    drawnow;
end
end

function onType(~,e,h)
% global h
%ONTYPE  Event handler for key press on figure

% handle keys
switch e.Key
    case {'q', 'escape'}
        close(h.fig);
        
    case {'c', 's', 'space', 'enter'}
        frame = h.cap.retrieve();
        filename = sprintf('capture_%s.jpg', datestr(now(),'yyyymmddTHHMMSS'));
        cv.imwrite(filename, frame);
        disp(['Saved ' filename]);
end
end

function onChange(~,~,h)
% global h
%ONCHANGE  Event handler for UI controls

% retrieve current values from UI controls
% and update corresponding properties
h.huethresh = round(get(h.slid(1), 'Value'));
h.satthresh   = round(get(h.slid(2), 'Value'));
h.hue = get(h.slid(3), 'Value');
h.saturation = get(h.slid(4), 'Value');
% h.cap.Saturation = round(get(h.slid(3), 'Value'));

% update UI
set(h.txt(1), 'String',sprintf('Hue T: %3d',h.huethresh));
set(h.txt(2), 'String',sprintf('Sat T: %3d',h.satthresh));
set(h.txt(3), 'String',sprintf('Hue: %1.3f',h.hue));
set(h.txt(4), 'String',sprintf('Sat: %1.3f',h.saturation));
% set(h.txt(3), 'String',sprintf('Saturation: %2d',h.cap.Saturation));
drawnow;
end

function h = buildGUI(color)
global scale
%BUILDGUI  Creates the UI

% setup video capture
cap = videoinput('linuxvideo',1,'RGB24_744x480');
        triggerconfig(cap,'manual');     % speeds up image acquisition for videoinput
        start(cap);
pause(1);
% assert(cap.isOpened());

% video settings
frame = getsnapshot(cap);
assert(~isempty(frame));
frame = frame(1:480,163:652,:);
frame = imrotate(frame,180);
% frame = imresize(frame,1/4);
sz = size(frame);
% fourcc = char(typecast(int32(cap.FourCC), 'uint8'));
% frame = logical(frame);
frame = imresize(frame,1/scale);
szm = size(frame);
mask = false(szm(1:2));

% h = h;
% build the user interface (no resizing to keep it simple)
% h = struct();
h.hue = color(:,:,1);
h.saturation = color(:,:,2);
h.satthresh = 80;        % threshold sensitivity for saturation channel (0-255)
h.huethresh = 20;        % threshold sensitivity for hue channel (0-255)
h.cap = cap;
h.fig = figure('Name',sprintf('Video Capture: %dx%d', sz(2), sz(1)), ...
    'NumberTitle','off', 'Menubar','none', 'Resize','off', ...
    'Position',[200 200 sz(2) sz(1)+80-1]);
if ~mexopencv.isOctave()
    %HACK: not implemented in Octave
    movegui(h.fig, 'center');
end
h.ax = axes('Parent',h.fig, 'Units','pixels', 'Position',[1 80 sz(2) sz(1)]);
if ~mexopencv.isOctave()
    h.img = imshow(mask, 'Parent',h.ax);
else
    %HACK: https://savannah.gnu.org/bugs/index.php?45473
    axes(h.ax);
    h.img = imshow(mask);
end
h.txt(1) = uicontrol('Parent',h.fig, 'Style','text', ...
    'Position',[5 5 130 20], 'FontSize',11, ...
    'String',sprintf('Hue T: %3d',h.huethresh));
h.txt(2) = uicontrol('Parent',h.fig, 'Style','text', ...
    'Position',[5 30 130 20], 'FontSize',11, ...
    'String',sprintf('Sat T: %3d',h.satthresh));
h.txt(3) = uicontrol('Parent',h.fig, 'Style','text', ...
    'Position',[5 55 130 20], 'FontSize',11, ...
    'String',sprintf('Hue: %1.3f',h.hue));
h.txt(4) = uicontrol('Parent',h.fig, 'Style','text', ...
    'Position',[5 80 130 20], 'FontSize',11, ...
    'String',sprintf('Sat: %1.3f',h.saturation));
h.slid(1) = uicontrol('Parent',h.fig, 'Style','slider', ...
    'Position',[135 5 sz(2)-135-5 20], 'Value',h.huethresh, ...
    'Min',0, 'Max',255, 'SliderStep',[1 10]./(255-0));
h.slid(2) = uicontrol('Parent',h.fig, 'Style','slider', ...
    'Position',[135 30 sz(2)-135-5 20], 'Value',h.satthresh, ...
    'Min',0, 'Max',255, 'SliderStep',[1 10]./(255-0));
h.slid(3) = uicontrol('Parent',h.fig, 'Style','slider', ...
    'Position',[135 55 sz(2)-135-5 20], 'Value',h.hue, ...
    'Min',0, 'Max',1, 'SliderStep',[1 10]./(1000-0));
h.slid(4) = uicontrol('Parent',h.fig, 'Style','slider', ...
    'Position',[135 80 sz(2)-135-5 20], 'Value',h.saturation, ...
    'Min',0, 'Max',1, 'SliderStep',[1 10]./(1000-0));

% hook event handlers
opts = {'Interruptible','off', 'BusyAction','cancel'};
set(h.slid, 'Callback',{@onChange,h}, opts{:});
set(h.fig, 'WindowKeyPressFcn',{@onType,h}, opts{:});
end