%initializing the video object
%vid = videoinput('tisimaq_r2013', 1, 'RGB32 (744x480)');
cam = webcam('DFK 22AUC03'); 

%tracking an object
blob = vision.BlobAnalysis(...
'CentroidOutputPort', true,'AreaOutputPort', true, ...
'BoundingBoxOutputPort', true, ...
'MajorAxisLengthOutputPort', true,...
'MinorAxisLengthOutputPort', true,...
'OrientationOutputPort', true,...
'MinimumBlobArea', 400, 'ExcludeBorderBlobs', true);

%shape inserting object
shapeInserter = vision.ShapeInserter('BorderColor', 'White');	

found = true; 

%main loop
while(found)
	
	%take picture
	
    frame = snapshot(cam);

	%create undistorted image
    %only after camera is calibrated
	%[udImg, newOrigin] = undistortImage(frame, cameraParams, 'OutputView', 'full');
	
	%find saturation channel and filter
	imHSV = rgb2hsv(frame); 
	saturation = imHSV(:,:,3);
	satImg = medfilt2(saturation);
	satImg = im2bw(satImg, 0.9);
	
	%implement blob analysis
	[areas, centroid, bboxs, major, minor, orientation] = step(blob, satImg);
	
	%check if any blobs are found
	if(~isempty(bboxs))
		
		%sort through the blobs and obtain the largest
		[~, idx] = sort(areas, 'Descend');
		%bbox = double(bboxs(idx(1), :)); 

		%adjust for coordinate system shift caused by undistortImage
		%bbox(:, 1:2) = bsxfun(@plus, bbox(:, 1:2), newOrigin); 

		%convert bbox back to int32 type for shapeInserter to work
		%bbox = int32(bbox);
		
		%%%
		bbox = bboxs(idx(1), :); 
		%%%
		
		%%Compute rotation and translation of the camera
		%imgActualCamPosition = imread(fullfile('C:\', 'Users', 'aldo9', 'Documents', ...
        %'Water Robot', 'CalibrationFotos', sprintf('photo11.jpg')));
		%[imagePoints, boardSize] = detectCheckerboardPoints(imgActualCamPosition);
		%[R, t] = extrinsics(imagePoints, worldPoints, cameraParams);
		
		%%Measure the length of the box
		%Get the top-left and the top-right corners.
		box1 = double(bbox);
		imagePoints1 = [box1(1:2); ...
                       box1(1), box1(2) - box1(4)];
				
		% Get the world coordinates of imagePoints1
		%worldPoints1 = pointsToWorld(cameraParams, R, t, imagePoints1);
		centroid = centroid(idx(1), :);
		orientation = orientation(idx(1)); 
		
		%length = worldPoints1(3) - worldPoints1(4);
        length = imagePoints1(3) - imagePoints1(4); 
		amLength = length/25.4;
		
		%create a line to show orientation. Line is centered at center of rectangle
		x = [centroid,...
		centroid(1)+100*(cos(orientation))...
		centroid(2)+100*(-sin(orientation))];
		
		%convert radiants to degrees
		degO = ((orientation/(pi))*180); 
		
		%find small angle
		if degO > 0
			smallAngle = 90 - degO; 
		else
			smallAngle = -(90 + degO); 
		end
		
		%measure the distance of the camera to the box
		%%%
		%get the orientation and location of the camera with respect to the picture
		%only after camera calibration
        %orientation  = R';
		%location = -t*orientation; 
		
		%%calculate the x and y components
		imageCent = centroid; 
		%worldCent = pointsToWorld(cameraParams, R, t, imageCent);    %convert the centroid to world points
		%dist_x  = location(1) - worldCent(1); 
        dist_x = imageCent(1) - size(frame, 1)/2;
		amXDist = dist_x/25.4; 
		dist_y = imageCent(2) - size(frame, 2)/2; 
		amYDist = dist_y/25.4; 
		
		%%create text
		%position of text values
		position = [70, 20];                         %angle
		position1 = [70, 50];						 %length of square
		position2 = [size(frame, 1) - 30, 20]; 		 %x distance
		position3 = [size(frame, 1) - 30, 50]; 		 %y distance
		
		%position of text for labels
		position4 = [5, 20];							%angle
		position5 = [5, 50]; 							%length of square
		position6 = [size(frame, 1) - 80, 20];          %x distance
		position7 = [size(frame, 1) - 80, 50];          %y distance
		
		%convert numbers to string
		strLength = num2str(amLength); 
		strAngle = num2str(smallAngle);
		strAmXDist = num2str(amXDist); 
		strAmYDist = num2str(amYDist);
		
		%create labels
		angleLabel = 'Angle:';
		lengthLabel = 'Length:'; 
		x_dist = 'X Dist:';
		y_dist = 'Y Dist:';
		
		%insert text for values
		img = insertText(frame, position, strAngle); 
		img = insertText(img, position1, strLength); 
		img = insertText(img, position2, strAmXDist); 
		img = insertText(img, position3, strAmYDist);
		
		%insert text for labels
		img = insertText(img, position4, angleLabel); 
		img = insertText(img, position5, lengthLabel); 
		img = insertText(img, position6, x_dist); 
		img = insertText(img, position7, y_dist);
		
		%insert shape and box
		img = insertShape(img,'line', x, 'Color', 'red');
		img = step(shapeInserter, img, bbox);
		imshow(img); 
		clear img;
    end
    clear frame;  
	found = true; 
end