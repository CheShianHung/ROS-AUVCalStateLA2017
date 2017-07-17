%cam = videoinput('tisimaq_r2013', 1, 'RGB32 (744x480)');
cam = videoinput('linuxvideo', 1, 'RGB24_640x480');
cam.FramesPerTrigger = 1;

found = false;
camNum = 1; 
%pause(30); 
while(~found & camNum < 70)
    start(cam); 
    frame = getdata(cam); 
    stop(cam); 
    imshow(frame); 
    filename = sprintf('photo%d.jpg', camNum); 
    imwrite(frame, filename); 
    camNum = camNum + 1;
end
    
