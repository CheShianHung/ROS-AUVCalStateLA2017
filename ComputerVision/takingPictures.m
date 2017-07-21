%cam = videoinput('tisimaq_r2013', 1, 'RGB32 (744x480)');
cam = videoinput('linuxvideo', 1, 'RGB24_744x480');
cam.FramesPerTrigger = 1;

found = false;
camNum = 1; 
pause(180); 
while(~found & camNum < 100)
    start(cam); 
    frame = getdata(cam); 
    stop(cam); 
    imshow(frame); 
    filename = sprintf('photo%d.jpg', camNum); 
    imwrite(frame, filename); 
    camNum = camNum + 1;
end
    
