addpath('~/cv/mexopencv');
addpath('~/cv/mexopencv/opencv_contrib');
delete(imaqfind);
camera = videoinput('linuxvideo', 1, 'RGB24_744x480');
triggerconfig(camera, 'manual');
start(camera);
system('v4l2-ctl -c brightness=120');
system('v4l2-ctl -c gain=30');
system('v4l2-ctl -c exposure_absolute=10');
%pause(180);

pub = rospublisher('/take_picture_status','std_msgs/Int32');
sub = rossubscriber('/take_picture');
pubMsg = rosmessage('std_msgs/Int32');
rate = rosrate(10);

pubMsg.Data = 1;
send(pub, pubMsg);
msg.Data = 0;
while msg.Data ~= 1
    pubMsg.Data = 1;
    send(pub, pubMsg);
    msg = receive(sub) ;
    disp(msg.Data);
    waitfor(rate);
end

camNum = 1;
while(true)
        pubMsg.Data = 3;
        send(pub, pubMsg);
%         exposure = sprintf('v4l2-ctl -c exposure_absolute=%d',i);
%         system(exposure);
        frame = getsnapshot(camera);
        frame = imrotate(frame,180);
        filename = sprintf('secondRun/secondRun%d.jpg', camNum);
        imwrite(frame, filename);
        camNum = camNum + 1;
        pause(.25);
end