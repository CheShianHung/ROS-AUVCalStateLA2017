clear;
clc;

hyPub = rospublisher('/hydrophone','auv_cal_state_la_2017/Hydrophone');
hyMsg = rosmessage('auv_cal_state_la_2017/Hydrophone');

rate = rosrate(10);

while 1
    hyMsg.Direction = 9;
    hyMsg.Angle = 55.5;
    
    send(hyPub, hyMsg);
    
    waitfor(rate);
end