%rosinit();

clear;
clc;

data = [0,0];
client = tcpip('11.0.0.2',55000,'NetworkRole','Client');
set(client,'InputBufferSize', 16);
set(client,'Timeout',60);
fopen(client);
disp("Client open");

hPub = rospublisher('/hydrophone','auv_cal_state_la_2017/Hydrophone');
h = rosmessage('auv_cal_state_la_2017/Hydrophone');

while 1
%     data = fread(client,3, 'double');
    data = fread(client,1,'int32');
    disp(data);
    h.Direction = data;
    h.Angle = 999;
    send(hPub, h);
    pause(3);
end

fclose(client);