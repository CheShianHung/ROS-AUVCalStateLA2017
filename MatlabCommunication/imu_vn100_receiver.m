%rosinit()

client = tcpip('10.0.0.2',55000,'NetworkRole','Client');
set(client,'InputBufferSize',7688);
set(client,'Timeout',30);
fopen(client);

rotationPub = rospublisher('/current_rotation','auv_cal_state_la_2017/Rotation');
rotation = rosmessage('auv_cal_state_la_2017/Rotation');

while 1
    rotation.Yaw = fread(client,1,'float');
    rotation.Pitch = fread(client,1,'float');
    rotation.Roll = fread(client,1,'float');
    disp(rotation);
    send(rotationPub, rotation);
    
    pause(0.1);
end

fclose(server);

%rosshutdown