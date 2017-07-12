client = tcpip('10.0.0.2', 55000, 'NetworkRole','Client');
set(client,'InputBufferSize',7688);
set(client,'Timeout',30);
fopen(client);

while 1
    data = fread(client,1,'double');
    disp(data);
    pause(1);
end

fclose(client);