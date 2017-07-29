client = tcpip('10.0.0.3',55000,'NetworkRole','Client');
set(client,'InputBufferSize',7688);
set(client,'Timeout',30);
fopen(client);

server = tcpip('0.0.0.0',55000,'NetworkRole','Server');
s = whos('data');
set(server,'OutputBufferSize',s.bytes);
fopen(server);

while 1
    data = fread(client,1,'double');
    data = data + 0.1;
    fwrite(server,data(:),'double');
    disp(data);
    pause(1);
end

fclose(client);