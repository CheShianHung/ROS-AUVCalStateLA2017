data = 1;

server = tcpip('0.0.0.0',55000,'NetworkRole','Server');
s = whos('data');
set(server,'OutputBufferSize',s.bytes);
fopen(server);

client = tcpip('10.0.0.1',55000,'NetworkRole','Client');
set(client,'InputBufferSize',7688);
set(client,'Timeout',30);
fopen(client);

while data < 100 
    fwrite(server,data(:),'double');
    data = fread(client,1,'double');
    disp(data);
    pause(1);
end

fclose(server);