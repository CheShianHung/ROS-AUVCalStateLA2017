data = 1;
server = tcpip('0.0.0.0',55000,'NetworkRole','Server');
s = whos('data');
set(server,'OutputBufferSize',s.bytes);
fopen(server);

while data < 100 
    fwrite(server,data(:),'double');
    data = data + 0.01;
    pause(1);
end

fclose(server);