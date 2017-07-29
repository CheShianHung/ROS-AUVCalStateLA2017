NET.addAssembly('C:\Users\lattepanda\Desktop\vnproglib-1.1\net\bin\win32\VectorNav.dll');
import VectorNav.Sensor.*;
import VectorNav.Protocol.Uart.*;

rotation = 0;

server = tcpip('0.0.0.0',55000,'NetworkRole','Server');
s = whos('rotation');
ser(server,'OutputBufferSize',s.bytes);
fopen(server);

%ez = EzAsyncData.Connect('COM7',115200);
while 1
    %disp(ez.CurrentData.YawPitchRoll);
    
    rotation = ez.CurrentData.YawPitchRoll.R;
    fwrite(server,rotation,'float');
    rotation = ez.CurrentData.YawPitchRoll.G;
    fwrite(server,rotation,'float');
    rotation = ez.CurrentData.YawPitchRoll.B;
    fwrite(server,rotation,'float');
    
    pause(0.1);
end
%ez.Disconnect();
    