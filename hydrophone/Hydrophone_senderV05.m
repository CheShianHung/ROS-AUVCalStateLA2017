    %Program Uses NI 9222 DATA Acquisition to detect hydrophone signals 
%V05- changed from IIR Butterworth filter to FIR Equiripple filter with 100 order.
%added left/right detection.
clc, clear all, close all

% server = tcpip('0.0.0.0',55000,'NetworkRole','Server');
% set(server,'OutputBufferSize',16);
% fopen(server);
% disp("server is open");

% client = tcpip('0.0.0.1',55000, 'NetworkRole', 'Client');
% set(server,'OutputBufferSize', 16);
% set(client, "Timeout", 30);
% fopen(client);
% disp("client open");

s = daq.createSession('ni');
d = daq.getDevices();
disp("daq session started");

peak = zeros(500,3);
% peak12 = zeros(500,3);
% peak23 = zeros(500,3);
% peak13 = zeros(500,3);
p1 = 1;
p2 = 1;
p3 = 1;
found1 = 0;
found2 = 0;
found3 = 0;
maxNoise = 0;
count = 0;
startValue = 0;
pos1 = 0;
pos2 = 0;
pos3 = 0;
pos12 = 0;
pos23 = 0;




% MATLAB Code
% Generated by MATLAB(R) 9.2 and the Signal Processing Toolbox 7.4.
% Generated on: 06-Jul-2017 17:40:20

% Equiripple Bandpass filter designed using the FIRPM function.

% All frequency values are in Hz.
Fs = 400000;  % Sampling Frequency

N      = 101;    % Order
Fstop1 = 18000;  % First Stopband Frequency
Fpass1 = 24900;  % First Passband Frequency
Fpass2 = 25100;  % Second Passband Frequency
Fstop2 = 42000;  % Second Stopband Frequency
Wstop1 = 1;      % First Stopband Weight
Wpass  = 1;      % Passband Weight
Wstop2 = 1;      % Second Stopband Weight
dens   = 20;     % Density Factor

% Calculate the coefficients using the FIRPM function.
b  = firpm(N, [0 Fstop1 Fpass1 Fpass2 Fstop2 Fs/2]/(Fs/2), [0 0 1 1 0 ...
           0], [Wstop1 Wpass Wstop2], {dens});
Hd = dfilt.dffir(b);


h1 = addAnalogInputChannel(s,'cDAQ1Mod1', 'ai1', 'Voltage');
h2 = addAnalogInputChannel(s,'cDAQ1Mod1', 'ai2', 'Voltage');
h3 = addAnalogInputChannel(s,'cDAQ1Mod1', 'ai3', 'Voltage');
s.Rate = 400000; %sampling rate
s.DurationInSeconds = 2;

disp("ready to loop");

%blue = 1, red = 2, yellow = 3
x = 0;
num = 1;
while (1)
    used = 0;
error = 0;
p1 = 1;
p2 = 1;
p3 = 1;
found1 = 0;
found2 = 0;
found3 = 0;
maxNoise = 0;
%determine the value to differentiate signal from noise
count = 0;
data = startForeground(s); 
disp("data collected");
f = filter(Hd, data);
%f = data;
for i = 1:3
    if(mean(f(:,i)) > 0)
        
            f(:,i) = f(:,i) - mean(f(:,i));
    else
            f(:,i) = f(:,i) + mean(f(:,i));
   end
end


f1 = f(:,1);
f2 = f(:,2);
f3 = f(:,3);


data1 = data(:,1);
data2 = data(:,2);
data3 = data(:,3);

%r = xcorr(f1, f3);
%delay = finddelay(f2,f);
%subplot(1,2,1);
plot(f);
%axis([0 4 -0.5 0.5]);
%subplot(1,2,2);
%plot (f30k);
%disp(delay);
%[newF1, newF2] = alignsignals(f1, f2);
%
count = 0;
%blue = 1, red = 2, yellow = 3
for i = 1000:2000
    if(count > 100)
        startValue = maxNoise * 20;
        i = 2000;
    end
    if (abs(f1(i) - f1(i-1)) < f1(i) * 10)
        count = count + 1;
        if(abs(f1(i)) > maxNoise)
            maxNoise = abs(f1(i));
            disp(maxNoise);
        end
    end
    
end
disp(startValue);

%determine which microphone receives signal 1st, 2nd, 3rd.(good for left or
%right);
for i = 2000:800000
    if(abs(f1(i)) > startValue && found1 == 0)
        pos1 = i;
        volt1 = f1(i);
        found1 = 1;
    end
    if(abs(f2(i)) > startValue && found2 == 0)
        pos2 = i;
        volt2 = f2(i);
        found2 = 1;
    end
    if( abs(f3(i)) > startValue && found3 == 0)
        pos3 = i;
        volt3 = f3(i);
        found3 = 1;
    end
    
end
pos = [pos1, pos2, pos3];
disp(pos);
pos12 = pos1 - pos2;
pos23 = pos2 - pos3;
%disp(pos12);
%disp(pos23);
%store time stamp of all peaks in 3 signals
j = pos1;
if (j+100 < size(f1,1) || j+100 < size(f2,1) || j+100 < size(f3,1))

for j = (pos1 + 100):(pos1 + 4000)
    if(f1(j) > f1(j+1) && f1(j) > f1(j-1))
        peak(p1,1) = j;
        p1 = p1 + 1;
    end
    if(f2(j) > f2(j+1) && f2(j) > f2(j-1))
        peak(p2,2) = j;
        p2 = p2 + 1;
    end
    if(f3(j) > f3(j+1) && f3(j) > f3(j-1))
        peak(p3,3) = j;
        p3 = p3 + 1;
    end
    
    if (j+2 > size(f1,1) || j+2 > size(f2,1) || j+2 > size(f3,1))
        error = 1;
        break;
    end
    

    
end
end
% %calculate time difference between peaks of signal 1 and 2
% for j = 2:240
%    peak12(j,1) = peak(j,1) - peak(j-1,2);
%    peak12(j,2) = peak(j,1) - peak(j, 2);
%    peak12(j,3) = peak(j,1) - peak(j+1, 2);
%    
%    peak23(j,1) = peak(j,2) - peak(j-1,3);
%    peak23(j,2) = peak(j,2) - peak(j,3);
%    peak23(j,3) = peak(j,2) - peak(j+1,3);
%    
%    peak13(j,1) = peak(j,1) - peak(j-1, 3);
%    peak13(j,2) = peak(j,1) - peak(j, 3);
%    peak13(j,3) = peak(j,1) - peak(j+1, 3);
% end

%f1 = middle, f2 = left, f3 = right
%red = 1, blue = 2, yellow = 3;
mid = pos1;
left = pos2;
right = pos3;
if (left > mid && mid > right)
    disp("right");
    direction = 5;
end
if (right > mid && mid > left)
    disp("left");
    direction = 1;
end
if (mid >= left && mid >= right || mid <= left && mid <= right)
    disp("middle");
    direction = 3;
end

% if (direction == 1 && pos23 < 0 && pos23 > -5)
%     direction = 2;
% end
% disp(direction);
% if (direction == 5 && pos12 > 0 && pos12 < 5 )
%     direction = 4;
% end
% if (abs(pos1-pos2) <= 5 || abs(pos2-pos3) <= 5)
%     disp("almost middle");
%     direction = 1;
% end
freq = zeros(49, 1);

for i = 2:50

freq(i-1) = peak(i, 1) - peak(i-1, 1);
    
end

frequency = 1 / (mean(freq) / 400000);
disp(frequency);


angle12 = acosd(pos12 * .0000025 * 1484 / 0.04); 
angle23 = acosd(pos23 * .0000025 * 1484 / 0.04);

x = x + 1;
%disp(x);

angle = (angle12 + angle23) / 2;
angle = angle - 90;
if (angle > 360 || angle < -360)
    angle = -999;
end
%disp(angle);
disp(direction);


if (error == 1)
    direction = 10;
    angle = -999;
end

%  filename = sprintf('1stPool%d', num); 
%  save(filename,'data', 'direction', 'angle', 'frequency');
%  num = num + 1;

% fwrite(server, direction(:), 'double');
% fwrite(server, angle(:), 'double');
% used = fread(client, 1, 'double');

end

fclose(server);