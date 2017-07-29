peak = zeros(500,3);
peak12 = zeros(500,3);
peak23 = zeros(500,3);
peak13 = zeros(500,3);
p1 = 1;
p2 = 1;
p3 = 1;
found1 = 0;
found2 = 0;
found3 = 0;
%determine the value to differentiate signal from noise
count = 0;
for i = 1000:2000
    if(count > 100)
        startValue = abs(f1(i) * 300);
        i = 2000;
    end
    if (abs(f1(i) - f1(i-1)) < f1(i) * 10)
        count = count + 1;
    end
    
end

%determine which microphone receives signal 1st, 2nd, 3rd.(good for left or
%right);
for i = 2000:400000
    if(abs(f1(i)) > startValue && found1 == 0)
        pos1 = i;
        found1 = 1;
    end
    if(abs(f2(i)) > startValue && found2 == 0)
        pos2 = i;
        found2 = 1;
    end
    if( abs(f3(i)) > startValue && found3 == 0)
        pos3 = i;
        found3 = 1;
    end
    
end
%store time stamp of all peaks in 3 signals
for j = (pos1 + 200):(pos1 + 4200)
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
    
end
%calculate time difference between peaks of signal 1 and 2
for j = 2:240
   peak12(j,1) = peak(j,1) - peak(j-1,2);
   peak12(j,2) = peak(j,1) - peak(j, 2);
   peak12(j,3) = peak(j,1) - peak(j+1, 2);
   
   peak23(j,1) = peak(j,2) - peak(j-1,3);
   peak23(j,2) = peak(j,2) - peak(j,3);
   peak23(j,3) = peak(j,2) - peak(j+1,3);
   
   peak13(j,1) = peak(j,1) - peak(j-1, 3);
   peak13(j,2) = peak(j,1) - peak(j, 3);
   peak13(j,3) = peak(j,1) - peak(j+1, 3);
end
if (pos1 > pos2 && pos2 > pos3)
    disp("right");
    direction = 2;
end
if (pos3 > pos2 && pos2 > pos1)
    disp("left");
    direction = 0;
end
if (pos2 > pos3 && pos2 > pos1 || pos2 < pos3 && pos2 < pos1)
    disp("middle");
    direction = 1;
end
