s = serial('COM7');
set(s, 'InputBufferSize', 1024); %number of bytes in inout buffer
set(s, 'FlowControl', 'none');
set(s, 'BaudRate', 57600);
set(s, 'Parity', 'none');
set(s, 'DataBits', 8);
set(s, 'StopBit', 1);
set(s, 'Timeout',100);
fopen(s)

figure;

t = 0:.01:2*pi;
track = 3.5*ones(size(t));
%P = polar(t, track);
%set(P, 'Visible', 'off');

hold on;

current = zeros(19,1);
last1 = zeros(19,1);
last2 = zeros(19,1);
avgdist = zeros(19,1);

for n = 1:(18)
    bin(n) = n;
end

while 1
    clf;
    last2 = last1;
    last1 = current;
    line = fgetl(s);
    vals = strsplit(line,'=');
    vals = transpose(vals);
    bin  = zeros(size(vals));
    for n = 1:(size(vals)-1)
        split_vals = strsplit(vals{n},',');
        bin(n) = str2double(split_vals{1});
        current(n) = str2double(split_vals{2});
    end
    
    %for n = 1:(size(vals)-1)
    %    fprintf('bin: %d, dist: %1.3f\n',bin(n),dist(n));
    %end
    
    if (last2(7) ~= 0) && (last1(7) ~= 0)
        avgdist = 0.6*current + 0.3*last1 + 0.1*last2;
    elseif last1(7) ~= 0
        avgdist = 0.65*current + 0.35*last1;
    else 
        avgdist = current;
    end
    
    angle = bin*-pi/9;
    angle = angle + pi/2;
    i=size(avgdist);
    avgdist(i(1)) = avgdist(1);
    %dist(i(1)) = dist(1);
    P = polar(t, track);
    set(P, 'Visible', 'off');
    hold on;
    polar(angle,avgdist);
    title('Car Surroundings (up is forwards, distance in meters)');
    drawnow;
end