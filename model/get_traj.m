clf;
figure(1);
axis([0 100 0 100]);
hold on;
grid on;
title('Draw:');
xlabel('X Coordinates');
ylabel('Y Coordinates');

h = drawfreehand('Closed', false, 'Smoothing', 2); 
input('Press Enter to finish:');


rawPos = h.Position;
delete(h);

rawX = rawPos(:,1);
rawY = rawPos(:,2);

diffs = diff(rawPos);
segmentLengths = sqrt(sum(diffs.^2, 2));
cumDist = [0; cumsum(segmentLengths)];

totalLength = cumDist(end);
numFrames = 200; %Variable frames, we can increase if we need more waypoints for better accuracy!
evenDist = linspace(0, totalLength, numFrames);

animX = interp1(cumDist, rawX, evenDist, 'pchip');
animY = interp1(cumDist, rawY, evenDist, 'pchip');
trajectory = [animX; animY]';