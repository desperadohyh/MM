p1 = [1 1];
line = [0 0; -1 0];
dist = distLinSeg(line(1,:), line(2,:), p1, p1);
disp(dist);