clear; close all;

splinePoints = [0 0
                1 0.5
                3 2.2
                9 3.4
                13 5.2
                20 10.3];

[x,y] = splineToTrajectory(splinePoints)