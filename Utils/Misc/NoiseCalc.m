function noise = NoiseCalc
%% ==========================Noise Calculation=============================
% RoboHAZMAT: Senior Design Project
% Motion Control Team
% Gerardo Bledt
% December 15, 2014
%
% Calculates synthetic noise to be used in a generic situation. Magnitude
% can be changed to fit various applications.

% The magnitude of the noise
mag = 0.005;

% Creates the Gaussian PDF
noisePDF = normpdf(-1:0.001:1,0,.75);

% Cooses random point on the distribution
index = randi(length(noisePDF));
% noise = mag*(noisePDF(index)/max(noisePDF));
if (index > length(noisePDF)/2)
    noise = mag*(noisePDF(index)/max(noisePDF));
else
    noise = -mag*(noisePDF(index)/max(noisePDF));
end