function noise = noiseCalc
mag = 0.005;
noisePDF = normpdf(-1:0.001:1,0,.75);
index = randi(length(noisePDF));
%noise = mag*(noisePDF(index)/max(noisePDF));
if (index > length(noisePDF)/2)
    noise = mag*(noisePDF(index)/max(noisePDF));
else
    noise = -mag*(noisePDF(index)/max(noisePDF));
end