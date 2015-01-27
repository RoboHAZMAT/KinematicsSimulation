function imageH = CropWhitespace(imageH)
[M,N,~] = size(imageH);
A = zeros(M,N);
for i=1:M
    for j=1:N
            if(sum(imageH(i,j,:)) == 765)
                imageH(i,j,:) = [231;231;231];
            end
    end
end
imwrite(imageH,'RoboHAZMAT_CAD.png');