function align = im_align2(channel1,channel2)
    
    offset = [0 0];
    img1 = channel1;
    img2 = channel2;
        
    % -- Calculation of Norm Correlation -- 
    c = normxcorr2(img1,img2);
    %figure, surf(c), shading flat
    
    % -- Calculating the Peak -- 
    [max_c, imax] = max(abs(c(:)));
    [ypeak, xpeak] = ind2sub(size(c),imax(1));
    
    % -- Calculating Offsets -- 
    offset(1) = ypeak - size(img1,1);
    offset(2) = xpeak - size(img1,2);
    
    fprintf('The alignments are %.2f and %.2f\n',offset(1),offset(2));
    align = circshift(channel1, [offset(1) offset(2)]);
end