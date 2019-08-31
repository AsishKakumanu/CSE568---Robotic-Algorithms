function align = im_align1(channel1,channel2)
    
    % Flooring Dimensions
    v1 = floor(0.1*size(channel1,1)):floor(0.9*size(channel1,1));
    h1 = floor(0.1*size(channel1,2)):floor(0.9*size(channel1,2));
    v2 = floor(0.1*size(channel2,1)):floor(0.9*size(channel2,1));
    h2 = floor(0.1*size(channel2,2)):floor(0.9*size(channel2,2));
    img1 = channel1(v1,h1);
    img2 = channel2(v2,h2);

    % Generate Shift Values
    ranges_1 = -20:20;
    ranges_2 = -20:20;
    
    %pre_ssd = sum(sum(channel1-channel2).^2);
    pre_ssd = inf;
    ranges = [0 0];
   
    for i = ranges_1
        for j = ranges_2
            %temp_channel = imtranslate(channel1, [i j]);
            temp_channel = circshift(img1, [i j]);
            temp_ssd = sum(sum((img2-temp_channel).^2));
            %temp_ssd = immse(img2,temp_channel);
            if temp_ssd < pre_ssd
                pre_ssd = temp_ssd;
                ranges(1) = i;
                ranges(2) = j;
            end
        end
    end
    
    %fprintf('\n The squared error for %s is %2.2f\n', round(pre_ssd,2));
    fprintf('The alignments are %.2f and %.2f\n',ranges(1),ranges(2));
    align = circshift(channel1, [ranges(1) ranges(2)]);
end