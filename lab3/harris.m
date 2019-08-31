clear all
clc

img_name = 'image';
ext = '.jpg';

for i=1:1
    num_ext = strcat(int2str(i),ext);
    filename = strcat(img_name,num_ext);
    img = imread(filename);
    img = im2double(img);
    %imshow(img);

    % -- Dimensions of Image --
    s = size(img);

    % -- Individual Dimensions --
    v_dimensions = s(1);
    h_dimensions = s(2);
    v_dimen_fix = round(v_dimensions/3);

    % -- Separating Color Channels --
    b_im=img(1:v_dimen_fix,:);
    g_im=img(v_dimen_fix+1:2*v_dimen_fix,:);
    %   r_im=img(2*v_dimen_fix+1:v_dimensions,:);
    r_im=img(2*v_dimen_fix+1:3*v_dimen_fix,:);
    %   r_im_resize = imresize(r_im, [v_dimen_fix h_dimensions]);
    % Default spatial referencing objects

    fixedRefObj = imref2d(size(b_im));
    movingRefObj = imref2d(size(r_im));
    movingRefObj2 = imref2d(size(g_im));

    points1 = detectHarrisFeatures(b_im);
    points2 = detectHarrisFeatures(r_im);
    points3 = detectHarrisFeatures(g_im);

    [features1,valid_points1] = extractFeatures(b_im,points1);
    [features2,valid_points2] = extractFeatures(r_im,points2);
    [features3,valid_points3] = extractFeatures(g_im,points3);

    indexPairs = matchFeatures(features1,features2);
    indexPairs2 = matchFeatures(features1,features3);

    matchedPoints1 = valid_points1(indexPairs(:,1),:);
    matchedPoints2 = valid_points2(indexPairs(:,2),:);
    p1 = matchedPoints1(:).Location;
    p2 = matchedPoints2(:).Location;

    % Red w.r.t Blue Offsets
    
    offset = floor(p1 - p2);
    

    shifted_red = circshift(r_im, [offset(1) offset(2)]);

    matchedPoints3 = valid_points1(indexPairs2(:,1),:);
    matchedPoints4 = valid_points3(indexPairs2(:,2),:);

    p3 = matchedPoints3(:).Location;
    p4 = matchedPoints4(:).Location;


    % Green w.r.t Blue Offsets
    offset2 = floor(p3 - p4);


    shifted_green = circshift(g_im, [offset(1) offset(2)]);


    size(shifted_red);
    size(shifted_green);
    size(b_im);


    f_im = cat(3,shifted_red,shifted_green,b_im);
    imshow(f_im)


    figure; showMatchedFeatures(b_im,r_im,matchedPoints1,matchedPoints2);
    figure; showMatchedFeatures(b_im,g_im,matchedPoints3,matchedPoints4);
    % 
    
end