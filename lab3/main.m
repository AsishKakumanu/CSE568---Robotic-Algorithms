% -- Clear --
% -- 1. Command Window --
% -- 2. Workspace --

clc
clear all

% -- Color Images --
img_name = 'image';
color = '-color.jpg';
ext = '.jpg';

% -- SSD Images --
output_ssd = '-ssd.jpg';

% -- NCC Images -- 
output_ncc = '-ncc.jpg';

for i = 1:6
    % -- Color Images - strCat -- 
    num_ext = strcat(int2str(i),ext);
    filename = strcat(img_name,num_ext);
    output_num_color = strcat(int2str(i),color);
    output_num_color_ext = strcat(img_name,output_num_color);
    
    % -- Color Images - strCat - SSD --
    num_ext_ssd = strcat(int2str(i),output_ssd);
    filename_ssd = strcat(img_name,num_ext_ssd);
    
    % -- Color Images - strCat - NCC --
    num_ext_ncc = strcat(int2str(i),output_ncc);
    filename_ncc = strcat(img_name,num_ext_ncc);
    
    % -- Reading the Image --
    img = imread(filename);
    img = im2double(img);
    %imshow(img);

    % -- Dimensions of Image --
    s = size(img);

    % -- Individual Dimensions --
    v_dimensions = s(1);
    h_dimensions = s(2);
    v_dimen_fix = floor(v_dimensions/3);

    % -- Separating Color Channels --
    b_im=img(1:v_dimen_fix,:);
    g_im=img(v_dimen_fix+1:2*v_dimen_fix,:);
%   r_im=img(2*v_dimen_fix+1:v_dimensions,:);
    r_im=img(2*v_dimen_fix+1:3*v_dimen_fix,:);
%   r_im_resize = imresize(r_im, [v_dimen_fix h_dimensions]);


    % -- Aligning together --
    RGB_im = cat(3,r_im,g_im,b_im);
    red_channel = RGB_im(:,:,1);
    green_channel = RGB_im(:,:,2);
    blue_channel = RGB_im(:,:,3);
    imwrite(RGB_im,output_num_color_ext,'jpg');
    
    % -- SSD -- 
    fprintf('\n\n');
    fprintf('File : %s\n',filename);
    fprintf('SSD\n');
    fprintf('shifted_red w.r.t blue_channel\n');
    shifted_red = im_align1(r_im,b_im);
    fprintf('shifted_green w.r.t blue_channel\n');
    shifted_green = im_align1(g_im,b_im);
    
    % -- Aligning together, Writing --
    RGB_final_ssd = cat(3,shifted_red,shifted_green,b_im);
    imwrite(RGB_final_ssd,filename_ssd,'jpg');
    
    % -- NCC -- 
    fprintf('\n\n');
    fprintf('File : %s\n',filename);
    fprintf('NCC\n');
    fprintf('shifted_red w.r.t blue_channel\n');
    shifted_red_ncc = im_align2(r_im,b_im);
    fprintf('shifted_green w.r.t blue_channel\n');
    shifted_green_ncc = im_align2(g_im,b_im);
    
    RGB_final_ncc = cat(3,shifted_red_ncc,shifted_green_ncc,b_im);
    %figure, imshow(RGB_final_ncc);
    imwrite(RGB_final_ncc,filename_ncc,'jpg');
    
    % -- Harris Corner -- 
    %shifted_red_harris = im_align3(r_im,b_im);
    
    registerImages(r_im,b_im);

    
end