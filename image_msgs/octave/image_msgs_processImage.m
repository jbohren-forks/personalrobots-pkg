%% I = image_msgs_processImage(image)
%%
%% returns an image matrix given a image_msgs/Image
function I = image_msgs_processImage(image)
%res.camimage.layout
%I = zeros([data.height data.width 3]);
layout = eval(sprintf('image.%s_data.layout',image.depth));
dimsizes = [];
for i = 1:length(layout.dim)
    dimsizes(i) = layout.dim{i}.size;
end
I = reshape(eval(sprintf('image.%s_data.data',image.depth)),dimsizes(end:-1:1));
%% reverse the dimension order
I = permute(I,length(dimsizes):-1:1);
