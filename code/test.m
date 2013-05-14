
img = imread('mapa.bmp');

if isnan(img)
    disp('Must load work area before loading the plan.');
    return;
end

points=NaN;
imshow(img);
hold;

img_coordinate=[];
xi = 2;
yi = 5;
img_coordinate = [img_coordinate; xi yi];

rate = 640/28.5;
while 1
    try
        [pos_x pos_y button]=ginput(1);
    catch
        return;
    end
    if button==3
        break;
    else
        img_coordinate=[img_coordinate;...
            fix(pos_y)/rate...
            fix(pos_x)/rate];
        plot(pos_x,pos_y,'color','g','Marker','+','LineWidth',3);
    end
end
hold;

%Interpolate data avoiding obstacles.

points = [];

for i = 1:length(img_coordinate)-1
    
    points=[ points; path_plan(img_coordinate(i,:), ...
        img_coordinate(i+1,:), ...
        img, ...
        1/rate) ];
end

disp('Path created saved to POINTS.mat');
save POINTS points
points