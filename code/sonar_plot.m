function [ handle ] = sonar_plot(axes_handle,radius)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    angle = (pi/2:-pi/7:-pi/2)+pi/2;
    handle = polar(axes_handle,angle,radius,'rx');
    set(handle,'LineWidth',2,'MarkerSize',10);
end

