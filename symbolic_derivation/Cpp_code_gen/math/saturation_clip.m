function [y] = saturation_clip(x,x_min,x_max)
  y=min(max(x,x_min),x_max);







end

