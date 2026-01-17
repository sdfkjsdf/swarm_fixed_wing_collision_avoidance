function [output] = saturation_tanh(x,x_max)


output=x_max*tanh(x/x_max);


end

