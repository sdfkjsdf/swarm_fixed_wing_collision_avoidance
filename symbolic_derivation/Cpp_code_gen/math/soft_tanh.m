function y = soft_tanh(val, limit)
    % SOFT_TANH: Smooth saturation using hyperbolic tangent
    % Formula: limit * tanh(val / limit)
    
    y = limit * tanh(val / limit);
end