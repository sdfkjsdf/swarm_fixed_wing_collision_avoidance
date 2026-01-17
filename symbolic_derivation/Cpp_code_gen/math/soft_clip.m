function y = soft_clip(val, low, high, k)
    % SOFT_CLIP: Differentiable saturation between low and high
    % Logic: Min(Max(val, low), high) using soft functions
    
    % 1. 하한선 적용 (Max(val, low))
    lower_bounded = soft_max(val, low, k);
    
    % 2. 상한선 적용 (Min(..., high))
    y = soft_min(lower_bounded, high, k);
end