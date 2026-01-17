function y = soft_min(a, b, k)
    % SOFT_MIN: Differentiable approximation of min(a, b)
    % Formula: -log(exp(-k*a) + exp(-k*b)) / k
    
    y = -log(exp(-k*a) + exp(-k*b)) / k;
end