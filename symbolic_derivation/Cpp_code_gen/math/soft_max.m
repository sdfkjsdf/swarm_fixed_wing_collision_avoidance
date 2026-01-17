function y = soft_max(a, b, k)
    % SOFT_MAX: Differentiable approximation of max(a, b)
    % Formula: log(exp(k*a) + exp(k*b)) / k
    
    y = log(exp(k*a) + exp(k*b)) / k;
end