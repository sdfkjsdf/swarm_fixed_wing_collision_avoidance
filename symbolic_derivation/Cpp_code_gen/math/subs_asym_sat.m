% function out = subs_asym_sat(u, L, U, k)
%     % Differentiable clip: smooth approx of min(max(u,L),U)
%     % (No max/abs/sign -> symbolic Jacobian clean)
% 
%     xL = k*(u - L);
%     xU = k*(u - U);
% 
%     spL = log(1 + exp(xL));
%     spU = log(1 + exp(xU));
% 
%     out = L + (spL - spU)./k;
% end

function out = subs_asym_sat(u, L, U, k)
   u0=(L+U)/2;
    eps = 1e-6;
    us = u - u0;  Ls = L - u0;  Us = U - u0;
    v0 = (1/k) * ( log(1 + exp(k*Ls)) - log(1 + exp(-k*Us)) );
    s0 = 1/(1 + exp(-k*(0 - v0 - Ls))) - 1/(1 + exp(-k*(0 - v0 - Us)));
    gamma = 1/(s0 + eps);
    out_s = Ls + log(1 + exp(k*(gamma*us - v0 - Ls)))/k ...
              - log(1 + exp(k*(gamma*us - v0 - Us)))/k;
    out = out_s + u0;
end

