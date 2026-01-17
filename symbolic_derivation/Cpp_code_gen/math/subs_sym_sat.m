function out = subs_sym_sat(u, A, k)
    % Python: s0_up = np.tanh(0.5 * 30 * A)
    s0_up = tanh(0.5 * k * A);
    
    % Python: gamma_up = 1 / (s0_up + 1e-6)
    gamma_up = 1 / (s0_up + 1e-6);
    
    % Python: -A + np.log(1 + np.exp(30 * (gamma_up * up + A))) / 30 ...
    term1 = log(1 + exp(k * (gamma_up * u + A))) / k;
    term2 = log(1 + exp(k * (gamma_up * u - A))) / k;
    
    out = -A + term1 - term2;
end