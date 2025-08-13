function [u] = fsmc_controller(ref, actual, de_input, fis, lambda) % <-- Pastikan ini 'de_input'
    e = ref - actual;
    de = de_input; % <-- Pastikan ini 'de = de_input;'
    s = de + lambda * e;
    alpha = evalfis(fis, [e de]);
    u = -alpha * sign(s);
    u = max(min(u, 5), -5); % Pastikan batasan ini sesuai, bisa jadi perlu ditingkatkan
end