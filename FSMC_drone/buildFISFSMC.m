function fis = buildFISFSMC(channel_name)
    fis = mamfis('Name', ['FSMC_' channel_name]);

    % Input: Error (e)
    fis = addInput(fis, [-5 5], 'Name', 'e');
    fis = addMF(fis, 'e', 'gaussmf', [0.4 -1], 'Name', 'N'); % index 1
    fis = addMF(fis, 'e', 'gaussmf', [0.4  0], 'Name', 'Z'); % index 2
    fis = addMF(fis, 'e', 'gaussmf', [0.4  1], 'Name', 'P'); % index 3

    % Input: Derivative of Error (de)
    fis = addInput(fis, [-5 5], 'Name', 'de');
    fis = addMF(fis, 'de', 'gaussmf', [0.4 -1], 'Name', 'N'); % index 1
    fis = addMF(fis, 'de', 'gaussmf', [0.4  0], 'Name', 'Z'); % index 2
    fis = addMF(fis, 'de', 'gaussmf', [0.4  1], 'Name', 'P'); % index 3

    % Output: Alpha (FSMC gain adaptif)
    fis = addOutput(fis, [0 2], 'Name', 'alpha');
    fis = addMF(fis, 'alpha', 'trimf', [0.0 0.5 1.0], 'Name', 'S'); % index 1
    fis = addMF(fis, 'alpha', 'trimf', [0.8 1.0 1.2], 'Name', 'M'); % index 2
    fis = addMF(fis, 'alpha', 'trimf', [1.0 1.5 2.0], 'Name', 'B'); % index 3

    % Rules: [e_idx, de_idx, alpha_idx, weight, operator]
    ruleList = [
        1 1 3 1 1   % N N -> B
        1 2 2 1 1   % N Z -> M
        1 3 1 1 1   % N P -> S
        2 1 2 1 1   % Z N -> M
        2 2 2 1 1   % Z Z -> M
        2 3 2 1 1   % Z P -> M
        3 1 1 1 1   % P N -> S
        3 2 2 1 1   % P Z -> M
        3 3 3 1 1   % P P -> B
    ];

    fis = addRule(fis, ruleList);
end
