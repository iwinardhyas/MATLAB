function r = isoctave()
    % ISOCTAVE  True if we are running in Octave.
    r = exist('OCTAVE_VERSION', 'builtin') ~= 0;
end
