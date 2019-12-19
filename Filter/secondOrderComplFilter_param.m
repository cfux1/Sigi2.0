
%% Initialize Sampling Time/Frequency
% Sampling frequency [Hz]
fs = 1000;
% Sampling Time [s]
Ts = 1/fs;
%% Initialize parameters filter
T   = 0.6;          % Filter time constant (higher -> slower, less noisy; lower -> faster, more noise)
d   = 2;            % damping (bigger 1; vgl Aachen)