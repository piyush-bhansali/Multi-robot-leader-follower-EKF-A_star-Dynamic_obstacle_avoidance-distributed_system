% Low pass filter to smooth noise signals
function state_filtered = low_pass_filter(state_filtered, state, alpha)
state_filtered = alpha * state + (1 - alpha) * state_filtered;
end