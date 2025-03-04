function springForce = fs(state1, state2, desiredSeparations)
    
    ci = 0.8 + (1.2 - 0.8) * rand(1);
    error = state1 - state2 - desiredSeparations;
    springForce = ci * error + 0.1 * error.^2;

end