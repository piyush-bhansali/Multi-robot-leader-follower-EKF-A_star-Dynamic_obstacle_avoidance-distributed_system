function state_pred = kinematic_model_v3(state, d, delta_theta)
theta = state(3);
% Predict next state
    x_new = state(1) + (d) * cos(theta + delta_theta);
    y_new = state(2) + (d) * sin(theta + delta_theta);
    theta_new = wrapToPi(state(3) + delta_theta);
    state_pred = [x_new; y_new; theta_new];

end