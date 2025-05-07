% Differential drive mobile robot model
function state_pred = kinematic_model(state, v, omega, dt)

  state_pred = state; % Initialize to maintain size
  state_pred(3) = state(3) + omega * dt;
  state_pred(3) = atan2(sin(state_pred(3)), cos(state_pred(3)));
  theta = state(3);
  state_pred(1) = state(1) + v * cos(theta) * dt;
  state_pred(2) = state(2) + v * sin(theta) * dt;
  % Normalize angle
  end