function [ticks_L, ticks_R] = get_encoder_values(v, omega, dt, robot)

  tickNoiseStd = 2;
  
  % Wheel velocities
  
  v_L = v - (robot.axleLength / 2) * omega;
  v_R = v + (robot.axleLength / 2) * omega;
  
  % Displacement per wheel
  s_L = v_L * dt;
  s_R = v_R * dt;
  
  % Convert to ticks
  ticks_L = round(((s_L / (2 * pi * robot.wheelRadius)) * robot.numOfTicks) + randn * tickNoiseStd);
  ticks_R = round(((s_R / (2 * pi * robot.wheelRadius)) * robot.numOfTicks) + randn * tickNoiseStd);
  
  end
  