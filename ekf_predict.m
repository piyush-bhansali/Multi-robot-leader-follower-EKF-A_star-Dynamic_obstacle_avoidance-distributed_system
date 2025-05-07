function [state_pred, P_pred] = ekf_predict(robotPose, encoder, P, Q, robot)

ticks_L = encoder(1);
ticks_R = encoder(2);

% Convert ticks to wheel displacements
s_L = (2 * pi * robot.wheelRadius/ robot.numOfTicks) * ticks_L;
s_R = (2 * pi * robot.wheelRadius / robot.numOfTicks) * ticks_R;

% Estimate robot motion
d = (s_L + s_R) / 2;
delta_theta = (s_R - s_L) / robot.axleLength;

state_pred = kinematic_model_v3(robotPose, d, delta_theta);
theta = robotPose(3);

F = [1, 0, -d * sin(theta + delta_theta/2);
     0, 1, d * cos(theta + delta_theta/2);
     0, 0, 1];

G = [cos(theta + delta_theta), (-d/2) * sin(theta + delta_theta);
     sin(theta + delta_theta), (d/2) * cos(theta + delta_theta);
     0, 1];

P_pred = F * P * F' + G * Q * G';

end

