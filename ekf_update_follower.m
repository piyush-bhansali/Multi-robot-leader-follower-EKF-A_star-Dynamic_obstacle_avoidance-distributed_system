
% Extended Kalman filter update state algorithm for the leader
function [state_upd, P_upd] = ekf_update_follower(state_pred, P_pred, measurement, R_GPS)

    if any(isnan(measurement))
        state_upd = state_pred;
        P_upd = P_pred;
        return;
    
    else
        z_pred_GPS = [state_pred(1); state_pred(2)];
        gps_x = measurement(1);
        gps_y = measurement(2);
        H_GPS = [1, 0, 0;
            0, 1, 0];
        y_GPS = [gps_x; gps_y] - z_pred_GPS;
        S_GPS = H_GPS * P_pred * H_GPS' + R_GPS;
        K_GPS = P_pred * H_GPS' / S_GPS;
        state_upd = state_pred + K_GPS * y_GPS;
        % Covariance update (Joseph form for numerical stability)
        I = eye(size(P_pred));
        P_upd = (I - K_GPS * H_GPS) * P_pred * (I - K_GPS * H_GPS)' + K_GPS * R_GPS * K_GPS';
    
    end
    end
    