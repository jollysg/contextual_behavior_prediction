% MIT License
%
% Copyright (c) 2020 Jasprit Singh Gill (jaspritsgill@gmail.com)
%
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:

% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.

% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.


meas = [groundTruth(:).y_tilde];
time_ser = [groundTruth(:).t];
estim = [filter_traj(:).combined_estimates];

% predictions - 5x50x5x251 no_of_states, no_of_predictions, no_of_models,
% no_of_time_steps

%1x5x251 
driver_wts = [filter_traj(:).driver_weights];
wts = [filter_traj(:).weights];
right_lane_center = zeros(1,length(time_ser));
left_lane_center = right_lane_center + 3.5;
trail_length = 20;

lane_reached = false;
max_ys_x = -1;

for i = 1:length(time_ser)
    if mod(i-1,5) == 0
        
        straight_pass_preds = filter_traj(i).predictions(:,:,1);
        straight_agg_preds = filter_traj(i).predictions(:,:,2);
        left_lane_passive = filter_traj(i).predictions(:,:,3);
        left_lane_aggressive = filter_traj(i).predictions(:,:,4);
        
        wt1 = wts(1,1:i);
        wt2 = wts(2,1:i);
        wt3 = wts(3,1:i);
        wt4 = wts(4,1:i);
        wted_pred = straight_pass_preds * wt1(end) ...
            + straight_agg_preds * wt2(end) ...
            + left_lane_passive * wt3(end) ...
            + left_lane_aggressive * wt4(end);
        trail_begin = i-trail_length;
        if trail_begin < 1
            trail_begin = 1;
        end
        front_vehicle_positions = [filter_traj(trail_begin:i).front_car_position];
        left_lane_vehicle_positions = [filter_traj(trail_begin:i).leftLaneVehPosn];
        tiledlayout(3,1);
        nexttile;
        
        plot(meas(1, 1:i), meas(2, 1:i), ...
            estim(1, 1:i), estim(4, 1:i), ...
            wted_pred(1, :), wted_pred(4,:), ...
            front_vehicle_positions, right_lane_center(trail_begin:i), ...
            left_lane_vehicle_positions, left_lane_center(trail_begin:i), ...
            'Linewidth', 1.5);

        ylabel('y coordinate (m)');
        xlabel('x coordinate (m)');
        xlim([0 estim(1, end)]);
        ymin = min(meas(2,:))-2;
        ymax = max(meas(2,:))+2;
        ylim([ymin ymax]);
        legend('measurement', 'estimate', 'prediction', 'front vehicle', 'left lane vehicle');
        legend('Location', 'eastoutside');
        title('Participant trajectories X vs Y (m)');
        grid on
        
        nexttile;
        plot(estim(1, 1:i), wt1, ...
            estim(1, 1:i), wt2, ...
            estim(1, 1:i), wt3, ...
            estim(1, 1:i), wt4, ...
            'Linewidth', 1.5);
        grid on
        xlabel('x coordinate (m)');
        xlim([0 estim(1, end)]);
        ylabel('probabilistic weights');
        ylim([-0.1 1.1]);
        legend('straight passive', 'straight aggressive', 'left LC long' ...
                                            , 'left LC short');
        legend('Location', 'eastoutside');
        title('Behavior weights (probability vs time)');
        
        nexttile

        plot(estim(1, 1:i), driver_wts(1,1:i), ...
            estim(1, 1:i), driver_wts(2,1:i), ...
            'Linewidth', 1.5);
        grid on
        xlabel('x coordinate (m)');
        xlim([0 estim(1, end)]);
        ylabel('probabilistic weights');
        ylim([-0.1 1.1]);
        legend('aggressive driver', 'passive driver');
        legend('Location', 'eastoutside');
        title('Driver type weights vs time');
        
        pause(0.2);
    end
end
 