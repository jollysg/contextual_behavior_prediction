
meas = [groundTruth(:).y_tilde];
time_ser = [groundTruth(:).t];
estim = [filter_traj(:).combined_estimates];

% predictions - 5x50x5x251 no_of_states, no_of_predictions, no_of_models,
% no_of_time_steps
%predictions = out.predictions;

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
        
%         preds = out.predictions.Data(:,:,:,i);
        straight_pass_preds = filter_traj(i).predictions(:,:,1);
        straight_agg_preds = filter_traj(i).predictions(:,:,2);
        left_lane_passive = filter_traj(i).predictions(:,:,3);
        left_lane_aggressive = filter_traj(i).predictions(:,:,4);
        
%         if (lane_reached == false)
% %             y_pred = wted_pred(4,:);
%             y_pred = left_lane_aggressive(4,:);
%             max_y = max(y_pred);
%             if ( max_y >= 3.49 )
%                 lane_reached = true;
%                 ind = find(y_pred == max_y);
%                 max_ys_x = left_lane_aggressive(1,ind);            
%             end
%         else
%             ind_to_fix = find(left_lane_aggressive(1,:) >= max_ys_x);
%             left_lane_aggressive(4,ind_to_fix) = max_y;
%         end
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
%             straight_preds(1, :), straight_preds(3,:));
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
%         wt1 = squeeze(wts(1,1,1:i));
        plot(estim(1, 1:i), wt1, ...
            estim(1, 1:i), wt2, ...
            estim(1, 1:i), wt3, ...
            estim(1, 1:i), wt4, ...
            'Linewidth', 1.5);
        grid on
%         plot(time_ser(1:i), wt1, ...
%             time_ser(1:i), wt2, ...
%             time_ser(1:i), wt3, ...
%             time_ser(1:i), wt4, ...
%             'Linewidth', 1.5);
%         xlim([0 time_ser(end)]);
%         xlabel('time (seconds)');
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
%         plot(time_ser(1:i), driver_wts(1,1:i), ...
%             time_ser(1:i), driver_wts(2,1:i), ...
%             'Linewidth', 1.5);
%         xlabel('time (seconds)');
%         xlim([0 time_ser(end)]);
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
 
% 111 is the index of the timestep
% predictions = out.predictions.Data(:,:,111);
% plot(predictions(1,:), predictions(3,:));
