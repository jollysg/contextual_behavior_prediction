
meas = out.measurements;
time_ser = out.tout;
estim = out.combined_estimate;

% predictions - 5x50x5x251 no_of_states, no_of_predictions, no_of_models,
% no_of_time_steps
predictions = out.predictions;

%1x5x251 
wts = out.weights.Data;
figure(1)

for i = 1:length(time_ser)
    if mod(i-1,5) == 0
        
        preds = out.predictions.Data(:,:,:,i);
        straight_preds = out.predictions.Data(:,:,1,i);
        left_lane_short_preds = out.predictions.Data(:,:,2,i);
        left_lane_long_preds = out.predictions.Data(:,:,3,i);
        right_lane_short_preds = out.predictions.Data(:,:,4, i);
        right_lane_long_preds = out.predictions.Data(:,:,5,i);
        wt1 = squeeze(wts(1,1,1:i));
        wt2 = squeeze(wts(1,2,1:i));
        wt3 = squeeze(wts(1,3,1:i));
        wt4 = squeeze(wts(1,4,1:i));
        wt5 = squeeze(wts(1,5,1:i));
        wted_pred = straight_preds * wt1(end) ...
            + left_lane_short_preds * wt2(end) ...
            + left_lane_long_preds * wt3(end) ...
            + right_lane_short_preds * wt4(end) ...
            + right_lane_long_preds * wt5(end);
        tiledlayout(2,1);
        nexttile;
%         plot(time_ser(1:i), meas.Data(1:i,2), ...
%             time_ser(1:i), estim.Data(1:i, 3), ...
%             time_ser(i:i-1+length(wted_pred(3,:))), wted_pred(3,:));
        plot(meas.Data(1:i,1), meas.Data(1:i,2), ...
            estim.Data(1:i, 1), estim.Data(1:i, 3), ...
            wted_pred(1, :), wted_pred(3,:), ...
            'Linewidth', 1.5);
%             straight_preds(1, :), straight_preds(3,:));
        xlabel('x coordinate (m)');
        ylabel('y coordinate (m)');
%         xlim([0 time_ser(end)]);
        xlim([0 estim.Data(end,1)]);
        ymin = min(meas.Data(:,2))-2;
        ymax = max(meas.Data(:,2))+2;
        ylim([ymin ymax]);
        legend('measurement', 'estimate', 'prediction');
        title('Trajectory of the tracked vehicle');
        
        nexttile;
%         wt1 = squeeze(wts(1,1,1:i));
        plot(estim.Data(1:i, 1), wt1, ...
            estim.Data(1:i, 1), wt2, ...
            estim.Data(1:i, 1), wt3, ...
            estim.Data(1:i, 1), wt4, ...
            estim.Data(1:i, 1), wt5, ...
            'Linewidth', 1.5);

%         plot(time_ser(1:i), wt1, ...
%             time_ser(1:i), wt2, ...
%             time_ser(1:i), wt3, ...
%             time_ser(1:i), wt4, ...
%             time_ser(1:i), wt5);
        xlabel('x coordinate (m)');
        ylabel('probabilistic weights');
        title('Evolution of the weights with the trajectory of the vehicle');

        xlim([0 estim.Data(end,1)]);

%         xlim([0 time_ser(end)]);
        ylim([-0.1 1.5]);
        legend('straight', 'left LC short', 'left LC long', ...
                'right LC short', 'right LC long');
        pause(0.2);
    end
end
 
figure(2)

plot(time_ser(1:i), wt1, ...
    time_ser(1:i), wt2, ...
    time_ser(1:i), wt3, ...
    time_ser(1:i), wt4, ...
    time_ser(1:i), wt5, ...
    'Linewidth', 1.5);
xlabel('Time (seconds)');
ylabel('probabilistic weights');
title('Evolution of the weights with time');

xlim([0 time_ser(end)]);
ylim([-0.1 1.3]);
legend('straight', 'left LC short', 'left LC long', ...
    'right LC short', 'right LC long');

%%

figure(3)
plot(meas.Time, meas.Data(:,2), 'Linewidth', 1.5);
        xlabel('Time (seconds)');
        ylabel('y coordinate (m)');
        xlim([0 time_ser(end)]);
%         xlim([0 estim.Data(end,1)]);
        ymin = min(meas.Data(:,2))-2;
        ymax = max(meas.Data(:,2))+2;
        ylim([ymin ymax]);
        legend('measurement');
        title('Trajectory of the tracked vehicle');
  

% 111 is the index of the timestep
predictions = out.predictions.Data(:,:,111);
% plot(predictions(1,:), predictions(3,:));

meas_data = [out.gt_highD.trajectories_x.Data(:) out.gt_highD.trajectories_y.Data(:)];
estim_data = [estim.Data(:,1) estim.Data(:,3)];
rms(meas_data - estim_data)


