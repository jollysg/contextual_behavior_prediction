function stats = findStatsFromVector(data_vector)
    % return matrix order min_y, max_y, mean_y, var_y, stddev_y
    min_y = min(data_vector);
    max_y = max(data_vector);
    mean_y = mean(data_vector);
    var_y = var(data_vector);
    stddev_y = std(data_vector);
    stats = [min_y, max_y, mean_y, var_y, stddev_y];
end