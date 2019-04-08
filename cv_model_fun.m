function x_plus_1 = cv_model_fun(X, Y, dt, w)
    % X = [x vx y vy]'
    x = X(1);
    vx = X(2);
    y = X(3);
    vy = X(4);    
    x_plus_1 = [x + dt * vx; vx; y + dt * vy; vy];
end