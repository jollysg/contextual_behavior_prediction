function x_plus_1 = ctra_model_fun(X, Y, dt, w)
    % X = [x y theta v a w]'
    x = X(1);
    y = X(2);
    theta = X(3);
    v = X(4);
    a = X(5);
    w = X(6);
    x_plus = 1/w^2*((v*w + a*w*dt) * sin(theta + w*dt) + a*cos(theta + w*dt) - v*w*sin(theta) - a*cos(theta)) + x;
    y_plus = 1/w^2*((-v*w - a*w*dt) * cos(theta + w*dt) + a*sin(theta +w*dt) + v*w*cos(theta) - a*sin(theta)) + y;
    x_plus_1 = [x_plus; y_plus; w*dt + theta; a*dt + v; a; w];
end
