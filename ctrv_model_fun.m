function x_plus_1 = ctrv_model_fun(X, Y, dt, w)
    % X = [x y theta v w]'
    x = X(1);
    y = X(2);
    theta = X(3);
    v = X(4);
    w = X(5);
    x_plus = v/w*sin(w * dt + theta) - v/w * sin(theta) + x;
%     y_plus = -v/w * cos(w * dt + theta) + v/w * sin(theta) + y;
    y_plus = -v/w * cos(w * dt + theta) + v/w * cos(theta) + y;
    x_plus_1 = [x_plus; y_plus; w*dt + theta; v; w];
end