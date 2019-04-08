% behaviors - left lane, lane follow, right lane, brake
pdf_behavior = [0.25 0.25 0.25 0.25];

% situations - front vehicle far enough(s1), front vehicle near(s2)
pdf_behavior_s1 = [1 0 1 1]*1/3;

pdf_behavior_s2 = [0 1 0 0];

mu1 = 10;
sigma1 = 1;
mu2 = 15;
sigma2 = 2;


x = 0:0.01:20;
n = length(x);
% y = zeros(1, n);
% y2 = zeros(1, n);

% for i = 1:n
%     y(1, i) = getGaussian(x(i), mu1, sigma1);
%     y1(1, i) = getGaussian(x(i), mu2, sigma2);
% end
% 

% y = normpdf(x, mu1, sigma1);
% y1 = normpdf(x, mu2, sigma2);
% 
% ym = y .* y1;
% normalizer = sum(ym);
% ynorm = ym/normalizer;
% 
% distr = fitdist(y', 'norm')
% getSituation([0])
% 
% plot(x, y, x, y1, x, ynorm)

% plot(x, y, x, y1, x, ym)
% figure(2)
% plot(x, ym, x, ynorm)

function y = getGaussian(x, mu, sigma)
    y = 1/sqrt(2*pi)/sigma*exp(-(x - mu)^2/(2*sigma*sigma));
end

function y = getSituation(context)
    % evaluate context - relative distances, velocities
    context = getContext([0]);
    
    disp('Character is:')
    disp(context)
    
    % calculate probability of a situation based on the context
    y = [0.9 0.1];
end

function y = getContext(X_states)
    % extract contextual information from the current states
    
    % context - relative distances, velocities
    %distances around [front, rear, left, right]. -1 indicates no vehicle,
    %large vehicle
    distances = [4, 10, -1, -1];

    %velocities around [front, rear, left, right]. -1 indicates the other
    %vehicle is slower
    velocities = [2, -1, 0, 0]; 
    y = [distances velocities];
end



