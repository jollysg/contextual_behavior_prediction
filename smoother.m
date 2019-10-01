likelihood = straight_likelihood.Data(:);

figure(2);

plot(likelihood);

smoothed_likelihood = movmean(likelihood, 30);

figure(3);
plot(smoothed_likelihood);

fft_likelihood = fft(likelihood);

f = (0:length(fft_likelihood)-1)*100/length(fft_likelihood);

figure(4);
plot(f,fft_likelihood)
title('Magnitude')

tic
A = rand(12000, 4400);
B = rand(12000, 4400);
toc
C = A'.*B';
toc