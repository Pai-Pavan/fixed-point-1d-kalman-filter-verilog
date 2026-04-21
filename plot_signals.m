function plot_signals()
    % Define Kalman filter output
    kf = [0.2, 0.7, 1.0, 2.2, 2.7, 3.8, 5.2, 5.7, 7.1, 7.6,...
          9.1, 10.1, 10.9, 12.5, 13.0, 14.2, 15.6, 16.0, 17.5, 18.1,...
          19.5, 20.1, 21.4, 22.2, 22.8, 24.6, 25.3, 26.5, 27.8, 28.2,...
          29.6, 29.4, 30.2, 29.6, 30.0, 29.8, 30.5, 29.8, 30.3, 30.0,...
          30.5, 29.7, 30.0, 29.8, 30.6, 29.7, 30.0, 29.7, 30.3, 29.6,...
          30.2]';

    % Define true signal
    true = [0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 ...
            20 21 22 23 24 25 26 27 28 29 30 ...
            30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 ...
            30 30 30 30 30 30 30 30 30 30 30 30 30 30]';

    % Define noisy measurements
    noisy = [0.8 1.3 1.5 3.6 3.2 5.1 6.8 6.2 8.7 8.1 10.9 11.4 11.8 14.2 13.5 ...
             15.7 17.1 16.6 19.0 18.3 21.2 20.5 22.7 23.1 23.6 26.4 25.8 27.9 ...
             29.3 28.7 30.9 29.2 31.1 28.8 30.5 29.6 31.4 28.9 30.7 29.3 31.2 ...
             28.7 30.4 29.8 31.5 28.6 30.3 29.5 31.0 28.9 30.6 29.7 31.3 28.8 ...
             30.2 29.4 31.1 28.7 30.5 29.6 30.2]';

    % Align lengths
    n = min([numel(kf), numel(true), numel(noisy)]);
    kf    = kf(1:n);
    true  = true(1:n);
    noisy = noisy(1:n);

    % Time vector
    t = (0:n-1)';

    % Plot signals
    figure;
    hold on;
    plot(t, true, '-g', 'LineWidth', 1.5, 'DisplayName', 'original signal');
    plot(t, noisy, '-r', 'LineWidth', 1.5, 'DisplayName', 'noisy measurement value');
    plot(t, kf, '-b', 'LineWidth', 1.5, 'DisplayName', 'kalman filter value');
    hold off;

    xlabel('time');
    ylabel('value');
    title('Original, Noisy Measurement, and Kalman Filter Estimates');
    legend('Location','best');
    grid on;
end
