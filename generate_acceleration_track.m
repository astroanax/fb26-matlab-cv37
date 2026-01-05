% Generate Acceleration Track for Team Unwired C37 - Formula Bharat 2026
% Elongated oval circuit with 250m straights and 20m radius corners
% 
% Track specification:
% - Two parallel straights of 250m each
% - Semicircular ends with 20m radius
% - Total lap length: 500m + 2*pi*20 = 625.66m
%
% Author: Team Unwired, NIT Calicut
% Date: January 2026

function [xTrack, yTrack, sTrack] = generate_acceleration_track()
    % Track parameters
    straight_length = 250;  % m
    corner_radius = 20;     % m
    track_width = 4;        % m (for visualization)
    
    % Number of points for smooth curves
    n_straight = 100;
    n_corner = 50;
    
    % Generate centerline coordinates
    % Start at origin, going counter-clockwise
    
    % Section 1: Bottom straight (left to right)
    x1 = linspace(0, straight_length, n_straight);
    y1 = zeros(size(x1)) - corner_radius;
    
    % Section 2: Right semicircle (bottom to top)
    theta2 = linspace(-pi/2, pi/2, n_corner);
    x2 = straight_length + corner_radius * cos(theta2);
    y2 = corner_radius * sin(theta2);
    
    % Section 3: Top straight (right to left)
    x3 = linspace(straight_length, 0, n_straight);
    y3 = zeros(size(x3)) + corner_radius;
    
    % Section 4: Left semicircle (top to bottom)
    theta4 = linspace(pi/2, 3*pi/2, n_corner);
    x4 = corner_radius * cos(theta4);
    y4 = corner_radius * sin(theta4);
    
    % Concatenate all sections
    xTrack = [x1, x2, x3, x4];
    yTrack = [y1, y2, y3, y4];
    
    % Calculate cumulative distance along centerline
    dx = diff(xTrack);
    dy = diff(yTrack);
    ds = sqrt(dx.^2 + dy.^2);
    sTrack = [0, cumsum(ds)];
    
    % Create figure
    figure('Position', [100 100 1200 500]);
    hold on; grid on; axis equal;
    
    % Plot centerline
    plot(xTrack, yTrack, 'k-', 'LineWidth', 2.5, 'DisplayName', 'Centerline');
    
    % Plot track boundaries
    track_normal_x = -gradient(yTrack);
    track_normal_y = gradient(xTrack);
    track_normal_mag = sqrt(track_normal_x.^2 + track_normal_y.^2);
    track_normal_x = track_normal_x ./ track_normal_mag;
    track_normal_y = track_normal_y ./ track_normal_mag;
    
    % Inner boundary
    x_inner = xTrack - track_width/2 * track_normal_x;
    y_inner = yTrack - track_width/2 * track_normal_y;
    plot(x_inner, y_inner, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Track Boundary');
    
    % Outer boundary
    x_outer = xTrack + track_width/2 * track_normal_x;
    y_outer = yTrack + track_width/2 * track_normal_y;
    plot(x_outer, y_outer, 'r--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    
    % Mark start/finish line
    plot([0 0], [y_inner(1) y_outer(1)], 'g-', 'LineWidth', 4, 'DisplayName', 'Start/Finish');
    
    % Mark 75m acceleration zone
    accel_75m_idx = find(sTrack >= 75, 1);
    if ~isempty(accel_75m_idx)
        plot([xTrack(accel_75m_idx) xTrack(accel_75m_idx)], ...
             [y_inner(accel_75m_idx) y_outer(accel_75m_idx)], ...
             'b-', 'LineWidth', 3, 'DisplayName', '75m Accel Zone');
    end
    
    % Formatting
    xlabel('X Position (m)', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('Y Position (m)', 'FontSize', 12, 'FontWeight', 'bold');
    title('Team Unwired C37 - Acceleration Test Track', 'FontSize', 14, 'FontWeight', 'bold');
    legend('Location', 'southeast', 'FontSize', 10);
    
    % Add track dimensions as text
    text(straight_length/2, -corner_radius-8, sprintf('Straight: %dm', straight_length), ...
         'HorizontalAlignment', 'center', 'FontSize', 11, 'FontWeight', 'bold');
    text(straight_length + corner_radius + 8, 0, sprintf('R=%dm', corner_radius), ...
         'HorizontalAlignment', 'left', 'FontSize', 11, 'FontWeight', 'bold');
    text(straight_length/2, corner_radius+8, sprintf('Total Lap: %.2fm', sTrack(end)), ...
         'HorizontalAlignment', 'center', 'FontSize', 11, 'FontWeight', 'bold', 'Color', [0 0.5 0]);
    
    % Set axis limits with padding
    xlim([-corner_radius-10, straight_length+corner_radius+10]);
    ylim([-corner_radius-15, corner_radius+15]);
    
    % Save figure
    saveas(gcf, 'results/acceleration_track.png');
    fprintf('Track diagram saved to results/acceleration_track.png\n');
    fprintf('Track length: %.2f m\n', sTrack(end));
    fprintf('Straight section: %d m\n', straight_length);
    fprintf('Corner radius: %d m\n', corner_radius);
    
    % Export track data for simulation
    Track = struct();
    Track.x = xTrack';
    Track.y = yTrack';
    Track.s = sTrack';
    Track.straight_length = straight_length;
    Track.corner_radius = corner_radius;
    Track.track_width = track_width;
    save('results/acceleration_track_data.mat', 'Track');
    fprintf('Track data saved to results/acceleration_track_data.mat\n');
end
