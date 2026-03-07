clear all; close all; clc;

%% 1. Calculate Nominal target system (1/s)
% This only needs to be computed once for all axes
num = [0 1];
den = [1 0];
sys = tf(num, den);
freq_range = linspace(0.2 * 2 * pi, 500 * 2 * pi, 10000); 
[MAG, PHASE, W] = bode(sys, freq_range);
freq_Hz_Nominal  = W / (2 * pi);
MAG_dB_Nominal = 20 * log10(abs(MAG));

%% 2. Loop through x, y, z axes
axes_list = {'x', 'y', 'z'};

for i = 1:length(axes_list)
    axis_name = axes_list{i};
    file_name = [axis_name, '_data.txt']; % Expects 'x_data.txt', 'y_data.txt', 'z_data.txt'
    
    % Check if file exists before loading
    if ~isfile(file_name)
        fprintf('Warning: File %s not found, skipping...\n', file_name);
        continue;
    end
    
    % Load data
    data_matrix = load(file_name);
    t_sim   = data_matrix(:, 1);
    in_vol  = data_matrix(:, 2);
    out_deg = data_matrix(:, 3);

    % 3. Frequency identification
    tSamp = 0.001;
    time = 0:tSamp:(length(in_vol)-1)*tSamp;
    
    u1 = vpck(in_vol, time);
    y1 = vpck(out_deg, time);
    P1 = vspect(u1, y1, 2056, 256, 'hanning');
    
    estdata = sel(P1, 1, 4);
    Omega = abs(estdata(1:end, 2));
    Mag = abs(estdata(1:end, 1));
    % phase = angle(estdata(1:end, 1)); % Optional, not used in plot
    
    freq_Hz_b1 = Omega / (2 * pi);
    MAG_dB_b1 = 20 * log10(Mag);
    
    % 4. Plotting
    figure('Position', [100, 100, 388*3, 388*3]);
    linewd = 10;
    lineco = [223, 168, 0]/255;
    
    h2 = loglog(freq_Hz_b1, squeeze(MAG_dB_b1), 'linewidth', linewd, 'Color', lineco, 'LineStyle', '-'); hold on;
    h3 = loglog(freq_Hz_Nominal, squeeze(MAG_dB_Nominal), 'linewidth', 10, 'Color', [0, 0, 0], 'LineStyle', '-.'); hold on;
    
    xlabel('Frequency(Hz)', 'FontSize', 50);
    ylabel('Mag(dB)', 'FontSize', 50);
    grid on;
    grid minor;
    set(gca, 'XScale', 'log');
    set(gca, 'YScale', 'log');
    set(gca, 'FontSize', 50);
    set(gca, 'XMinorTick', 'on');
    set(gca, 'YMinorTick', 'on');
    
    legend([h3, h2], {'Target ($\frac{1}{s}$)', 'Offline-Linearized'}, 'Interpreter', 'latex', 'FontSize', 50);
    title([axis_name, '-axis'], 'Interpreter', 'tex', 'FontSize', 50, 'FontWeight', 'normal');
    
    % 5. Save figure
    output_filename = ['offline-', axis_name, '.png'];
    %print(gcf, '-dpng', '-r600', output_filename);
    fprintf('Successfully processed %s-axis and saved as %s\n', axis_name, output_filename);
end

disp('All axes processed successfully!');