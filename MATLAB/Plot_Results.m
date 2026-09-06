function Plot_Results(t, x, u_hist, u_nom_hist, control_mode, params)
% PLOT_RESULTS

    if nargin < 5, control_mode = 'Unknown'; end
    if nargin < 6, params = System_Parameters(); end

    switch control_mode
        case 'CLF_Only'
            mode_label    = 'CLF Only';
            applied_label = '$u_{\mathrm{CLF}}$';
            fig2_title    = 'CLF Control Inputs';
            show_cbf      = false;
            show_bounds   = false;   % unconstrained — no actuator limit lines
        case 'Decoupled'
            mode_label    = 'Decoupled (CLF + Safety Filter)';
            applied_label = '$u_{\mathrm{safe}}$ (CLF + Safety Filter)';
            fig2_title    = 'Nominal CLF vs Safety-Filtered Control';
            show_cbf      = true;
            show_bounds   = true;
        case 'Unified'
            mode_label    = 'Unified (CLF-CBF QP)';
            applied_label = '$u_{\mathrm{unified}}$ (CLF-CBF QP)';
            fig2_title    = 'Nominal CLF vs Unified CLF-CBF Control';
            show_cbf      = true;
            show_bounds   = true;
        otherwise
            mode_label    = control_mode;
            applied_label = '$u_{\mathrm{applied}}$';
            fig2_title    = 'Control Inputs';
            show_cbf      = false;
            show_bounds   = false;
    end

 
    theta_max_deg = rad2deg(params.theta_max);

    theta_des_deg = 30;   % desired pitch in degrees

%% FIGURE 1 — Euler Angles

    figure;
    set(gcf, 'Color', 'White');

    angle_cols    = {'b', 'r', [0.2 0.6 0.2]};
    angle_titles  = {'Roll $\phi$', 'Pitch $\theta$', 'Yaw $\psi$'};
    angle_ylabels = {'$\phi \; [\mathrm{deg}]$', ...
                     '$\theta \; [\mathrm{deg}]$', ...
                     '$\psi \; [\mathrm{deg}]$'};

    for k = 1:3
        subplot(3,1,k);
        hold on;
        plot(t, rad2deg(x(:,k)), 'Color', angle_cols{k}, 'LineWidth', 2);

        if k == 2 && show_cbf
            yline(theta_max_deg, 'k--', 'LineWidth', 1.5);
        end

        if k == 2 && show_cbf
            yline(theta_des_deg, 'b-.', 'LineWidth', 1.2);
        end

        grid on; grid minor;
        set(gca, 'FontSize', 12, 'TickLabelInterpreter', 'latex');
        ylabel(angle_ylabels{k}, 'Interpreter', 'latex');
        xlabel('Time [s]', 'Interpreter', 'latex');

        if k == 2 && show_cbf
            title(['Pitch $\theta$ --- safety limit: $\theta_{\max} = ' ...
                   num2str(theta_max_deg, '%.0f') '^\circ$, desired: $' ...
                   num2str(theta_des_deg, '%.0f') '^\circ$'], ...
                  'Interpreter', 'latex');
            legend({'$\theta$', '$\theta_{\max}$', '$\theta_{\mathrm{des}}$'}, ...
                   'Location', 'best', 'Interpreter', 'latex');
        else
            title([angle_titles{k} ' --- ' mode_label], 'Interpreter', 'latex');
        end
        hold off;
    end

    %% FIGURE 2 — Angular Rates

    figure;
    set(gcf, 'Color', 'White');
    hold on;

    for k = 1:3
        plot(t, rad2deg(x(:, 3+k)), 'LineWidth', 2);
    end

    grid on; grid minor;
    set(gca, 'FontSize', 12, 'TickLabelInterpreter', 'latex');
    title(['Angular Rates --- ' mode_label], 'Interpreter', 'latex');
    xlabel('Time [s]', 'Interpreter', 'latex');
    ylabel('Rate [deg/s]', 'Interpreter', 'latex');
    legend({'$p$', '$q$', '$r$'}, 'Location', 'best', 'Interpreter', 'latex');
    hold off;

%% FIGURE 3 — Applied control inputs

    figure;
    set(gcf, 'Color', 'White');
    hold on;

    plot(t, u_hist, 'LineWidth', 2);

    if show_bounds
        yline(params.u_max, 'k:', 'LineWidth', 1.2);
        yline(params.u_min, 'k:', 'LineWidth', 1.2);
        u_legend = {'$u_1$','$u_2$','$u_3$','$u_4$','$u_{\max}$','$u_{\min}$'};
    else
        u_legend = {'$u_1$','$u_2$','$u_3$','$u_4$'};
    end

    grid on; grid minor;
    set(gca, 'FontSize', 12, 'TickLabelInterpreter', 'latex');
    title(['Applied Control Inputs --- ' mode_label], 'Interpreter', 'latex');
    xlabel('Time [s]', 'Interpreter', 'latex');
    ylabel('$u \; [\mathrm{N \cdot m}]$', 'Interpreter', 'latex');
    legend(u_legend, 'Location', 'best', 'Interpreter', 'latex');
    hold off;

%% FIGURE 4 — Nominal vs Applied, per channel

    figure;
    set(gcf, 'Color', 'White');

    color_nom     = [0.2 0.4 0.8];
    color_applied = [0.8 0.2 0.2];

    for k = 1:4
        subplot(4,1,k);
        hold on;
        plot(t, u_nom_hist(:,k), '--', 'Color', color_nom,     'LineWidth', 2);
        plot(t, u_hist(:,k),     '-',  'Color', color_applied, 'LineWidth', 2);

        if show_bounds
            yline(params.u_max, 'k:', 'LineWidth', 1.0);
            yline(params.u_min, 'k:', 'LineWidth', 1.0);
        end

        grid on; grid minor;
        set(gca, 'FontSize', 12, 'TickLabelInterpreter', 'latex');
        ylabel(['$u_' num2str(k) ' \; [\mathrm{N \cdot m}]$'], 'Interpreter', 'latex');

        if k == 1
            title(fig2_title, 'Interpreter', 'latex');
            legend({'$u_{\mathrm{nom}}$ (CLF)', applied_label}, ...
                   'Location', 'best', 'Interpreter', 'latex');
        end
        hold off;
    end
    xlabel('Time [s]', 'Interpreter', 'latex');

%% FIGURE 5 — CBF value (Decoupled / Unified only)

    if show_cbf
        figure;
        set(gcf, 'Color', 'White');
        hold on;

        h_vals = theta_max_deg - rad2deg(x(:,2));
        plot(t, h_vals, 'm', 'LineWidth', 2);
        yline(0, 'k--', 'LineWidth', 1.5);

        % Show where theta equals the desired value
       % h_des = theta_max_deg - theta_des_deg;
       % yline(h_des, 'b-.', 'LineWidth', 1.2);

        grid on; grid minor;
        set(gca, 'FontSize', 12, 'TickLabelInterpreter', 'latex');
        title('CBF Value $h = \theta_{\max} - \theta$ (must remain $\geq 0$)', ...
              'Interpreter', 'latex');
        xlabel('Time [s]', 'Interpreter', 'latex');
        ylabel('$h \; [\mathrm{deg}]$', 'Interpreter', 'latex');
        legend({'$h(x)$', 'Safety boundary ($h=0$)', ...
                ['Desired $\theta = ' num2str(theta_des_deg) '^\circ$']}, ...
               'Location', 'best', 'Interpreter', 'latex');
        hold off;
    end

end