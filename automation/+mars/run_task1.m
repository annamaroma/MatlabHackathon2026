function result = run_task1(cfg)
if nargin < 1 || isempty(cfg)
    cfg = mars.default_config();
end

task_cfg = cfg.task1;
debug_prints = local_get_task1_setting(cfg, task_cfg, 'debug_prints', true);
max_debug_failures = local_get_task1_setting(cfg, task_cfg, 'max_debug_failures', 5);

if debug_prints && ~cfg.parallel.show_progress
    cfg.parallel.show_progress = true;
end

parallel_enabled = mars.detect_parallel_support(cfg);
mars.ensure_profile_data_loaded(cfg);

result_dir = mars.build_results_dir(cfg, task_cfg.name);
runner = mars.create_runner_model(cfg, task_cfg, result_dir);

addpath(result_dir);
cleanup = onCleanup(@() local_cleanup(runner.model_name, result_dir));

load_system(runner.model_file);
stop_time = mars.get_stop_time(runner.model_name);

solar_values = task_cfg.solar_kw_grid(:);
soc_values = task_cfg.init_soc_grid(:);
[solar_grid, soc_grid] = ndgrid(solar_values, soc_values);

solar_block = [runner.model_name, '/', local_get_task1_setting(cfg, task_cfg, 'solar_block', 'Solar')];
solar_parameter = local_get_task1_setting(cfg, task_cfg, 'solar_parameter', 'power_rating');
battery_block = [runner.model_name, '/', local_get_task1_setting(cfg, task_cfg, 'battery_block', 'Battery')];
battery_parameter = local_get_task1_setting(cfg, task_cfg, 'battery_parameter', 'initSOC');

num_runs = numel(solar_grid);

if debug_prints
    local_print_task1_start( ...
        task_cfg, ...
        runner, ...
        result_dir, ...
        stop_time, ...
        solar_values, ...
        soc_values, ...
        num_runs, ...
        parallel_enabled, ...
        solar_block, ...
        solar_parameter, ...
        battery_block, ...
        battery_parameter);
end

sim_inputs(num_runs, 1) = Simulink.SimulationInput(runner.model_name);
for idx = 1:num_runs
    sim_inputs(idx, 1) = Simulink.SimulationInput(runner.model_name);
    sim_inputs(idx, 1) = sim_inputs(idx, 1).setModelParameter('ReturnWorkspaceOutputs', 'on');
    sim_inputs(idx, 1) = sim_inputs(idx, 1).setBlockParameter(solar_block, solar_parameter, mars.format_scalar(solar_grid(idx)));
    sim_inputs(idx, 1) = sim_inputs(idx, 1).setBlockParameter(battery_block, battery_parameter, mars.format_scalar(soc_grid(idx)));
end

sim_outputs = mars.run_simulations(cfg, sim_inputs);
summary_table = local_build_summary_table(cfg, task_cfg, runner, stop_time, solar_grid(:), soc_grid(:), sim_outputs);
best_result = local_select_best_result(summary_table, runner, result_dir, stop_time, sim_outputs);
fig_handles = local_create_figures(task_cfg, summary_table, best_result, solar_values, soc_values, stop_time);

if debug_prints
    local_print_task1_summary(summary_table, best_result, result_dir, max_debug_failures);
end

run_results = struct();
run_results.runner = runner;
run_results.stop_time = stop_time;
run_results.sim_outputs = sim_outputs;
run_results.solar_values = solar_values;
run_results.soc_values = soc_values;

mars.save_task_outputs(cfg, task_cfg.name, result_dir, summary_table, run_results, best_result, fig_handles);

if debug_prints
    fprintf('[task1] Saved outputs to %s\n', result_dir);
end

result = struct();
result.result_dir = result_dir;
result.runner = runner;
result.stop_time = stop_time;
result.summary_table = summary_table;
result.best_result = best_result;
result.figure_files = local_build_figure_files(fig_handles, result_dir);
end

function summary_table = local_build_summary_table(cfg, task_cfg, runner, stop_time, solar_values, soc_values, sim_outputs)
num_runs = numel(sim_outputs);

run_index = (1:num_runs).';
solar_kW = solar_values(:);
initSOC_pct = soc_values(:);
completed = false(num_runs, 1);
mission_fraction = zeros(num_runs, 1);
end_time_s = NaN(num_runs, 1);
finalSOC_pct = NaN(num_runs, 1);
minSOC_pct = NaN(num_runs, 1);
maxSOC_pct = NaN(num_runs, 1);
error_message = strings(num_runs, 1);

soc_var_name = local_logging_variable(runner, 'SOC', 'mars_SOC');

for idx = 1:num_runs
    sim_out = sim_outputs(idx, 1);
    error_message(idx) = string(mars.get_error_message(sim_out));
    end_time_s(idx) = mars.get_simulation_end_time(sim_out);

    if ~isnan(end_time_s(idx)) && stop_time > 0
        mission_fraction(idx) = min(end_time_s(idx) / stop_time, 1.0);
    end

    soc_signal = mars.get_sim_output_var(sim_out, soc_var_name);
    if soc_signal.found && ~isempty(soc_signal.data)
        finalSOC_pct(idx) = soc_signal.data(end);
        minSOC_pct(idx) = min(soc_signal.data);
        maxSOC_pct(idx) = max(soc_signal.data);
    end

    completed(idx) = strlength(error_message(idx)) == 0 ...
        && ~isnan(end_time_s(idx)) ...
        && end_time_s(idx) >= stop_time - cfg.simulation.time_tolerance;
end

summary_table = table( ...
    run_index, ...
    solar_kW, ...
    initSOC_pct, ...
    completed, ...
    mission_fraction, ...
    end_time_s, ...
    finalSOC_pct, ...
    minSOC_pct, ...
    maxSOC_pct, ...
    error_message, ...
    'VariableNames', { ...
        'run_index', ...
        'solar_kW', ...
        'initSOC_pct', ...
        'pass', ...
        'mission_fraction', ...
        'end_time_s', ...
        'finalSOC_pct', ...
        'minSOC_pct', ...
        'maxSOC_pct', ...
        'error_message' ...
    });

    summary_table = sortrows(summary_table, {'solar_kW', 'initSOC_pct'}, {'ascend', 'ascend'});
    summary_table.secondary_rank = zeros(height(summary_table), 1);
    passing_rows = find(summary_table.pass);
    summary_table.secondary_rank(passing_rows) = 1:numel(passing_rows);

    summary_table.soc_within_bounds = ...
        summary_table.minSOC_pct >= task_cfg.soc_min ...
        & summary_table.maxSOC_pct <= task_cfg.soc_max;
end

function best_result = local_select_best_result(summary_table, runner, result_dir, stop_time, sim_outputs)
passing_rows = summary_table(summary_table.pass, :);
if isempty(passing_rows)
    best_result = struct();
    best_result.found = false;
    best_result.message = 'No Task 1 combination completed the full mission duration.';
    best_result.result_dir = result_dir;
    best_result.runner = runner;
    best_result.stop_time = stop_time;
    best_result.selection_rule = 'minimum solar_kW, then minimum initSOC_pct';
    return;
end

passing_rows = sortrows(passing_rows, {'solar_kW', 'initSOC_pct'}, {'ascend', 'ascend'});
selected_row = passing_rows(1, :);
selected_index = selected_row.run_index;
selected_output = sim_outputs(selected_index, 1);

best_result = table2struct(selected_row, 'ToScalar', true);
best_result.found = true;
best_result.result_dir = result_dir;
best_result.runner = runner;
best_result.stop_time = stop_time;
best_result.selection_rule = 'minimum solar_kW, then minimum initSOC_pct';
best_result.signals = local_collect_best_signals(runner, selected_output);
end

function signals = local_collect_best_signals(runner, sim_out)
signals = struct();
signals.SOC = mars.get_sim_output_var(sim_out, local_logging_variable(runner, 'SOC', 'mars_SOC'));
signals.battPower_kW = mars.get_sim_output_var(sim_out, local_logging_variable(runner, 'battPower_kW', 'mars_battPower_kW'));
signals.loadVoltage_V = mars.get_sim_output_var(sim_out, local_logging_variable(runner, 'loadVoltage_V', 'mars_loadVoltage_V'));
end

function fig_handles = local_create_figures(task_cfg, summary_table, best_result, solar_values, soc_values, stop_time)
pass_matrix = reshape(summary_table.pass, numel(solar_values), numel(soc_values));
completion_matrix = reshape(summary_table.mission_fraction, numel(solar_values), numel(soc_values));
final_soc_matrix = reshape(summary_table.finalSOC_pct, numel(solar_values), numel(soc_values));
final_soc_matrix(~pass_matrix) = NaN;

fig_handles = gobjects(0, 1);
fig_handles(end + 1, 1) = local_plot_pass_heatmap(pass_matrix, solar_values, soc_values, best_result);
fig_handles(end + 1, 1) = local_plot_completion_heatmap(completion_matrix, solar_values, soc_values, best_result);
fig_handles(end + 1, 1) = local_plot_final_soc_heatmap(final_soc_matrix, solar_values, soc_values, best_result);
fig_handles(end + 1, 1) = local_plot_best_traces(best_result, stop_time, task_cfg);
end

function fig_handle = local_plot_pass_heatmap(pass_matrix, solar_values, soc_values, best_result)
fig_handle = figure('Visible', 'off', 'Color', 'w', 'Tag', 'task1_pass_heatmap');
imagesc(soc_values, solar_values, double(pass_matrix));
set(gca, 'YDir', 'normal');
colormap(fig_handle, [0.80, 0.27, 0.27; 0.22, 0.66, 0.33]);
caxis([0, 1]);
colorbar('Ticks', [0, 1], 'TickLabels', {'Fail', 'Pass'});
xlabel('Initial SOC (%)');
ylabel('Solar Power Rating (kW)');
title('Task 1 Pass Map');
grid on;
hold on;
local_plot_selection_marker(best_result);
hold off;
end

function fig_handle = local_plot_completion_heatmap(completion_matrix, solar_values, soc_values, best_result)
fig_handle = figure('Visible', 'off', 'Color', 'w', 'Tag', 'task1_completion_heatmap');
imagesc(soc_values, solar_values, completion_matrix);
set(gca, 'YDir', 'normal');
colormap(fig_handle, parula(256));
caxis([0, 1]);
color_handle = colorbar('Ticks', 0:0.25:1);
color_handle.Label.String = 'Mission Completion Fraction';
xlabel('Initial SOC (%)');
ylabel('Solar Power Rating (kW)');
title('Task 1 Mission Completion');
grid on;
hold on;
local_plot_selection_marker(best_result);
hold off;
end

function fig_handle = local_plot_final_soc_heatmap(final_soc_matrix, solar_values, soc_values, best_result)
fig_handle = figure('Visible', 'off', 'Color', 'w', 'Tag', 'task1_final_soc_heatmap');
imagesc(soc_values, solar_values, final_soc_matrix, 'AlphaData', ~isnan(final_soc_matrix));
set(gca, 'YDir', 'normal');
colormap(fig_handle, turbo(256));
caxis([0, 100]);
color_handle = colorbar();
color_handle.Label.String = 'Final SOC (%)';
xlabel('Initial SOC (%)');
ylabel('Solar Power Rating (kW)');
title('Task 1 Final SOC for Passing Runs');
grid on;
hold on;
local_plot_selection_marker(best_result);
hold off;
end

function fig_handle = local_plot_best_traces(best_result, stop_time, task_cfg)
fig_handle = figure('Visible', 'off', 'Color', 'w', 'Tag', 'task1_best_trace');
tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

if ~best_result.found
    nexttile([3, 1]);
    axis off;
    text(0.5, 0.5, 'No passing Task 1 configuration was found.', ...
        'HorizontalAlignment', 'center', ...
        'FontSize', 12);
    return;
end

    nexttile;
    local_plot_signal(best_result.signals.SOC, stop_time, 'SOC (%)', [task_cfg.soc_min, task_cfg.soc_max]);
    title(sprintf('Best Task 1 Run: %g kW, %g%% initial SOC', best_result.solar_kW, best_result.initSOC_pct));

    nexttile;
    local_plot_signal(best_result.signals.battPower_kW, stop_time, 'Battery Power (kW)', []);
    yline(0, '--', 'Zero Power', 'Color', [0.3, 0.3, 0.3]);

    nexttile;
    local_plot_signal(best_result.signals.loadVoltage_V, stop_time, 'Load Voltage (V)', []);
    xlabel('Time (s)');
end

function local_plot_signal(signal, stop_time, y_label, band)
plotted = false;
if signal.found && ~isempty(signal.time) && ~isempty(signal.data)
    plot(signal.time, signal.data, 'LineWidth', 1.3);
    xlim([0, stop_time]);
    ylabel(y_label);
    grid on;
    plotted = true;
else
    axis off;
    text(0.5, 0.5, ['Signal not available: ', y_label], ...
        'HorizontalAlignment', 'center');
end

if plotted && ~isempty(band)
    hold on;
    yline(band(1), '--', 'Min', 'Color', [0.35, 0.35, 0.35]);
    yline(band(2), '--', 'Max', 'Color', [0.35, 0.35, 0.35]);
    hold off;
end
end

function local_plot_selection_marker(best_result)
if ~isstruct(best_result) || ~isfield(best_result, 'found') || ~best_result.found
    return;
end

plot(best_result.initSOC_pct, best_result.solar_kW, 'kp', ...
    'MarkerSize', 12, ...
    'MarkerFaceColor', [1, 0.90, 0.1], ...
    'DisplayName', 'Selected optimum');
text(best_result.initSOC_pct, best_result.solar_kW, ...
    sprintf('  %g kW / %g%%', best_result.solar_kW, best_result.initSOC_pct), ...
    'Color', [0.1, 0.1, 0.1], ...
    'FontWeight', 'bold', ...
    'VerticalAlignment', 'bottom');
end

function figure_files = local_build_figure_files(fig_handles, result_dir)
figure_files = strings(0, 1);
for idx = 1:numel(fig_handles)
    if ~ishandle(fig_handles(idx))
        continue;
    end
    figure_files(end + 1, 1) = string(fullfile(result_dir, 'figures', [get(fig_handles(idx), 'Tag'), '.png']));
end
end

function local_print_task1_start(task_cfg, runner, result_dir, stop_time, solar_values, soc_values, num_runs, parallel_enabled, solar_block, solar_parameter, battery_block, battery_parameter)
fprintf('[task1] Starting Task 1 sweep for %s.\n', task_cfg.model);
fprintf('[task1] Result directory: %s\n', result_dir);
fprintf('[task1] Runner model: %s\n', runner.model_name);
fprintf('[task1] Execution mode: %s\n', local_execution_mode(parallel_enabled));
fprintf('[task1] Stop time: %.6g s\n', stop_time);
fprintf('[task1] Solar sweep: %g to %g kW (%d values)\n', solar_values(1), solar_values(end), numel(solar_values));
fprintf('[task1] Initial SOC sweep: %g to %g %% (%d values)\n', soc_values(1), soc_values(end), numel(soc_values));
fprintf('[task1] Total scheduled runs: %d\n', num_runs);
fprintf('[task1] Solar parameter: %s / %s\n', solar_block, solar_parameter);
fprintf('[task1] Battery parameter: %s / %s\n', battery_block, battery_parameter);
end

function local_print_task1_summary(summary_table, best_result, result_dir, max_debug_failures)
num_runs = height(summary_table);
num_pass = nnz(summary_table.pass);
num_fail = num_runs - num_pass;

fprintf('[task1] Sweep finished. %d/%d runs passed.\n', num_pass, num_runs);

if num_fail > 0
    failed_rows = summary_table(~summary_table.pass, :);
    failed_rows = sortrows(failed_rows, {'mission_fraction', 'solar_kW', 'initSOC_pct'}, {'descend', 'ascend', 'ascend'});
    report_count = min(max_debug_failures, height(failed_rows));
    if report_count > 0
        fprintf('[task1] Closest failing runs (%d shown):\n', report_count);
        for idx = 1:report_count
            row = failed_rows(idx, :);
            message = local_single_line_message(row.error_message);
            if strlength(message) == 0
                message = "no explicit error captured";
            end

            fprintf('[task1]   run=%d solar=%g kW initSOC=%g %% mission=%.1f %% end=%.6g s error=%s\n', ...
                row.run_index, ...
                row.solar_kW, ...
                row.initSOC_pct, ...
                100 * row.mission_fraction, ...
                row.end_time_s, ...
                char(message));
        end
    end
end

if isstruct(best_result) && isfield(best_result, 'found') && best_result.found
    fprintf('[task1] Selected best run: solar=%g kW, initSOC=%g %%, finalSOC=%.2f %%.\n', ...
        best_result.solar_kW, ...
        best_result.initSOC_pct, ...
        best_result.finalSOC_pct);
else
    [~, best_idx] = max(summary_table.mission_fraction);
    best_attempt = summary_table(best_idx, :);
    fprintf('[task1] No passing run found. Best attempt reached %.1f %% of the mission at %g kW and %g %% initSOC.\n', ...
        100 * best_attempt.mission_fraction, ...
        best_attempt.solar_kW, ...
        best_attempt.initSOC_pct);
end

fprintf('[task1] Summary artifacts will be under %s\n', result_dir);
end

function mode = local_execution_mode(parallel_enabled)
if parallel_enabled
    mode = 'parallel (parsim)';
else
    mode = 'sequential fallback';
end
end

function message = local_single_line_message(raw_message)
message = string(raw_message);
message = replace(message, newline, ' | ');
message = replace(message, sprintf('\r'), ' ');
message = strtrim(message);

if strlength(message) > 160
    message = extractBefore(message, 158) + "...";
end
end

function value = local_get_task1_setting(cfg, task_cfg, field_name, default_value)
value = default_value;

if isfield(task_cfg, field_name) && ~isempty(task_cfg.(field_name))
    value = task_cfg.(field_name);
end

if isfield(cfg, 'overrides') && isstruct(cfg.overrides)
    if isfield(cfg.overrides, 'task1') && isstruct(cfg.overrides.task1) ...
            && isfield(cfg.overrides.task1, field_name) ...
            && ~isempty(cfg.overrides.task1.(field_name))
        value = cfg.overrides.task1.(field_name);
        return;
    end

    if isfield(cfg.overrides, field_name) && ~isempty(cfg.overrides.(field_name))
        value = cfg.overrides.(field_name);
    end
end
end

function var_name = local_logging_variable(runner, signal_name, default_name)
field_name = matlab.lang.makeValidName(signal_name);
if isfield(runner.logging, field_name)
    var_name = runner.logging.(field_name);
else
    var_name = default_name;
end
end

function local_cleanup(model_name, result_dir)
if bdIsLoaded(model_name)
    close_system(model_name, 0);
end

try
    rmpath(result_dir);
catch
end
end
