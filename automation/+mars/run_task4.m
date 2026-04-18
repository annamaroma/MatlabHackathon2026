function result = run_task4(cfg)
if nargin < 1 || isempty(cfg)
    cfg = mars.default_config();
end

task_cfg = cfg.task4;
mars.ensure_profile_data_loaded(cfg);

result_dir = mars.build_results_dir(cfg, task_cfg.name);
runner = mars.create_runner_model(cfg, task_cfg, result_dir);

addpath(result_dir);
cleanup = onCleanup(@() local_cleanup(runner.model_name, result_dir));

load_system(runner.model_file);
stop_time = mars.get_stop_time(runner.model_name);

setpoint_block = local_get_block_path(runner.model_name, {task_cfg.setpoint_block, sprintf('Voltage \nSet Point')});
integrator_block = local_get_block_path(runner.model_name, {task_cfg.integrator_block, sprintf('Discrete-Time\nIntegrator')});

parameter_grid = local_build_parameter_grid(task_cfg);
summary_table = local_empty_summary_table();
sim_outputs = struct([]);

if ~isempty(parameter_grid)
    [summary_table, sim_outputs] = local_run_parameter_sweep( ...
        cfg, task_cfg, runner, stop_time, ...
        setpoint_block, integrator_block, parameter_grid);
end

best_result = local_select_best_result(summary_table, runner, result_dir, stop_time, sim_outputs);

if isfield(best_result, 'parameters')
    local_apply_solution_to_runner(runner, task_cfg, setpoint_block, integrator_block, best_result);
end

fig_handles = local_create_figures(task_cfg, summary_table, best_result, stop_time);

run_results = struct();
run_results.runner = runner;
run_results.stop_time = stop_time;
run_results.parameter_grid = parameter_grid;
run_results.sim_outputs = sim_outputs;

mars.save_task_outputs(cfg, task_cfg.name, result_dir, summary_table, run_results, best_result, fig_handles);

result = struct();
result.result_dir = result_dir;
result.runner = runner;
result.stop_time = stop_time;
result.summary_table = summary_table;
result.best_result = best_result;
result.figure_files = local_build_figure_files(fig_handles, result_dir);
end

function [summary_table, sim_outputs] = local_run_parameter_sweep(cfg, task_cfg, runner, stop_time, setpoint_block, integrator_block, parameter_grid)
num_runs = height(parameter_grid);
sim_inputs(num_runs, 1) = Simulink.SimulationInput(runner.model_name);

for idx = 1:num_runs
    sim_inputs(idx, 1) = Simulink.SimulationInput(runner.model_name);
    sim_inputs(idx, 1) = sim_inputs(idx, 1).setModelParameter('ReturnWorkspaceOutputs', 'on');
    sim_inputs(idx, 1) = sim_inputs(idx, 1).setBlockParameter( ...
        setpoint_block, task_cfg.setpoint_parameter, mars.format_scalar(parameter_grid.setpoint_V(idx)));
    sim_inputs(idx, 1) = sim_inputs(idx, 1).setBlockParameter( ...
        integrator_block, task_cfg.integrator_gain_parameter, mars.format_scalar(parameter_grid.integrator_gain(idx)));
    sim_inputs(idx, 1) = sim_inputs(idx, 1).setBlockParameter( ...
        integrator_block, task_cfg.integrator_ic_parameter, mars.format_scalar(parameter_grid.integrator_ic(idx)));
end

sim_outputs = mars.run_simulations(cfg, sim_inputs);
summary_table = local_build_summary_table(cfg, task_cfg, runner, stop_time, parameter_grid, sim_outputs);
end

function summary_table = local_build_summary_table(cfg, task_cfg, runner, stop_time, parameter_grid, sim_outputs)
num_runs = height(parameter_grid);
run_index = (1:num_runs).';
task4_pass = false(num_runs, 1);
end_time_s = NaN(num_runs, 1);
mission_fraction = zeros(num_runs, 1);
min_voltage_V = NaN(num_runs, 1);
max_voltage_V = NaN(num_runs, 1);
voltage_rms_error_V = NaN(num_runs, 1);
voltage_ripple_V = NaN(num_runs, 1);
under_voltage_V = NaN(num_runs, 1);
over_voltage_V = NaN(num_runs, 1);
duty_mean = NaN(num_runs, 1);
duty_activity = NaN(num_runs, 1);
finalSOC_pct = NaN(num_runs, 1);
minSOC_pct = NaN(num_runs, 1);
maxSOC_pct = NaN(num_runs, 1);
error_message = strings(num_runs, 1);

load_var = local_logging_variable(runner, 'loadVoltage_V', 'mars_loadVoltage_V');
soc_var = local_logging_variable(runner, 'SOC', 'mars_SOC');
duty_var = local_logging_variable(runner, 'duty_cycle', 'mars_duty_cycle');

for idx = 1:num_runs
    sim_out = sim_outputs(idx, 1);
    error_message(idx) = string(mars.get_error_message(sim_out));
    end_time_s(idx) = mars.get_simulation_end_time(sim_out);
    if ~isnan(end_time_s(idx)) && stop_time > 0
        mission_fraction(idx) = min(end_time_s(idx) / stop_time, 1.0);
    end

    voltage_stats = local_signal_stats(mars.get_sim_output_var(sim_out, load_var));
    if voltage_stats.found
        min_voltage_V(idx) = voltage_stats.min_value;
        max_voltage_V(idx) = voltage_stats.max_value;
        voltage_ripple_V(idx) = voltage_stats.max_value - voltage_stats.min_value;
        voltage_rms_error_V(idx) = sqrt(mean((voltage_stats.segment - task_cfg.target_voltage_v).^2));
        under_voltage_V(idx) = max(0, task_cfg.voltage_min - voltage_stats.min_value);
        over_voltage_V(idx) = max(0, voltage_stats.max_value - task_cfg.voltage_max);
    end

    duty_stats = local_signal_stats(mars.get_sim_output_var(sim_out, duty_var));
    if duty_stats.found
        duty_mean(idx) = duty_stats.mean_value;
        duty_activity(idx) = duty_stats.activity;
    end

    soc_stats = local_signal_stats(mars.get_sim_output_var(sim_out, soc_var));
    if soc_stats.found
        soc_values = local_normalize_soc_percent(soc_stats.segment);
        finalSOC_pct(idx) = soc_values(end);
        minSOC_pct(idx) = min(soc_values);
        maxSOC_pct(idx) = max(soc_values);
    end

    task4_pass(idx) = strlength(error_message(idx)) == 0 ...
        && ~isnan(end_time_s(idx)) ...
        && end_time_s(idx) >= stop_time - cfg.simulation.time_tolerance ...
        && isfinite(min_voltage_V(idx)) ...
        && isfinite(max_voltage_V(idx)) ...
        && min_voltage_V(idx) >= task_cfg.voltage_min ...
        && max_voltage_V(idx) <= task_cfg.voltage_max;
end

summary_table = table( ...
    run_index, ...
    parameter_grid.setpoint_V, ...
    parameter_grid.integrator_gain, ...
    parameter_grid.integrator_ic, ...
    task4_pass, ...
    end_time_s, ...
    mission_fraction, ...
    min_voltage_V, ...
    max_voltage_V, ...
    voltage_rms_error_V, ...
    voltage_ripple_V, ...
    under_voltage_V, ...
    over_voltage_V, ...
    duty_mean, ...
    duty_activity, ...
    finalSOC_pct, ...
    minSOC_pct, ...
    maxSOC_pct, ...
    error_message, ...
    'VariableNames', { ...
        'run_index', ...
        'setpoint_V', ...
        'integrator_gain', ...
        'integrator_ic', ...
        'task4_pass', ...
        'end_time_s', ...
        'mission_fraction', ...
        'min_voltage_V', ...
        'max_voltage_V', ...
        'voltage_rms_error_V', ...
        'voltage_ripple_V', ...
        'under_voltage_V', ...
        'over_voltage_V', ...
        'duty_mean', ...
        'duty_activity', ...
        'finalSOC_pct', ...
        'minSOC_pct', ...
        'maxSOC_pct', ...
        'error_message' ...
    });
end

function parameter_grid = local_build_parameter_grid(task_cfg)
[setpoint_grid, gain_grid, ic_grid] = ndgrid( ...
    task_cfg.setpoint_grid(:), ...
    task_cfg.gain_grid(:), ...
    task_cfg.initial_condition_grid(:));

parameter_grid = table( ...
    setpoint_grid(:), ...
    gain_grid(:), ...
    ic_grid(:), ...
    'VariableNames', {'setpoint_V', 'integrator_gain', 'integrator_ic'});
end

function best_result = local_select_best_result(summary_table, runner, result_dir, stop_time, sim_outputs)
best_result = struct();
best_result.result_dir = result_dir;
best_result.runner = runner;
best_result.stop_time = stop_time;
best_result.selection_rule = 'pass threshold first, then minimum RMS voltage error and minimum ripple';

if isempty(summary_table)
    best_result.found = false;
    best_result.message = 'Task 4 parameter grid is empty.';
    return;
end

passing_rows = summary_table(summary_table.task4_pass, :);
if ~isempty(passing_rows)
    ranked_rows = sortrows(passing_rows, {'voltage_rms_error_V', 'voltage_ripple_V', 'duty_activity'}, {'ascend', 'ascend', 'ascend'});
    selected_row = ranked_rows(1, :);
    best_result.found = true;
else
    ranked_rows = sortrows(summary_table, {'under_voltage_V', 'over_voltage_V', 'voltage_rms_error_V', 'voltage_ripple_V'}, {'ascend', 'ascend', 'ascend', 'ascend'});
    selected_row = ranked_rows(1, :);
    best_result.found = false;
    best_result.message = 'No Task 4 candidate held the full 500-520 V band. Returning the closest candidate.';
end

best_result.parameters = table2struct(selected_row, 'ToScalar', true);
selected_index = selected_row.run_index;
best_result.signals = local_collect_task4_signals(runner, sim_outputs(selected_index, 1));
end

function signals = local_collect_task4_signals(runner, sim_out)
signals = struct();
signals.Vbatt = mars.get_sim_output_var(sim_out, local_logging_variable(runner, 'Vbatt', 'mars_Vbatt'));
signals.loadVoltage_V = mars.get_sim_output_var(sim_out, local_logging_variable(runner, 'loadVoltage_V', 'mars_loadVoltage_V'));
signals.SOC = mars.get_sim_output_var(sim_out, local_logging_variable(runner, 'SOC', 'mars_SOC'));
signals.duty_cycle = mars.get_sim_output_var(sim_out, local_logging_variable(runner, 'duty_cycle', 'mars_duty_cycle'));
end

function local_apply_solution_to_runner(runner, task_cfg, setpoint_block, integrator_block, best_result)
load_system(runner.model_file);
set_param(setpoint_block, task_cfg.setpoint_parameter, mars.format_scalar(best_result.parameters.setpoint_V));
set_param(integrator_block, task_cfg.integrator_gain_parameter, mars.format_scalar(best_result.parameters.integrator_gain));
set_param(integrator_block, task_cfg.integrator_ic_parameter, mars.format_scalar(best_result.parameters.integrator_ic));
save_system(runner.model_name, runner.model_file);
end

function fig_handles = local_create_figures(task_cfg, summary_table, best_result, stop_time)
fig_handles = gobjects(0, 1);
fig_handles(end + 1, 1) = local_plot_tuning_scatter(summary_table);
fig_handles(end + 1, 1) = local_plot_top_candidates(summary_table);
fig_handles(end + 1, 1) = local_plot_best_trace(task_cfg, best_result, stop_time);
end

function fig_handle = local_plot_tuning_scatter(summary_table)
fig_handle = figure('Visible', 'off', 'Color', 'w', 'Tag', 'task4_tuning_scatter');
if isempty(summary_table)
    axis off;
    text(0.5, 0.5, 'No Task 4 runs available.', 'HorizontalAlignment', 'center');
    return;
end

gain_data = log10(summary_table.integrator_gain);
scatter(summary_table.voltage_rms_error_V, summary_table.voltage_ripple_V, 60, gain_data, 'filled');
xlabel('Voltage RMS Error (V)');
ylabel('Voltage Ripple (V)');
title('Task 4 Tuning Candidates');
grid on;
color_handle = colorbar();
color_handle.Label.String = 'log10(Integrator Gain)';
end

function fig_handle = local_plot_top_candidates(summary_table)
fig_handle = figure('Visible', 'off', 'Color', 'w', 'Tag', 'task4_top_candidates');
if isempty(summary_table)
    axis off;
    text(0.5, 0.5, 'No Task 4 runs available.', 'HorizontalAlignment', 'center');
    return;
end

ranked_rows = sortrows(summary_table, {'task4_pass', 'voltage_rms_error_V', 'voltage_ripple_V'}, {'descend', 'ascend', 'ascend'});
top_rows = ranked_rows(1:min(10, height(ranked_rows)), :);
bar(categorical(compose('run%d', top_rows.run_index)), top_rows.voltage_rms_error_V);
ylabel('Voltage RMS Error (V)');
title('Task 4 Top Candidates');
grid on;
end

function fig_handle = local_plot_best_trace(task_cfg, best_result, stop_time)
fig_handle = figure('Visible', 'off', 'Color', 'w', 'Tag', 'task4_best_trace');
tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

if ~isfield(best_result, 'parameters')
    nexttile([3, 1]);
    axis off;
    text(0.5, 0.5, 'No Task 4 candidate available.', 'HorizontalAlignment', 'center', 'FontSize', 12);
    return;
end

nexttile;
local_plot_signal(best_result.signals.loadVoltage_V, stop_time, 'Load Voltage (V)');
hold on;
yline(task_cfg.voltage_min, '--', '500 V', 'Color', [0.3, 0.3, 0.3]);
yline(task_cfg.target_voltage_v, ':', '510 V', 'Color', [0.4, 0.4, 0.4]);
yline(task_cfg.voltage_max, '--', '520 V', 'Color', [0.3, 0.3, 0.3]);
hold off;
title(sprintf('Best Task 4 Candidate: Vset=%.0f, gain=%.3g, IC=%.2f', ...
    best_result.parameters.setpoint_V, ...
    best_result.parameters.integrator_gain, ...
    best_result.parameters.integrator_ic));

nexttile;
local_plot_signal(best_result.signals.SOC, stop_time, 'SOC');

nexttile;
local_plot_signal(best_result.signals.duty_cycle, stop_time, 'Duty Cycle');
xlabel('Time (s)');
end

function local_plot_signal(signal, stop_time, y_label)
if signal.found && ~isempty(signal.time) && ~isempty(signal.data)
    plot(signal.time, signal.data, 'LineWidth', 1.3);
    xlim([0, stop_time]);
    ylabel(y_label);
    grid on;
else
    axis off;
    text(0.5, 0.5, ['Signal not available: ', y_label], 'HorizontalAlignment', 'center');
end
end

function stats = local_signal_stats(signal)
stats = struct('found', false, 'segment', [], 'min_value', NaN, 'max_value', NaN, 'mean_value', NaN, 'activity', NaN);
if ~signal.found || isempty(signal.time) || isempty(signal.data)
    return;
end

time_value = signal.time(:);
data_value = signal.data(:);
valid = isfinite(time_value) & isfinite(data_value);
data_value = data_value(valid);
if isempty(data_value)
    return;
end

stats.found = true;
stats.segment = data_value;
stats.min_value = min(data_value);
stats.max_value = max(data_value);
stats.mean_value = mean(data_value);
if numel(data_value) > 1
    stats.activity = mean(abs(diff(data_value)));
else
    stats.activity = 0;
end
end

function soc_values = local_normalize_soc_percent(values)
soc_values = values(:);
if isempty(soc_values)
    return;
end
if max(abs(soc_values)) <= 1.5
    soc_values = soc_values * 100;
end
end

function block_path = local_get_block_path(model_name, candidate_names)
hits = {};
for idx = 1:numel(candidate_names)
    matches = find_system(model_name, ...
        'FollowLinks', 'on', ...
        'LookUnderMasks', 'all', ...
        'SearchDepth', 1, ...
        'RegExp', 'off', ...
        'Name', candidate_names{idx});
    hits = [hits; matches(:)]; %#ok<AGROW>
end

if isempty(hits)
    block_path = mars.find_one_block(model_name, candidate_names);
else
    hits = unique(hits);
    block_path = hits{1};
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

function fig_files = local_build_figure_files(fig_handles, result_dir)
fig_files = strings(0, 1);
for idx = 1:numel(fig_handles)
    if ~ishandle(fig_handles(idx))
        continue;
    end
    fig_files(end + 1, 1) = string(fullfile(result_dir, 'figures', [get(fig_handles(idx), 'Tag'), '.png']));
end
end

function summary_table = local_empty_summary_table()
summary_table = table( ...
    zeros(0, 1), ...
    zeros(0, 1), ...
    zeros(0, 1), ...
    zeros(0, 1), ...
    false(0, 1), ...
    zeros(0, 1), ...
    zeros(0, 1), ...
    zeros(0, 1), ...
    zeros(0, 1), ...
    zeros(0, 1), ...
    zeros(0, 1), ...
    zeros(0, 1), ...
    zeros(0, 1), ...
    zeros(0, 1), ...
    zeros(0, 1), ...
    zeros(0, 1), ...
    zeros(0, 1), ...
    zeros(0, 1), ...
    strings(0, 1), ...
    'VariableNames', { ...
        'run_index', ...
        'setpoint_V', ...
        'integrator_gain', ...
        'integrator_ic', ...
        'task4_pass', ...
        'end_time_s', ...
        'mission_fraction', ...
        'min_voltage_V', ...
        'max_voltage_V', ...
        'voltage_rms_error_V', ...
        'voltage_ripple_V', ...
        'under_voltage_V', ...
        'over_voltage_V', ...
        'duty_mean', ...
        'duty_activity', ...
        'finalSOC_pct', ...
        'minSOC_pct', ...
        'maxSOC_pct', ...
        'error_message' ...
    });
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
