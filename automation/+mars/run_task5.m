function result = run_task5(cfg)
if nargin < 1 || isempty(cfg)
    cfg = mars.default_config();
end

task_cfg = cfg.task5;
mars.ensure_profile_data_loaded(cfg);

[task3_candidates, array_settings, target_power_kw, task3_source] = local_get_task3_candidates(cfg);
[task4_candidates, task4_source] = local_get_task4_candidates(cfg);
[init_soc_pct, soc_source] = local_get_task5_initial_soc(cfg);

result_dir = mars.build_results_dir(cfg, task_cfg.name);
runner = mars.create_runner_model(cfg, task_cfg, result_dir, struct('prepare_fcn', @mars.create_task5_runner_model));

addpath(result_dir);
cleanup = onCleanup(@() local_cleanup(runner.model_name, result_dir));

load_system(runner.model_file);
stop_time = mars.get_stop_time(runner.model_name);

solar_block = local_get_block_path([runner.model_name, '/', task_cfg.solar_subsystem], {task_cfg.solar_block});
mppt_block = local_get_block_path([runner.model_name, '/', task_cfg.solar_subsystem], {task_cfg.mppt_block});
mppt_constants = mars.discover_mppt_constant_blocks(mppt_block);

battery_root = [runner.model_name, '/', task_cfg.battery_subsystem];
setpoint_block = local_get_block_path(battery_root, {task_cfg.setpoint_block, sprintf('Voltage \nSet Point')});
integrator_block = local_get_block_path(battery_root, {task_cfg.integrator_block, sprintf('Discrete-Time\nIntegrator')});
battery_cell_block = local_get_block_path(battery_root, {task_cfg.battery_cell_block, sprintf('Battery\n(Table-Based)'), sprintf('Battery \n(Table-Based)')});

combo_table = local_build_combo_table(task3_candidates, task4_candidates, cfg.task5.top_k, array_settings, init_soc_pct);
summary_table = local_empty_summary_table();
sim_outputs = struct([]);

if ~isempty(combo_table)
    [summary_table, sim_outputs] = local_run_combo_sweep( ...
        cfg, task_cfg, runner, stop_time, ...
        solar_block, mppt_constants, setpoint_block, integrator_block, battery_cell_block, ...
        target_power_kw, combo_table);
end

best_result = local_select_best_result(summary_table, runner, result_dir, stop_time, target_power_kw, task3_source, task4_source, soc_source, sim_outputs);

if isfield(best_result, 'parameters')
    local_apply_solution_to_runner( ...
        runner, task_cfg, solar_block, mppt_constants, setpoint_block, integrator_block, battery_cell_block, best_result);
end

fig_handles = local_create_figures(task_cfg, summary_table, best_result, stop_time, target_power_kw);

run_results = struct();
run_results.runner = runner;
run_results.stop_time = stop_time;
run_results.target_power_kw = target_power_kw;
run_results.task3_source = task3_source;
run_results.task4_source = task4_source;
run_results.soc_source = soc_source;
run_results.combo_table = combo_table;
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

function combo_table = local_build_combo_table(task3_candidates, task4_candidates, top_k, array_settings, init_soc_pct)
if isempty(task3_candidates) || isempty(task4_candidates)
    combo_table = local_empty_combo_table();
    return;
end

task3_candidates = task3_candidates(1:min(top_k, height(task3_candidates)), :);
task4_candidates = task4_candidates(1:min(top_k, height(task4_candidates)), :);

num_rows = height(task3_candidates) * height(task4_candidates);
combo_index = zeros(num_rows, 1);
task3_run_index = zeros(num_rows, 1);
task4_run_index = zeros(num_rows, 1);
N_series = zeros(num_rows, 1);
N_parallel = zeros(num_rows, 1);
Dinit = zeros(num_rows, 1);
Dmax = zeros(num_rows, 1);
Dmin = zeros(num_rows, 1);
deltaD = zeros(num_rows, 1);
setpoint_V = zeros(num_rows, 1);
integrator_gain = zeros(num_rows, 1);
integrator_ic = zeros(num_rows, 1);
initSOC_pct = repmat(init_soc_pct, num_rows, 1);

row_idx = 0;
for idx3 = 1:height(task3_candidates)
    for idx4 = 1:height(task4_candidates)
        row_idx = row_idx + 1;
        combo_index(row_idx) = row_idx;
        task3_run_index(row_idx) = task3_candidates.run_index(idx3);
        task4_run_index(row_idx) = task4_candidates.run_index(idx4);
        N_series(row_idx) = array_settings.N_series;
        N_parallel(row_idx) = array_settings.N_parallel;
        Dinit(row_idx) = task3_candidates.Dinit(idx3);
        Dmax(row_idx) = task3_candidates.Dmax(idx3);
        Dmin(row_idx) = task3_candidates.Dmin(idx3);
        deltaD(row_idx) = task3_candidates.deltaD(idx3);
        setpoint_V(row_idx) = task4_candidates.setpoint_V(idx4);
        integrator_gain(row_idx) = task4_candidates.integrator_gain(idx4);
        integrator_ic(row_idx) = task4_candidates.integrator_ic(idx4);
    end
end

combo_table = table( ...
    combo_index, ...
    task3_run_index, ...
    task4_run_index, ...
    N_series, ...
    N_parallel, ...
    Dinit, ...
    Dmax, ...
    Dmin, ...
    deltaD, ...
    setpoint_V, ...
    integrator_gain, ...
    integrator_ic, ...
    initSOC_pct, ...
    'VariableNames', { ...
        'combo_index', ...
        'task3_run_index', ...
        'task4_run_index', ...
        'N_series', ...
        'N_parallel', ...
        'Dinit', ...
        'Dmax', ...
        'Dmin', ...
        'deltaD', ...
        'setpoint_V', ...
        'integrator_gain', ...
        'integrator_ic', ...
        'initSOC_pct' ...
    });
end

function [summary_table, sim_outputs] = local_run_combo_sweep(cfg, task_cfg, runner, stop_time, solar_block, mppt_constants, setpoint_block, integrator_block, battery_cell_block, target_power_kw, combo_table)
num_runs = height(combo_table);
sim_inputs(num_runs, 1) = Simulink.SimulationInput(runner.model_name);

for idx = 1:num_runs
    sim_inputs(idx, 1) = Simulink.SimulationInput(runner.model_name);
    sim_inputs(idx, 1) = sim_inputs(idx, 1).setModelParameter('ReturnWorkspaceOutputs', 'on');
    sim_inputs(idx, 1) = sim_inputs(idx, 1).setBlockParameter( ...
        solar_block, task_cfg.series_parameter, mars.format_scalar(combo_table.N_series(idx)));
    sim_inputs(idx, 1) = sim_inputs(idx, 1).setBlockParameter( ...
        solar_block, task_cfg.parallel_parameter, mars.format_scalar(combo_table.N_parallel(idx)));
    sim_inputs(idx, 1) = sim_inputs(idx, 1).setBlockParameter(mppt_constants{1}, 'Value', mars.format_scalar(combo_table.Dinit(idx)));
    sim_inputs(idx, 1) = sim_inputs(idx, 1).setBlockParameter(mppt_constants{2}, 'Value', mars.format_scalar(combo_table.Dmax(idx)));
    sim_inputs(idx, 1) = sim_inputs(idx, 1).setBlockParameter(mppt_constants{3}, 'Value', mars.format_scalar(combo_table.Dmin(idx)));
    sim_inputs(idx, 1) = sim_inputs(idx, 1).setBlockParameter(mppt_constants{4}, 'Value', mars.format_scalar(combo_table.deltaD(idx)));
    sim_inputs(idx, 1) = sim_inputs(idx, 1).setBlockParameter(setpoint_block, task_cfg.setpoint_parameter, mars.format_scalar(combo_table.setpoint_V(idx)));
    sim_inputs(idx, 1) = sim_inputs(idx, 1).setBlockParameter(integrator_block, task_cfg.integrator_gain_parameter, mars.format_scalar(combo_table.integrator_gain(idx)));
    sim_inputs(idx, 1) = sim_inputs(idx, 1).setBlockParameter(integrator_block, task_cfg.integrator_ic_parameter, mars.format_scalar(combo_table.integrator_ic(idx)));
    sim_inputs(idx, 1) = sim_inputs(idx, 1).setBlockParameter(battery_cell_block, task_cfg.state_of_charge_parameter, mars.format_scalar(combo_table.initSOC_pct(idx) / 100));
end

sim_outputs = mars.run_simulations(cfg, sim_inputs);
summary_table = local_build_summary_table(cfg, task_cfg, runner, stop_time, target_power_kw, combo_table, sim_outputs);
end

function summary_table = local_build_summary_table(cfg, task_cfg, runner, stop_time, target_power_kw, combo_table, sim_outputs)
num_runs = height(combo_table);
task5_pass = false(num_runs, 1);
end_time_s = NaN(num_runs, 1);
mission_fraction = zeros(num_runs, 1);
min_voltage_V = NaN(num_runs, 1);
max_voltage_V = NaN(num_runs, 1);
voltage_rms_error_V = NaN(num_runs, 1);
voltage_ripple_V = NaN(num_runs, 1);
under_voltage_V = NaN(num_runs, 1);
over_voltage_V = NaN(num_runs, 1);
Ps_mean_kW = NaN(num_runs, 1);
finalSOC_pct = NaN(num_runs, 1);
minSOC_pct = NaN(num_runs, 1);
maxSOC_pct = NaN(num_runs, 1);
error_message = strings(num_runs, 1);

voltage_var = local_logging_variable(runner, 'loadVoltage_V', 'mars_loadVoltage_V');
soc_var = local_logging_variable(runner, 'SOC', 'mars_SOC');
ps_var = local_logging_variable(runner, 'Ps_kW', 'mars_Ps_kW');

for idx = 1:num_runs
    sim_out = sim_outputs(idx, 1);
    error_message(idx) = string(mars.get_error_message(sim_out));
    end_time_s(idx) = mars.get_simulation_end_time(sim_out);
    if ~isnan(end_time_s(idx)) && stop_time > 0
        mission_fraction(idx) = min(end_time_s(idx) / stop_time, 1.0);
    end

    voltage_stats = local_signal_stats(mars.get_sim_output_var(sim_out, voltage_var));
    if voltage_stats.found
        min_voltage_V(idx) = voltage_stats.min_value;
        max_voltage_V(idx) = voltage_stats.max_value;
        voltage_ripple_V(idx) = voltage_stats.max_value - voltage_stats.min_value;
        voltage_rms_error_V(idx) = sqrt(mean((voltage_stats.segment - task_cfg.target_voltage_v).^2));
        under_voltage_V(idx) = max(0, task_cfg.voltage_min - voltage_stats.min_value);
        over_voltage_V(idx) = max(0, voltage_stats.max_value - task_cfg.voltage_max);
    end

    soc_stats = local_signal_stats(mars.get_sim_output_var(sim_out, soc_var));
    if soc_stats.found
        soc_values = local_normalize_soc_percent(soc_stats.segment);
        finalSOC_pct(idx) = soc_values(end);
        minSOC_pct(idx) = min(soc_values);
        maxSOC_pct(idx) = max(soc_values);
    end

    ps_stats = local_signal_stats(mars.get_sim_output_var(sim_out, ps_var));
    if ps_stats.found
        Ps_mean_kW(idx) = ps_stats.mean_value;
    end

    task5_pass(idx) = strlength(error_message(idx)) == 0 ...
        && ~isnan(end_time_s(idx)) ...
        && end_time_s(idx) >= stop_time - cfg.simulation.time_tolerance ...
        && isfinite(min_voltage_V(idx)) ...
        && isfinite(max_voltage_V(idx)) ...
        && min_voltage_V(idx) >= task_cfg.voltage_min ...
        && max_voltage_V(idx) <= task_cfg.voltage_max ...
        && isfinite(minSOC_pct(idx)) ...
        && isfinite(maxSOC_pct(idx)) ...
        && minSOC_pct(idx) >= task_cfg.soc_min ...
        && maxSOC_pct(idx) <= task_cfg.soc_max;
end

summary_table = [combo_table, table( ...
    task5_pass, ...
    end_time_s, ...
    mission_fraction, ...
    min_voltage_V, ...
    max_voltage_V, ...
    voltage_rms_error_V, ...
    voltage_ripple_V, ...
    under_voltage_V, ...
    over_voltage_V, ...
    Ps_mean_kW, ...
    finalSOC_pct, ...
    minSOC_pct, ...
    maxSOC_pct, ...
    error_message)];
end

function best_result = local_select_best_result(summary_table, runner, result_dir, stop_time, target_power_kw, task3_source, task4_source, soc_source, sim_outputs)
best_result = struct();
best_result.result_dir = result_dir;
best_result.runner = runner;
best_result.stop_time = stop_time;
best_result.target_power_kw = target_power_kw;
best_result.task3_source = task3_source;
best_result.task4_source = task4_source;
best_result.soc_source = soc_source;
best_result.selection_rule = 'pass threshold first, then maximum final SOC and minimum voltage ripple';

if isempty(summary_table)
    best_result.found = false;
    best_result.message = 'Task 5 candidate grid is empty.';
    return;
end

passing_rows = summary_table(summary_table.task5_pass, :);
if ~isempty(passing_rows)
    ranked_rows = sortrows(passing_rows, {'finalSOC_pct', 'voltage_ripple_V', 'voltage_rms_error_V'}, {'descend', 'ascend', 'ascend'});
    selected_row = ranked_rows(1, :);
    best_result.found = true;
else
    ranked_rows = sortrows(summary_table, {'under_voltage_V', 'over_voltage_V', 'mission_fraction', 'finalSOC_pct'}, {'ascend', 'ascend', 'descend', 'descend'});
    selected_row = ranked_rows(1, :);
    best_result.found = false;
    best_result.message = 'No Task 5 integrated candidate fully passed. Returning the closest candidate.';
end

best_result.parameters = table2struct(selected_row, 'ToScalar', true);
selected_index = selected_row.combo_index;
best_result.signals = local_collect_task5_signals(runner, sim_outputs(selected_index, 1));
end

function signals = local_collect_task5_signals(runner, sim_out)
signals = struct();
signals.loadVoltage_V = mars.get_sim_output_var(sim_out, local_logging_variable(runner, 'loadVoltage_V', 'mars_loadVoltage_V'));
signals.Vbatt = mars.get_sim_output_var(sim_out, local_logging_variable(runner, 'Vbatt', 'mars_Vbatt'));
signals.SOC = mars.get_sim_output_var(sim_out, local_logging_variable(runner, 'SOC', 'mars_SOC'));
signals.Ps_kW = mars.get_sim_output_var(sim_out, local_logging_variable(runner, 'Ps_kW', 'mars_Ps_kW'));
signals.duty_cycle = mars.get_sim_output_var(sim_out, local_logging_variable(runner, 'duty_cycle', 'mars_duty_cycle'));
end

function local_apply_solution_to_runner(runner, task_cfg, solar_block, mppt_constants, setpoint_block, integrator_block, battery_cell_block, best_result)
load_system(runner.model_file);
set_param(solar_block, task_cfg.series_parameter, mars.format_scalar(best_result.parameters.N_series));
set_param(solar_block, task_cfg.parallel_parameter, mars.format_scalar(best_result.parameters.N_parallel));
set_param(mppt_constants{1}, 'Value', mars.format_scalar(best_result.parameters.Dinit));
set_param(mppt_constants{2}, 'Value', mars.format_scalar(best_result.parameters.Dmax));
set_param(mppt_constants{3}, 'Value', mars.format_scalar(best_result.parameters.Dmin));
set_param(mppt_constants{4}, 'Value', mars.format_scalar(best_result.parameters.deltaD));
set_param(setpoint_block, task_cfg.setpoint_parameter, mars.format_scalar(best_result.parameters.setpoint_V));
set_param(integrator_block, task_cfg.integrator_gain_parameter, mars.format_scalar(best_result.parameters.integrator_gain));
set_param(integrator_block, task_cfg.integrator_ic_parameter, mars.format_scalar(best_result.parameters.integrator_ic));
set_param(battery_cell_block, task_cfg.state_of_charge_parameter, mars.format_scalar(best_result.parameters.initSOC_pct / 100));
save_system(runner.model_name, runner.model_file);
end

function fig_handles = local_create_figures(task_cfg, summary_table, best_result, stop_time, target_power_kw)
fig_handles = gobjects(0, 1);
fig_handles(end + 1, 1) = local_plot_candidate_ranking(summary_table);
fig_handles(end + 1, 1) = local_plot_best_dashboard(task_cfg, best_result, stop_time, target_power_kw);
fig_handles(end + 1, 1) = local_plot_pass_banner(best_result);
end

function fig_handle = local_plot_candidate_ranking(summary_table)
fig_handle = figure('Visible', 'off', 'Color', 'w', 'Tag', 'task5_candidate_ranking');
if isempty(summary_table)
    axis off;
    text(0.5, 0.5, 'No Task 5 runs available.', 'HorizontalAlignment', 'center');
    return;
end

scatter(summary_table.voltage_ripple_V, summary_table.finalSOC_pct, 70, double(summary_table.task5_pass), 'filled');
xlabel('Voltage Ripple (V)');
ylabel('Final SOC (%)');
title('Task 5 Integrated Candidates');
grid on;
colormap(fig_handle, [0.80, 0.27, 0.27; 0.22, 0.66, 0.33]);
color_handle = colorbar('Ticks', [0, 1]);
color_handle.TickLabels = {'Fail', 'Pass'};
end

function fig_handle = local_plot_best_dashboard(task_cfg, best_result, stop_time, target_power_kw)
fig_handle = figure('Visible', 'off', 'Color', 'w', 'Tag', 'task5_best_dashboard');
tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

if ~isfield(best_result, 'parameters')
    nexttile([4, 1]);
    axis off;
    text(0.5, 0.5, 'No Task 5 candidate available.', 'HorizontalAlignment', 'center', 'FontSize', 12);
    return;
end

nexttile;
local_plot_signal(best_result.signals.loadVoltage_V, stop_time, 'Load Voltage (V)');
hold on;
yline(task_cfg.voltage_min, '--', '500 V', 'Color', [0.3, 0.3, 0.3]);
yline(task_cfg.target_voltage_v, ':', '510 V', 'Color', [0.4, 0.4, 0.4]);
yline(task_cfg.voltage_max, '--', '520 V', 'Color', [0.3, 0.3, 0.3]);
hold off;
title(sprintf('Best Task 5 Candidate: T3 run %d, T4 run %d', best_result.parameters.task3_run_index, best_result.parameters.task4_run_index));

nexttile;
local_plot_signal(best_result.signals.SOC, stop_time, 'SOC');

nexttile;
local_plot_signal(best_result.signals.Ps_kW, stop_time, 'Ps (kW)');
if isfinite(target_power_kw)
    hold on;
    yline(target_power_kw, '--', 'Target', 'Color', [0.3, 0.3, 0.3]);
    hold off;
end

nexttile;
local_plot_signal(best_result.signals.duty_cycle, stop_time, 'Duty Cycle');
xlabel('Time (s)');
end

function fig_handle = local_plot_pass_banner(best_result)
fig_handle = figure('Visible', 'off', 'Color', 'w', 'Tag', 'task5_pass_banner');
axis off;
if best_result.found
    text(0.5, 0.6, 'TASK 5 PASSING CANDIDATE FOUND', ...
        'HorizontalAlignment', 'center', ...
        'FontWeight', 'bold', ...
        'FontSize', 18, ...
        'Color', [0.15, 0.45, 0.18]);
else
    text(0.5, 0.6, 'TASK 5 NOT YET PASSING', ...
        'HorizontalAlignment', 'center', ...
        'FontWeight', 'bold', ...
        'FontSize', 18, ...
        'Color', [0.65, 0.16, 0.16]);
    if isfield(best_result, 'message')
        text(0.5, 0.4, best_result.message, 'HorizontalAlignment', 'center');
    end
end
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
stats = struct('found', false, 'segment', [], 'min_value', NaN, 'max_value', NaN, 'mean_value', NaN);
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

function [task3_candidates, array_settings, target_power_kw, source_note] = local_get_task3_candidates(cfg)
task3_candidates = table();
array_settings = struct();
source_note = '';

task5_overrides = local_get_task5_overrides(cfg);
if local_has_task3_override(task5_overrides)
    task3_candidates = table( ...
        1, ...
        task5_overrides.Dinit, ...
        task5_overrides.Dmax, ...
        task5_overrides.Dmin, ...
        task5_overrides.deltaD, ...
        true, ...
        1, ...
        0, ...
        0, ...
        0, ...
        'VariableNames', {'run_index', 'Dinit', 'Dmax', 'Dmin', 'deltaD', 'task3_pass', 'power_fraction', 'Ps_ripple_kW', 'duty_activity', 'rail_fraction'});
    array_settings.N_series = task5_overrides.N_series;
    array_settings.N_parallel = task5_overrides.N_parallel;
    if isfield(task5_overrides, 'target_power_kw') && ~isempty(task5_overrides.target_power_kw)
        target_power_kw = task5_overrides.target_power_kw;
    elseif isfield(task5_overrides, 'array_mpp_power_kw') && ~isempty(task5_overrides.array_mpp_power_kw)
        target_power_kw = task5_overrides.array_mpp_power_kw;
    elseif isfield(task5_overrides, 'array_mpp_power_w') && ~isempty(task5_overrides.array_mpp_power_w)
        target_power_kw = task5_overrides.array_mpp_power_w / 1000;
    else
        error('mars:Task5NeedsTask3Power', ...
            'Task 5 override mode needs target_power_kw, array_mpp_power_kw, or array_mpp_power_w.');
    end
    source_note = 'cfg.overrides.task5 Task 3 fields';
    return;
end

task3_payload = mars.load_latest_all_runs(cfg, 'task3');
task3_summary = task3_payload.summary_table;
task3_best = mars.load_latest_best_result(cfg, 'task3');

if ~istable(task3_summary) || isempty(task3_summary)
    if isfield(task3_best, 'parameters')
        task3_candidates = struct2table(task3_best.parameters, 'AsArray', true);
    else
        error('mars:Task5NeedsTask3', 'Task 5 needs Task 3 results or cfg.overrides.task5 Task 3 fields.');
    end
else
    task3_candidates = sortrows(task3_summary, {'task3_pass', 'power_fraction', 'Ps_ripple_kW', 'duty_activity'}, {'descend', 'descend', 'ascend', 'ascend'});
end

if isfield(task3_best, 'array_settings')
    array_settings = task3_best.array_settings;
else
    error('mars:Task5NeedsTask3Array', 'Task 5 needs Task 3 best_result.array_settings or cfg.overrides.task5 N_series/N_parallel.');
end

if isfield(task3_best, 'target_power_kw') && ~isempty(task3_best.target_power_kw)
    target_power_kw = task3_best.target_power_kw;
else
    error('mars:Task5NeedsTask3Power', 'Task 5 needs Task 3 best_result.target_power_kw or cfg.overrides.task5 power overrides.');
end

source_note = 'Task 3 all_runs + best_result';
end

function [task4_candidates, source_note] = local_get_task4_candidates(cfg)
task4_candidates = table();
source_note = '';

task5_overrides = local_get_task5_overrides(cfg);
if local_has_task4_override(task5_overrides)
    task4_candidates = table( ...
        1, ...
        task5_overrides.setpoint_V, ...
        task5_overrides.integrator_gain, ...
        task5_overrides.integrator_ic, ...
        true, ...
        0, ...
        0, ...
        'VariableNames', {'run_index', 'setpoint_V', 'integrator_gain', 'integrator_ic', 'task4_pass', 'voltage_rms_error_V', 'voltage_ripple_V'});
    source_note = 'cfg.overrides.task5 Task 4 fields';
    return;
end

task4_payload = mars.load_latest_all_runs(cfg, 'task4');
task4_summary = task4_payload.summary_table;
task4_best = mars.load_latest_best_result(cfg, 'task4');

if ~istable(task4_summary) || isempty(task4_summary)
    if isfield(task4_best, 'parameters')
        task4_candidates = struct2table(task4_best.parameters, 'AsArray', true);
    else
        error('mars:Task5NeedsTask4', 'Task 5 needs Task 4 results or cfg.overrides.task5 Task 4 fields.');
    end
else
    task4_candidates = sortrows(task4_summary, {'task4_pass', 'voltage_rms_error_V', 'voltage_ripple_V'}, {'descend', 'ascend', 'ascend'});
end

source_note = 'Task 4 all_runs + best_result';
end

function [init_soc_pct, source_note] = local_get_task5_initial_soc(cfg)
task5_overrides = local_get_task5_overrides(cfg);
if isfield(task5_overrides, 'init_soc_pct') && ~isempty(task5_overrides.init_soc_pct)
    init_soc_pct = task5_overrides.init_soc_pct;
    source_note = 'cfg.overrides.task5.init_soc_pct';
    return;
end

best_task1 = mars.load_latest_best_result(cfg, 'task1');
if isfield(best_task1, 'found') && best_task1.found && isfield(best_task1, 'initSOC_pct')
    init_soc_pct = best_task1.initSOC_pct;
    source_note = 'Task 1 best_result.initSOC_pct';
    return;
end

error('mars:Task5NeedsTask1SOC', 'Task 5 needs Task 1 best_result.initSOC_pct or cfg.overrides.task5.init_soc_pct.');
end

function overrides = local_get_task5_overrides(cfg)
overrides = struct();
if isfield(cfg, 'overrides') && isstruct(cfg.overrides) ...
        && isfield(cfg.overrides, 'task5') && isstruct(cfg.overrides.task5)
    overrides = cfg.overrides.task5;
end
end

function tf = local_has_task3_override(overrides)
required_fields = {'N_series', 'N_parallel', 'Dinit', 'Dmax', 'Dmin', 'deltaD'};
tf = true;
for idx = 1:numel(required_fields)
    if ~isfield(overrides, required_fields{idx}) || isempty(overrides.(required_fields{idx}))
        tf = false;
        return;
    end
end
end

function tf = local_has_task4_override(overrides)
required_fields = {'setpoint_V', 'integrator_gain', 'integrator_ic'};
tf = true;
for idx = 1:numel(required_fields)
    if ~isfield(overrides, required_fields{idx}) || isempty(overrides.(required_fields{idx}))
        tf = false;
        return;
    end
end
end

function block_path = local_get_block_path(root_path, candidate_names)
hits = {};
for idx = 1:numel(candidate_names)
    matches = find_system(root_path, ...
        'FollowLinks', 'on', ...
        'LookUnderMasks', 'all', ...
        'SearchDepth', 1, ...
        'RegExp', 'off', ...
        'Name', candidate_names{idx});
    hits = [hits; matches(:)]; %#ok<AGROW>
end

if isempty(hits)
    block_path = mars.find_one_block(root_path, candidate_names);
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

function combo_table = local_empty_combo_table()
combo_table = table( ...
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
    'VariableNames', { ...
        'combo_index', ...
        'task3_run_index', ...
        'task4_run_index', ...
        'N_series', ...
        'N_parallel', ...
        'Dinit', ...
        'Dmax', ...
        'Dmin', ...
        'deltaD', ...
        'setpoint_V', ...
        'integrator_gain', ...
        'integrator_ic', ...
        'initSOC_pct' ...
    });
end

function summary_table = local_empty_summary_table()
summary_table = [local_empty_combo_table(), table( ...
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
    strings(0, 1), ...
    'VariableNames', { ...
        'task5_pass', ...
        'end_time_s', ...
        'mission_fraction', ...
        'min_voltage_V', ...
        'max_voltage_V', ...
        'voltage_rms_error_V', ...
        'voltage_ripple_V', ...
        'under_voltage_V', ...
        'over_voltage_V', ...
        'Ps_mean_kW', ...
        'finalSOC_pct', ...
        'minSOC_pct', ...
        'maxSOC_pct', ...
        'error_message' ...
    })];
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
