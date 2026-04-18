function result = run_task3(cfg)
if nargin < 1 || isempty(cfg)
    cfg = mars.default_config();
end

task_cfg = cfg.task3;
debug_prints = local_get_task3_setting(cfg, task_cfg, 'debug_prints', true);
max_debug_failures = local_get_task3_setting(cfg, task_cfg, 'max_debug_failures', 5);

if debug_prints && ~cfg.parallel.show_progress
    cfg.parallel.show_progress = true;
end

parallel_enabled = mars.detect_parallel_support(cfg);
mars.ensure_profile_data_loaded(cfg);

[array_settings, target_power_kw, source_note] = local_get_task3_inputs(cfg);
result_dir = mars.build_results_dir(cfg, task_cfg.name);
runner = mars.create_runner_model(cfg, task_cfg, result_dir);

addpath(result_dir);
cleanup = onCleanup(@() local_cleanup(runner.model_name, result_dir));

load_system(runner.model_file);
stop_time = mars.get_stop_time(runner.model_name);
steady_state_start = max(task_cfg.enable_time_s + task_cfg.settling_time_s, cfg.simulation.task3_steady_state_start);

solar_block = local_get_block_path(runner.model_name, task_cfg.solar_block);
mppt_block = local_get_block_path(runner.model_name, task_cfg.mppt_block);
mppt_constants = mars.discover_mppt_constant_blocks(mppt_block);

parameter_grid = local_build_parameter_grid(task_cfg);
summary_table = local_empty_summary_table();
sim_outputs = struct([]);

if debug_prints
    local_print_task3_start( ...
        task_cfg, ...
        runner, ...
        result_dir, ...
        stop_time, ...
        steady_state_start, ...
        parallel_enabled, ...
        array_settings, ...
        target_power_kw, ...
        source_note, ...
        parameter_grid);
end

if ~isempty(parameter_grid)
    if debug_prints
        fprintf('[task3] Starting parameter sweep with %d candidates.\n', height(parameter_grid));
    end

    [summary_table, sim_outputs] = local_run_parameter_sweep( ...
        cfg, task_cfg, runner, stop_time, steady_state_start, ...
        solar_block, mppt_constants, array_settings, target_power_kw, parameter_grid);
elseif debug_prints
    fprintf('[task3] Parameter sweep skipped because the filtered parameter grid is empty.\n');
end

best_result = local_select_best_result(summary_table, runner, result_dir, stop_time, array_settings, target_power_kw, source_note, sim_outputs);

if debug_prints
    local_print_task3_summary(summary_table, best_result, result_dir, max_debug_failures);
end

if isfield(best_result, 'parameters')
    local_apply_solution_to_runner(runner, task_cfg, solar_block, mppt_constants, best_result);
end

fig_handles = local_create_figures(summary_table, best_result, stop_time, target_power_kw);

run_results = struct();
run_results.runner = runner;
run_results.stop_time = stop_time;
run_results.steady_state_start = steady_state_start;
run_results.array_settings = array_settings;
run_results.target_power_kw = target_power_kw;
run_results.target_source = source_note;
run_results.parameter_grid = parameter_grid;
run_results.sim_outputs = sim_outputs;

mars.save_task_outputs(cfg, task_cfg.name, result_dir, summary_table, run_results, best_result, fig_handles);

if debug_prints
    fprintf('[task3] Saved outputs to %s\n', result_dir);
end

result = struct();
result.result_dir = result_dir;
result.runner = runner;
result.stop_time = stop_time;
result.steady_state_start = steady_state_start;
result.summary_table = summary_table;
result.best_result = best_result;
result.figure_files = local_build_figure_files(fig_handles, result_dir);
end

function [summary_table, sim_outputs] = local_run_parameter_sweep(cfg, task_cfg, runner, stop_time, steady_state_start, solar_block, mppt_constants, array_settings, target_power_kw, parameter_grid)
num_runs = height(parameter_grid);
sim_inputs(num_runs, 1) = Simulink.SimulationInput(runner.model_name);

for idx = 1:num_runs
    sim_inputs(idx, 1) = Simulink.SimulationInput(runner.model_name);
    sim_inputs(idx, 1) = sim_inputs(idx, 1).setModelParameter('ReturnWorkspaceOutputs', 'on');
    sim_inputs(idx, 1) = sim_inputs(idx, 1).setBlockParameter( ...
        solar_block, ...
        task_cfg.series_parameter, mars.format_scalar(array_settings.N_series));
    sim_inputs(idx, 1) = sim_inputs(idx, 1).setBlockParameter( ...
        solar_block, ...
        task_cfg.parallel_parameter, mars.format_scalar(array_settings.N_parallel));
    sim_inputs(idx, 1) = sim_inputs(idx, 1).setBlockParameter( ...
        mppt_constants{1}, 'Value', mars.format_scalar(parameter_grid.Dinit(idx)));
    sim_inputs(idx, 1) = sim_inputs(idx, 1).setBlockParameter( ...
        mppt_constants{2}, 'Value', mars.format_scalar(parameter_grid.Dmax(idx)));
    sim_inputs(idx, 1) = sim_inputs(idx, 1).setBlockParameter( ...
        mppt_constants{3}, 'Value', mars.format_scalar(parameter_grid.Dmin(idx)));
    sim_inputs(idx, 1) = sim_inputs(idx, 1).setBlockParameter( ...
        mppt_constants{4}, 'Value', mars.format_scalar(parameter_grid.deltaD(idx)));
end

sim_outputs = mars.run_simulations(cfg, sim_inputs);
summary_table = local_build_summary_table(cfg, task_cfg, runner, stop_time, steady_state_start, target_power_kw, parameter_grid, sim_outputs);
end

function summary_table = local_build_summary_table(cfg, task_cfg, runner, stop_time, steady_state_start, target_power_kw, parameter_grid, sim_outputs)
num_runs = height(parameter_grid);
run_index = (1:num_runs).';
task3_pass = false(num_runs, 1);
end_time_s = NaN(num_runs, 1);
mission_fraction = zeros(num_runs, 1);
Ps_mean_kW = NaN(num_runs, 1);
Ps_peak_kW = NaN(num_runs, 1);
Ps_ripple_kW = NaN(num_runs, 1);
power_fraction = NaN(num_runs, 1);
duty_mean = NaN(num_runs, 1);
duty_ripple = NaN(num_runs, 1);
duty_activity = NaN(num_runs, 1);
rail_fraction = NaN(num_runs, 1);
error_message = strings(num_runs, 1);

ps_var = local_logging_variable(runner, 'Ps_kW', 'mars_Ps_kW');
duty_var = local_logging_variable(runner, 'duty_cycle', 'mars_duty_cycle');

for idx = 1:num_runs
    sim_out = sim_outputs(idx, 1);
    error_message(idx) = string(mars.get_error_message(sim_out));
    end_time_s(idx) = mars.get_simulation_end_time(sim_out);
    if ~isnan(end_time_s(idx)) && stop_time > 0
        mission_fraction(idx) = min(end_time_s(idx) / stop_time, 1.0);
    end

    ps_stats = local_steady_state_stats(mars.get_sim_output_var(sim_out, ps_var), steady_state_start);
    duty_stats = local_steady_state_stats(mars.get_sim_output_var(sim_out, duty_var), steady_state_start);

    Ps_mean_kW(idx) = ps_stats.mean_value;
    Ps_peak_kW(idx) = ps_stats.peak_value;
    Ps_ripple_kW(idx) = ps_stats.ripple;
    duty_mean(idx) = duty_stats.mean_value;
    duty_ripple(idx) = duty_stats.ripple;
    duty_activity(idx) = duty_stats.activity;

    if isfinite(target_power_kw) && target_power_kw > 0 && isfinite(Ps_mean_kW(idx))
        power_fraction(idx) = Ps_mean_kW(idx) / target_power_kw;
    end

    if duty_stats.found
        rail_fraction(idx) = mean(duty_stats.segment <= parameter_grid.Dmin(idx) + task_cfg.duty_tolerance ...
            | duty_stats.segment >= parameter_grid.Dmax(idx) - task_cfg.duty_tolerance);
    end

    task3_pass(idx) = strlength(error_message(idx)) == 0 ...
        && ~isnan(end_time_s(idx)) ...
        && end_time_s(idx) >= stop_time - cfg.simulation.time_tolerance ...
        && isfinite(power_fraction(idx)) ...
        && power_fraction(idx) >= task_cfg.power_fraction_limit ...
        && isfinite(rail_fraction(idx)) ...
        && rail_fraction(idx) <= task_cfg.rail_fraction_limit;
end

summary_table = table( ...
    run_index, ...
    parameter_grid.Dinit, ...
    parameter_grid.Dmax, ...
    parameter_grid.Dmin, ...
    parameter_grid.deltaD, ...
    task3_pass, ...
    end_time_s, ...
    mission_fraction, ...
    Ps_mean_kW, ...
    Ps_peak_kW, ...
    Ps_ripple_kW, ...
    power_fraction, ...
    duty_mean, ...
    duty_ripple, ...
    duty_activity, ...
    rail_fraction, ...
    error_message, ...
    'VariableNames', { ...
        'run_index', ...
        'Dinit', ...
        'Dmax', ...
        'Dmin', ...
        'deltaD', ...
        'task3_pass', ...
        'end_time_s', ...
        'mission_fraction', ...
        'Ps_mean_kW', ...
        'Ps_peak_kW', ...
        'Ps_ripple_kW', ...
        'power_fraction', ...
        'duty_mean', ...
        'duty_ripple', ...
        'duty_activity', ...
        'rail_fraction', ...
        'error_message' ...
    });
end

function parameter_grid = local_build_parameter_grid(task_cfg)
[dinit_grid, dmax_grid, dmin_grid, delta_grid] = ndgrid( ...
    task_cfg.dinit_grid(:), ...
    task_cfg.dmax_grid(:), ...
    task_cfg.dmin_grid(:), ...
    task_cfg.delta_d_grid(:));

parameter_grid = table( ...
    dinit_grid(:), ...
    dmax_grid(:), ...
    dmin_grid(:), ...
    delta_grid(:), ...
    'VariableNames', {'Dinit', 'Dmax', 'Dmin', 'deltaD'});

mask = parameter_grid.Dmin < parameter_grid.Dinit & parameter_grid.Dinit < parameter_grid.Dmax;
parameter_grid = parameter_grid(mask, :);
end

function best_result = local_select_best_result(summary_table, runner, result_dir, stop_time, array_settings, target_power_kw, source_note, sim_outputs)
best_result = struct();
best_result.result_dir = result_dir;
best_result.runner = runner;
best_result.stop_time = stop_time;
best_result.array_settings = array_settings;
best_result.target_power_kw = target_power_kw;
best_result.target_source = source_note;
best_result.selection_rule = 'pass threshold first, then minimum power ripple and minimum duty activity';

if isempty(summary_table)
    best_result.found = false;
    best_result.message = 'Task 3 parameter grid is empty.';
    return;
end

passing_rows = summary_table(summary_table.task3_pass, :);
if ~isempty(passing_rows)
    ranked_rows = sortrows(passing_rows, {'Ps_ripple_kW', 'duty_activity', 'power_fraction'}, {'ascend', 'ascend', 'descend'});
    selected_row = ranked_rows(1, :);
    best_result.found = true;
else
    ranked_rows = sortrows(summary_table, {'power_fraction', 'rail_fraction', 'Ps_ripple_kW'}, {'descend', 'ascend', 'ascend'});
    selected_row = ranked_rows(1, :);
    best_result.found = false;
    best_result.message = 'No Task 3 parameter set met the pass threshold. Returning the closest candidate.';
end

best_result.parameters = table2struct(selected_row, 'ToScalar', true);
selected_index = selected_row.run_index;
best_result.signals = local_collect_task3_signals(runner, sim_outputs(selected_index, 1));
end

function signals = local_collect_task3_signals(runner, sim_out)
signals = struct();
signals.Vs = mars.get_sim_output_var(sim_out, local_logging_variable(runner, 'Vs', 'mars_Vs'));
signals.Is = mars.get_sim_output_var(sim_out, local_logging_variable(runner, 'Is', 'mars_Is'));
signals.Ps_kW = mars.get_sim_output_var(sim_out, local_logging_variable(runner, 'Ps_kW', 'mars_Ps_kW'));
signals.duty_cycle = mars.get_sim_output_var(sim_out, local_logging_variable(runner, 'duty_cycle', 'mars_duty_cycle'));
end

function local_apply_solution_to_runner(runner, task_cfg, solar_block, mppt_constants, best_result)
load_system(runner.model_file);
set_param(solar_block, ...
    task_cfg.series_parameter, mars.format_scalar(best_result.array_settings.N_series), ...
    task_cfg.parallel_parameter, mars.format_scalar(best_result.array_settings.N_parallel));
set_param(mppt_constants{1}, 'Value', mars.format_scalar(best_result.parameters.Dinit));
set_param(mppt_constants{2}, 'Value', mars.format_scalar(best_result.parameters.Dmax));
set_param(mppt_constants{3}, 'Value', mars.format_scalar(best_result.parameters.Dmin));
set_param(mppt_constants{4}, 'Value', mars.format_scalar(best_result.parameters.deltaD));
save_system(runner.model_name, runner.model_file);
end

function fig_handles = local_create_figures(summary_table, best_result, stop_time, target_power_kw)
fig_handles = gobjects(0, 1);
fig_handles(end + 1, 1) = local_plot_ranked_scatter(summary_table);
fig_handles(end + 1, 1) = local_plot_top_candidates(summary_table);
fig_handles(end + 1, 1) = local_plot_best_traces(best_result, stop_time, target_power_kw);
end

function fig_handle = local_plot_ranked_scatter(summary_table)
fig_handle = figure('Visible', 'off', 'Color', 'w', 'Tag', 'task3_ranked_scatter');
if isempty(summary_table)
    axis off;
    text(0.5, 0.5, 'No Task 3 runs available.', 'HorizontalAlignment', 'center');
    return;
end

power_data = summary_table.power_fraction;
power_data(~isfinite(power_data)) = 0;
size_data = max(20, 40 + 120 * min(1, max(0, power_data)));
scatter(summary_table.Ps_ripple_kW, summary_table.power_fraction, size_data, summary_table.rail_fraction, 'filled');
xlabel('Power Ripple (kW)');
ylabel('Delivered Power Fraction');
title('Task 3 Candidate Ranking');
grid on;
color_handle = colorbar();
color_handle.Label.String = 'Duty Rail Fraction';
end

function fig_handle = local_plot_top_candidates(summary_table)
fig_handle = figure('Visible', 'off', 'Color', 'w', 'Tag', 'task3_top_candidates');
if isempty(summary_table)
    axis off;
    text(0.5, 0.5, 'No Task 3 runs available.', 'HorizontalAlignment', 'center');
    return;
end

ranked_rows = sortrows(summary_table, {'task3_pass', 'power_fraction', 'Ps_ripple_kW', 'duty_activity'}, {'descend', 'descend', 'ascend', 'ascend'});
top_rows = ranked_rows(1:min(10, height(ranked_rows)), :);
bar(categorical(compose('run%d', top_rows.run_index)), top_rows.Ps_mean_kW);
ylabel('Mean Solar Power (kW)');
title('Task 3 Top Candidates');
grid on;
end

function fig_handle = local_plot_best_traces(best_result, stop_time, target_power_kw)
fig_handle = figure('Visible', 'off', 'Color', 'w', 'Tag', 'task3_best_trace');
tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

if ~isfield(best_result, 'parameters')
    nexttile([3, 1]);
    axis off;
    text(0.5, 0.5, 'No Task 3 candidate available.', ...
        'HorizontalAlignment', 'center', ...
        'FontSize', 12);
    return;
end

nexttile;
local_plot_signal(best_result.signals.Ps_kW, stop_time, 'Ps (kW)');
if isfinite(target_power_kw)
    hold on;
    yline(target_power_kw, '--', 'Target', 'Color', [0.3, 0.3, 0.3]);
    hold off;
end
title(sprintf('Best Task 3 Candidate: Dinit=%.4f, Dmax=%.4f, Dmin=%.4f, deltaD=%.4f', ...
    best_result.parameters.Dinit, ...
    best_result.parameters.Dmax, ...
    best_result.parameters.Dmin, ...
    best_result.parameters.deltaD));

nexttile;
local_plot_signal(best_result.signals.duty_cycle, stop_time, 'Duty Cycle');

nexttile;
local_plot_signal(best_result.signals.Vs, stop_time, 'Vs (V)');
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

function stats = local_steady_state_stats(signal, steady_state_start)
stats = struct();
stats.found = false;
stats.mean_value = NaN;
stats.peak_value = NaN;
stats.ripple = NaN;
stats.activity = NaN;
stats.segment = [];

if ~signal.found || isempty(signal.time) || isempty(signal.data)
    return;
end

time_value = signal.time(:);
data_value = signal.data(:);
valid = isfinite(time_value) & isfinite(data_value);
time_value = time_value(valid);
data_value = data_value(valid);
if isempty(time_value)
    return;
end

mask = time_value >= steady_state_start;
if ~any(mask)
    mask = true(size(time_value));
end

segment = data_value(mask);
stats.found = true;
stats.segment = segment;
stats.mean_value = mean(segment);
stats.peak_value = max(segment);
stats.ripple = max(segment) - min(segment);
if numel(segment) > 1
    stats.activity = mean(abs(diff(segment)));
else
    stats.activity = 0;
end
end

function [array_settings, target_power_kw, source_note] = local_get_task3_inputs(cfg)
array_settings = struct();
array_source = '';
power_source = '';

task3_overrides = struct();
if isfield(cfg, 'overrides') && isstruct(cfg.overrides) ...
        && isfield(cfg.overrides, 'task3') && isstruct(cfg.overrides.task3)
    task3_overrides = cfg.overrides.task3;
end

if isfield(task3_overrides, 'N_series') && ~isempty(task3_overrides.N_series) ...
        && isfield(task3_overrides, 'N_parallel') && ~isempty(task3_overrides.N_parallel)
    array_settings.N_series = task3_overrides.N_series;
    array_settings.N_parallel = task3_overrides.N_parallel;
    array_source = 'cfg.overrides.task3.N_series/N_parallel';
else
    try
        best_task2 = mars.load_latest_best_result(cfg, 'task2');
    catch err
        error('mars:Task3NeedsTask2', ...
            ['Task 3 needs a passing Task 2 result or cfg.overrides.task3.N_series/N_parallel. ', err.message]);
    end

    if isfield(best_task2, 'found') && best_task2.found && isfield(best_task2, 'array_solution')
        array_settings.N_series = best_task2.array_solution.N_series;
        array_settings.N_parallel = best_task2.array_solution.N_parallel;
        array_source = 'Task 2 best_result array_solution';
    else
        error('mars:Task3NeedsTask2', ...
            'Task 3 needs a passing Task 2 result or cfg.overrides.task3.N_series/N_parallel.');
    end
end

if isfield(task3_overrides, 'target_power_kw') && ~isempty(task3_overrides.target_power_kw)
    target_power_kw = task3_overrides.target_power_kw;
    power_source = 'cfg.overrides.task3.target_power_kw';
elseif isfield(task3_overrides, 'array_mpp_power_kw') && ~isempty(task3_overrides.array_mpp_power_kw)
    target_power_kw = task3_overrides.array_mpp_power_kw;
    power_source = 'cfg.overrides.task3.array_mpp_power_kw';
elseif isfield(task3_overrides, 'array_mpp_power_w') && ~isempty(task3_overrides.array_mpp_power_w)
    target_power_kw = task3_overrides.array_mpp_power_w / 1000;
    power_source = 'cfg.overrides.task3.array_mpp_power_w';
elseif exist('best_task2', 'var') && isfield(best_task2, 'array_solution') && isfield(best_task2.array_solution, 'Psolar_mean_W')
    target_power_kw = best_task2.array_solution.Psolar_mean_W / 1000;
    power_source = 'Task 2 best_result array_solution.Psolar_mean_W';
else
    error('mars:Task3NeedsTargetPower', ...
        'Task 3 needs cfg.overrides.task3.target_power_kw/array_mpp_power_kw/array_mpp_power_w or a Task 2 best_result with array_solution.Psolar_mean_W.');
end

source_note = ['array: ', array_source, ' | power: ', power_source];
end

function block_path = local_get_block_path(model_name, block_name)
hits = find_system(model_name, ...
    'FollowLinks', 'on', ...
    'LookUnderMasks', 'all', ...
    'SearchDepth', 1, ...
    'Name', block_name);
if isempty(hits)
    block_path = mars.find_one_block(model_name, block_name);
else
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

function local_print_task3_start(task_cfg, runner, result_dir, stop_time, steady_state_start, parallel_enabled, array_settings, target_power_kw, source_note, parameter_grid)
raw_candidate_count = numel(task_cfg.dinit_grid) * numel(task_cfg.dmax_grid) * numel(task_cfg.dmin_grid) * numel(task_cfg.delta_d_grid);

fprintf('[task3] Starting Task 3 sweep for %s.\n', task_cfg.model);
fprintf('[task3] Result directory: %s\n', result_dir);
fprintf('[task3] Runner model: %s\n', runner.model_name);
fprintf('[task3] Execution mode: %s\n', local_execution_mode(parallel_enabled));
fprintf('[task3] Stop time: %.6g s\n', stop_time);
fprintf('[task3] Steady-state analysis starts at %.6g s\n', steady_state_start);
fprintf('[task3] Array settings: Ns=%d, Np=%d\n', array_settings.N_series, array_settings.N_parallel);
fprintf('[task3] Target power: %.3f kW (%s)\n', target_power_kw, char(string(source_note)));
fprintf('[task3] Duty limits: power_fraction >= %.3f, rail_fraction <= %.3f\n', ...
    task_cfg.power_fraction_limit, ...
    task_cfg.rail_fraction_limit);
fprintf('[task3] MPPT grid sizes: Dinit=%d, Dmax=%d, Dmin=%d, deltaD=%d\n', ...
    numel(task_cfg.dinit_grid), ...
    numel(task_cfg.dmax_grid), ...
    numel(task_cfg.dmin_grid), ...
    numel(task_cfg.delta_d_grid));
fprintf('[task3] Valid parameter candidates: %d of %d\n', height(parameter_grid), raw_candidate_count);
end

function local_print_task3_summary(summary_table, best_result, result_dir, max_debug_failures)
if isempty(summary_table)
    fprintf('[task3] No Task 3 runs were executed.\n');
    fprintf('[task3] Summary artifacts will be under %s\n', result_dir);
    return;
end

num_runs = height(summary_table);
num_pass = nnz(summary_table.task3_pass);
num_fail = num_runs - num_pass;
fprintf('[task3] Sweep finished. %d/%d runs passed.\n', num_pass, num_runs);

if num_fail > 0
    failed_rows = summary_table(~summary_table.task3_pass, :);
    failed_rows = sortrows(failed_rows, {'power_fraction', 'rail_fraction', 'Ps_ripple_kW', 'duty_activity'}, {'descend', 'ascend', 'ascend', 'ascend'});
    report_count = min(max_debug_failures, height(failed_rows));
    if report_count > 0
        fprintf('[task3] Closest failing runs (%d shown):\n', report_count);
        for idx = 1:report_count
            row = failed_rows(idx, :);
            message = local_single_line_message(row.error_message);
            if strlength(message) == 0
                message = "no explicit error captured";
            end

            fprintf('[task3]   run=%d Dinit=%.4f Dmax=%.4f Dmin=%.4f deltaD=%.4f power=%.3f %% rail=%.3f %% ripple=%.4f kW error=%s\n', ...
                row.run_index, ...
                row.Dinit, ...
                row.Dmax, ...
                row.Dmin, ...
                row.deltaD, ...
                100 * row.power_fraction, ...
                100 * row.rail_fraction, ...
                row.Ps_ripple_kW, ...
                char(message));
        end
    end
end

if isfield(best_result, 'parameters')
    fprintf('[task3] Selected candidate: Dinit=%.4f, Dmax=%.4f, Dmin=%.4f, deltaD=%.4f, mean power=%.4f kW, ripple=%.4f kW, rail=%.3f %%.\n', ...
        best_result.parameters.Dinit, ...
        best_result.parameters.Dmax, ...
        best_result.parameters.Dmin, ...
        best_result.parameters.deltaD, ...
        best_result.parameters.Ps_mean_kW, ...
        best_result.parameters.Ps_ripple_kW, ...
        100 * best_result.parameters.rail_fraction);

    if isfield(best_result, 'found') && best_result.found
        fprintf('[task3] Selected candidate meets the Task 3 pass thresholds.\n');
    elseif isfield(best_result, 'message') && ~isempty(best_result.message)
        fprintf('[task3] %s\n', best_result.message);
    end
elseif isfield(best_result, 'message') && ~isempty(best_result.message)
    fprintf('[task3] %s\n', best_result.message);
end

fprintf('[task3] Summary artifacts will be under %s\n', result_dir);
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

function value = local_get_task3_setting(cfg, task_cfg, field_name, default_value)
value = default_value;

if isfield(task_cfg, field_name) && ~isempty(task_cfg.(field_name))
    value = task_cfg.(field_name);
end

if isfield(cfg, 'overrides') && isstruct(cfg.overrides)
    if isfield(cfg.overrides, 'task3') && isstruct(cfg.overrides.task3) ...
            && isfield(cfg.overrides.task3, field_name) ...
            && ~isempty(cfg.overrides.task3.(field_name))
        value = cfg.overrides.task3.(field_name);
        return;
    end

    if isfield(cfg.overrides, field_name) && ~isempty(cfg.overrides.(field_name))
        value = cfg.overrides.(field_name);
    end
end
end

function summary_table = local_empty_summary_table()
summary_table = table( ...
    zeros(0, 1), ...
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
    strings(0, 1), ...
    'VariableNames', { ...
        'run_index', ...
        'Dinit', ...
        'Dmax', ...
        'Dmin', ...
        'deltaD', ...
        'task3_pass', ...
        'end_time_s', ...
        'mission_fraction', ...
        'Ps_mean_kW', ...
        'Ps_peak_kW', ...
        'Ps_ripple_kW', ...
        'power_fraction', ...
        'duty_mean', ...
        'duty_ripple', ...
        'duty_activity', ...
        'rail_fraction', ...
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
