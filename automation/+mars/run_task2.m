function result = run_task2(cfg)
if nargin < 1 || isempty(cfg)
    cfg = mars.default_config();
end

task_cfg = cfg.task2;
debug_prints = local_get_task2_setting(cfg, task_cfg, 'debug_prints', true);
max_debug_failures = local_get_task2_setting(cfg, task_cfg, 'max_debug_failures', 5);

if debug_prints && ~cfg.parallel.show_progress
    cfg.parallel.show_progress = true;
end

parallel_enabled = mars.detect_parallel_support(cfg);
mars.ensure_profile_data_loaded(cfg);

[power_target_w, power_target_source] = local_get_power_target(cfg);
result_dir = mars.build_results_dir(cfg, task_cfg.name);
runner = mars.create_runner_model(cfg, task_cfg, result_dir);

addpath(result_dir);
cleanup = onCleanup(@() local_cleanup(runner.model_name, result_dir));

load_system(runner.model_file);
stop_time = mars.get_stop_time(runner.model_name);

if debug_prints
    local_print_task2_start( ...
        task_cfg, ...
        runner, ...
        result_dir, ...
        stop_time, ...
        parallel_enabled, ...
        power_target_w, ...
        power_target_source);
    fprintf('[task2] Starting built-in load profile simulation to locate the cell MPP.\n');
end

[sweep_table, profile_outputs] = local_run_cell_profile(cfg, task_cfg, runner, stop_time, "cell_profile");

if debug_prints
    local_print_sweep_stage_summary("Cell load profile", sweep_table, max_debug_failures);
end

cell_mpp = local_select_best_sweep_row(sweep_table);

if debug_prints
    local_print_cell_mpp(cell_mpp);
end

array_candidates = local_build_array_candidates(cfg, task_cfg, cell_mpp, power_target_w);

if debug_prints
    local_print_array_candidate_plan(array_candidates, power_target_w, task_cfg.target_voltage_v);
end

[verification_table, verification_outputs] = local_verify_array_candidates(cfg, task_cfg, runner, stop_time, power_target_w, array_candidates);
best_array = local_select_best_array_candidate(verification_table);

if debug_prints
    local_print_verification_summary(verification_table, best_array, max_debug_failures);
end

summary_table = [sweep_table; verification_table];
best_result = local_build_best_result( ...
    task_cfg, ...
    runner, ...
    result_dir, ...
    stop_time, ...
    power_target_w, ...
    power_target_source, ...
    cell_mpp, ...
    best_array, ...
    verification_outputs);

if best_result.found
    local_apply_solution_to_runner(runner, task_cfg, best_result);
end

fig_handles = local_create_figures(task_cfg, sweep_table, best_result, verification_table, stop_time);

run_results = struct();
run_results.runner = runner;
run_results.stop_time = stop_time;
run_results.power_target_w = power_target_w;
run_results.power_target_source = power_target_source;
run_results.profile_outputs = profile_outputs;
run_results.verification_outputs = verification_outputs;
run_results.sweep_table = sweep_table;
run_results.verification_table = verification_table;
run_results.array_candidates = array_candidates;

mars.save_task_outputs(cfg, task_cfg.name, result_dir, summary_table, run_results, best_result, fig_handles);

if debug_prints
    local_print_task2_result(best_result, result_dir);
    fprintf('[task2] Saved outputs to %s\n', result_dir);
end

result = struct();
result.result_dir = result_dir;
result.runner = runner;
result.stop_time = stop_time;
result.power_target_w = power_target_w;
result.summary_table = summary_table;
result.best_result = best_result;
result.figure_files = local_build_figure_files(fig_handles, result_dir);
end

function [summary_table, sim_outputs] = local_run_cell_profile(cfg, task_cfg, runner, stop_time, stage_name)
sim_input = Simulink.SimulationInput(runner.model_name);
sim_input = sim_input.setModelParameter('ReturnWorkspaceOutputs', 'on');

sim_outputs = mars.run_simulations(cfg, sim_input);
summary_table = local_build_profile_table(task_cfg, runner, stop_time, stage_name, sim_outputs);
end

function summary_table = local_build_profile_table(task_cfg, runner, stop_time, stage_name, sim_outputs)
run_index = 1;
stage = string(stage_name);
resistance_ohm = NaN;
N_series = NaN;
N_parallel = NaN;
task2_pass = false;
end_time_s = NaN;
mission_fraction = 0;
Vsolar_mean_V = NaN;
Isolar_mean_A = NaN;
Psolar_mean_W = NaN;
Psolar_peak_W = NaN;
voltage_error_V = NaN;
power_margin_W = NaN;
error_message = "";

v_var = local_logging_variable(runner, 'Vsolar', 'mars_Vsolar');
i_var = local_logging_variable(runner, 'Isolar', 'mars_Isolar');
p_var = local_logging_variable(runner, 'Psolar', 'mars_Psolar');

sim_out = sim_outputs(1, 1);
error_message = string(mars.get_error_message(sim_out));
end_time_s = mars.get_simulation_end_time(sim_out);
if ~isnan(end_time_s) && stop_time > 0
    mission_fraction = min(end_time_s / stop_time, 1.0);
end

profile_stats = local_profile_stats( ...
    mars.get_sim_output_var(sim_out, v_var), ...
    mars.get_sim_output_var(sim_out, i_var), ...
    mars.get_sim_output_var(sim_out, p_var));

Vsolar_mean_V = profile_stats.V_at_peak;
Isolar_mean_A = profile_stats.I_at_peak;
Psolar_mean_W = profile_stats.P_at_peak;
Psolar_peak_W = profile_stats.P_at_peak;
resistance_ohm = profile_stats.resistance_ohm;
task2_pass = strlength(error_message) == 0 ...
    && ~isnan(end_time_s) ...
    && end_time_s >= stop_time ...
    && isfinite(Psolar_mean_W);

summary_table = table( ...
    run_index, ...
    stage, ...
    resistance_ohm, ...
    N_series, ...
    N_parallel, ...
    task2_pass, ...
    end_time_s, ...
    mission_fraction, ...
    Vsolar_mean_V, ...
    Isolar_mean_A, ...
    Psolar_mean_W, ...
    Psolar_peak_W, ...
    voltage_error_V, ...
    power_margin_W, ...
    error_message, ...
    'VariableNames', local_summary_variable_names());
end

function [summary_table, sim_outputs] = local_run_sweep(cfg, task_cfg, runner, stop_time, stage_name, resistance_values)
num_runs = numel(resistance_values);
sim_inputs(num_runs, 1) = Simulink.SimulationInput(runner.model_name);
for idx = 1:num_runs
    sim_inputs(idx, 1) = Simulink.SimulationInput(runner.model_name);
    sim_inputs(idx, 1) = sim_inputs(idx, 1).setModelParameter('ReturnWorkspaceOutputs', 'on');
    sim_inputs(idx, 1) = sim_inputs(idx, 1).setVariable(task_cfg.load_variable, resistance_values(idx));
end

sim_outputs = mars.run_simulations(cfg, sim_inputs);
summary_table = local_build_sweep_table(cfg, task_cfg, runner, stop_time, stage_name, resistance_values, sim_outputs);
end

function summary_table = local_build_sweep_table(cfg, ~, runner, stop_time, stage_name, resistance_values, sim_outputs)
num_runs = numel(resistance_values);
run_index = (1:num_runs).';
stage = repmat(string(stage_name), num_runs, 1);
resistance_ohm = resistance_values(:);
N_series = NaN(num_runs, 1);
N_parallel = NaN(num_runs, 1);
task2_pass = false(num_runs, 1);
end_time_s = NaN(num_runs, 1);
mission_fraction = zeros(num_runs, 1);
Vsolar_mean_V = NaN(num_runs, 1);
Isolar_mean_A = NaN(num_runs, 1);
Psolar_mean_W = NaN(num_runs, 1);
Psolar_peak_W = NaN(num_runs, 1);
voltage_error_V = NaN(num_runs, 1);
power_margin_W = NaN(num_runs, 1);
error_message = strings(num_runs, 1);

v_var = local_logging_variable(runner, 'Vsolar', 'mars_Vsolar');
i_var = local_logging_variable(runner, 'Isolar', 'mars_Isolar');
p_var = local_logging_variable(runner, 'Psolar', 'mars_Psolar');

for idx = 1:num_runs
    sim_out = sim_outputs(idx, 1);
    error_message(idx) = string(mars.get_error_message(sim_out));
    end_time_s(idx) = mars.get_simulation_end_time(sim_out);
    if ~isnan(end_time_s(idx)) && stop_time > 0
        mission_fraction(idx) = min(end_time_s(idx) / stop_time, 1.0);
    end

    v_stats = local_signal_stats(mars.get_sim_output_var(sim_out, v_var), stop_time, cfg.simulation.time_tolerance, cfg.task2.steady_state_fraction);
    i_stats = local_signal_stats(mars.get_sim_output_var(sim_out, i_var), stop_time, cfg.simulation.time_tolerance, cfg.task2.steady_state_fraction);
    p_stats = local_signal_stats(mars.get_sim_output_var(sim_out, p_var), stop_time, cfg.simulation.time_tolerance, cfg.task2.steady_state_fraction);

    Vsolar_mean_V(idx) = v_stats.mean_abs;
    Isolar_mean_A(idx) = i_stats.mean_abs;
    Psolar_mean_W(idx) = p_stats.mean_abs;
    Psolar_peak_W(idx) = p_stats.peak_abs;

    if ~isfinite(Psolar_mean_W(idx)) && isfinite(Vsolar_mean_V(idx)) && isfinite(Isolar_mean_A(idx))
        Psolar_mean_W(idx) = Vsolar_mean_V(idx) * Isolar_mean_A(idx);
        Psolar_peak_W(idx) = Psolar_mean_W(idx);
    end

    task2_pass(idx) = strlength(error_message(idx)) == 0 ...
        && ~isnan(end_time_s(idx)) ...
        && end_time_s(idx) >= stop_time - cfg.simulation.time_tolerance ...
        && isfinite(Psolar_mean_W(idx));
end

summary_table = table( ...
    run_index, ...
    stage, ...
    resistance_ohm, ...
    N_series, ...
    N_parallel, ...
    task2_pass, ...
    end_time_s, ...
    mission_fraction, ...
    Vsolar_mean_V, ...
    Isolar_mean_A, ...
    Psolar_mean_W, ...
    Psolar_peak_W, ...
    voltage_error_V, ...
    power_margin_W, ...
    error_message, ...
    'VariableNames', { ...
        'run_index', ...
        'stage', ...
        'resistance_ohm', ...
        'N_series', ...
        'N_parallel', ...
        'task2_pass', ...
        'end_time_s', ...
        'mission_fraction', ...
        'Vsolar_mean_V', ...
        'Isolar_mean_A', ...
        'Psolar_mean_W', ...
        'Psolar_peak_W', ...
        'voltage_error_V', ...
        'power_margin_W', ...
        'error_message' ...
    });
end

function fine_resistance = local_build_fine_resistance_grid(task_cfg, coarse_best)
fine_resistance = [];
if isempty(coarse_best)
    return;
end

upper_bound = min(max(task_cfg.coarse_resistance_ohm), coarse_best.resistance_ohm + task_cfg.fine_half_span_ohm);
lower_bound = max(min(task_cfg.coarse_resistance_ohm), coarse_best.resistance_ohm - task_cfg.fine_half_span_ohm);
if upper_bound < lower_bound
    return;
end

fine_resistance = upper_bound:-task_cfg.fine_step_ohm:lower_bound;
fine_resistance = round(fine_resistance * 1000) / 1000;
fine_resistance = fine_resistance(:);
end

function best_row = local_select_best_sweep_row(summary_table)
best_row = table();
if isempty(summary_table)
    return;
end

valid_rows = summary_table(summary_table.task2_pass & isfinite(summary_table.Psolar_mean_W), :);
if isempty(valid_rows)
    return;
end

valid_rows = sortrows(valid_rows, {'Psolar_mean_W', 'resistance_ohm'}, {'descend', 'descend'});
best_row = valid_rows(1, :);
end

function candidate_table = local_build_array_candidates(cfg, task_cfg, cell_mpp, power_target_w)
candidate_table = local_empty_verification_table();

if isempty(cell_mpp) || ~isfinite(cell_mpp.Vsolar_mean_V) || cell_mpp.Vsolar_mean_V <= 0 ...
        || ~isfinite(cell_mpp.Psolar_mean_W) || cell_mpp.Psolar_mean_W <= 0
    return;
end

series_center = local_get_task2_setting(cfg, task_cfg, 'series_search_center', 850);
series_half_span = local_get_task2_setting(cfg, task_cfg, 'series_search_half_span', 50);
series_step = local_get_task2_setting(cfg, task_cfg, 'series_search_step', 5);
parallel_center = local_get_task2_setting(cfg, task_cfg, 'parallel_search_center', 10);
parallel_half_span = local_get_task2_setting(cfg, task_cfg, 'parallel_search_half_span', 5);
parallel_step = local_get_task2_setting(cfg, task_cfg, 'parallel_search_step', 1);

series_min = max(1, series_center - series_half_span);
series_max = max(series_min, series_center + series_half_span);
parallel_min = max(1, parallel_center - parallel_half_span);
parallel_max = max(parallel_min, parallel_center + parallel_half_span);

series_candidates = series_min:series_step:series_max;
parallel_candidates = parallel_min:parallel_step:parallel_max;
if isempty(series_candidates) || isempty(parallel_candidates)
    return;
end

[series_grid, parallel_grid] = ndgrid(series_candidates(:), parallel_candidates(:));
num_candidates = numel(series_grid);

candidate_index = (1:num_candidates).';
stage = repmat("array_verify", num_candidates, 1);
run_index = candidate_index;
resistance_ohm = NaN(num_candidates, 1);
N_series = series_grid(:);
N_parallel = parallel_grid(:);
task2_pass = false(num_candidates, 1);
end_time_s = NaN(num_candidates, 1);
mission_fraction = zeros(num_candidates, 1);
Vsolar_mean_V = NaN(num_candidates, 1);
Isolar_mean_A = NaN(num_candidates, 1);
Psolar_mean_W = NaN(num_candidates, 1);
Psolar_peak_W = NaN(num_candidates, 1);
voltage_error_V = NaN(num_candidates, 1);
power_margin_W = NaN(num_candidates, 1);
error_message = strings(num_candidates, 1);

for idx = 1:num_candidates
    resistance_ohm(idx) = cell_mpp.resistance_ohm * N_series(idx) / N_parallel(idx);
    Vsolar_mean_V(idx) = N_series(idx) * cell_mpp.Vsolar_mean_V;
    Isolar_mean_A(idx) = N_parallel(idx) * cell_mpp.Isolar_mean_A;
    Psolar_mean_W(idx) = N_series(idx) * N_parallel(idx) * cell_mpp.Psolar_mean_W;
    Psolar_peak_W(idx) = Psolar_mean_W(idx);
    voltage_error_V(idx) = abs(Vsolar_mean_V(idx) - task_cfg.target_voltage_v);
    power_margin_W(idx) = Psolar_mean_W(idx) - power_target_w;
    task2_pass(idx) = power_margin_W(idx) >= 0;
end

candidate_table = table( ...
    run_index, ...
    stage, ...
    resistance_ohm, ...
    N_series, ...
    N_parallel, ...
    task2_pass, ...
    end_time_s, ...
    mission_fraction, ...
    Vsolar_mean_V, ...
    Isolar_mean_A, ...
    Psolar_mean_W, ...
    Psolar_peak_W, ...
    voltage_error_V, ...
    power_margin_W, ...
    error_message, ...
    'VariableNames', local_summary_variable_names());
end

function [verification_table, sim_outputs] = local_verify_array_candidates(cfg, task_cfg, runner, stop_time, power_target_w, candidate_table)
verification_table = local_empty_verification_table();
sim_outputs = struct([]);
if isempty(candidate_table)
    return;
end

solar_block = local_get_block_path(runner.model_name, task_cfg.solar_block);
num_runs = height(candidate_table);
sim_inputs(num_runs, 1) = Simulink.SimulationInput(runner.model_name);
for idx = 1:num_runs
    sim_inputs(idx, 1) = Simulink.SimulationInput(runner.model_name);
    sim_inputs(idx, 1) = sim_inputs(idx, 1).setModelParameter('ReturnWorkspaceOutputs', 'on');
    sim_inputs(idx, 1) = sim_inputs(idx, 1).setBlockParameter( ...
        solar_block, ...
        task_cfg.series_parameter, mars.format_scalar(candidate_table.N_series(idx)));
    sim_inputs(idx, 1) = sim_inputs(idx, 1).setBlockParameter( ...
        solar_block, ...
        task_cfg.parallel_parameter, mars.format_scalar(candidate_table.N_parallel(idx)));
end

sim_outputs = mars.run_simulations(cfg, sim_inputs);
verification_table = candidate_table;

v_var = local_logging_variable(runner, 'Vsolar', 'mars_Vsolar');
i_var = local_logging_variable(runner, 'Isolar', 'mars_Isolar');
p_var = local_logging_variable(runner, 'Psolar', 'mars_Psolar');

for idx = 1:num_runs
    sim_out = sim_outputs(idx, 1);
    verification_table.error_message(idx) = string(mars.get_error_message(sim_out));
    verification_table.end_time_s(idx) = mars.get_simulation_end_time(sim_out);
    if ~isnan(verification_table.end_time_s(idx)) && stop_time > 0
        verification_table.mission_fraction(idx) = min(verification_table.end_time_s(idx) / stop_time, 1.0);
    end

    profile_stats = local_profile_stats( ...
        mars.get_sim_output_var(sim_out, v_var), ...
        mars.get_sim_output_var(sim_out, i_var), ...
        mars.get_sim_output_var(sim_out, p_var));

    verification_table.resistance_ohm(idx) = profile_stats.resistance_ohm;
    verification_table.Vsolar_mean_V(idx) = profile_stats.V_at_peak;
    verification_table.Isolar_mean_A(idx) = profile_stats.I_at_peak;
    verification_table.Psolar_mean_W(idx) = profile_stats.P_at_peak;
    verification_table.Psolar_peak_W(idx) = profile_stats.P_at_peak;
    verification_table.voltage_error_V(idx) = abs(profile_stats.V_at_peak - task_cfg.target_voltage_v);
    verification_table.power_margin_W(idx) = profile_stats.P_at_peak - power_target_w;
    verification_table.task2_pass(idx) = strlength(verification_table.error_message(idx)) == 0 ...
        && ~isnan(verification_table.end_time_s(idx)) ...
        && verification_table.end_time_s(idx) >= stop_time - cfg.simulation.time_tolerance ...
        && isfinite(verification_table.Psolar_mean_W(idx)) ...
        && verification_table.power_margin_W(idx) >= 0;
end
end

function best_row = local_select_best_array_candidate(verification_table)
best_row = table();
if isempty(verification_table)
    return;
end

valid_rows = verification_table(verification_table.task2_pass, :);
if isempty(valid_rows)
    return;
end

valid_rows = sortrows(valid_rows, {'voltage_error_V', 'power_margin_W', 'N_series', 'N_parallel'}, {'ascend', 'ascend', 'ascend', 'ascend'});
best_row = valid_rows(1, :);
end

function best_result = local_build_best_result(task_cfg, runner, result_dir, stop_time, power_target_w, power_target_source, cell_mpp, best_array, verification_outputs)
best_result = struct();
best_result.result_dir = result_dir;
best_result.runner = runner;
best_result.stop_time = stop_time;
best_result.power_target_w = power_target_w;
best_result.power_target_source = power_target_source;
best_result.selection_rule = 'find the cell peak from the built-in load profile, then choose the verified array candidate by voltage error and power margin';
best_result.cell_mpp_found = ~isempty(cell_mpp);
best_result.array_solution_found = ~isempty(best_array);

if isempty(cell_mpp) || isempty(best_array)
    best_result.found = false;
    best_result.message = 'Task 2 could not find a valid cell peak from the built-in load profile and a passing array-sizing candidate.';
    if ~isempty(cell_mpp)
        best_result.cell_mpp = table2struct(cell_mpp, 'ToScalar', true);
    end
    return;
end

best_result.found = true;
best_result.cell_mpp = table2struct(cell_mpp, 'ToScalar', true);
best_result.array_solution = table2struct(best_array, 'ToScalar', true);
best_result.array_solution.R_array_mpp_ohm = best_array.resistance_ohm;
best_result.array_solution.target_voltage_v = task_cfg.target_voltage_v;
selected_index = best_array.run_index;
best_result.signals = local_collect_task2_signals(runner, verification_outputs(selected_index, 1));
end

function signals = local_collect_task2_signals(runner, sim_out)
signals = struct();
signals.Vsolar = mars.get_sim_output_var(sim_out, local_logging_variable(runner, 'Vsolar', 'mars_Vsolar'));
signals.Isolar = mars.get_sim_output_var(sim_out, local_logging_variable(runner, 'Isolar', 'mars_Isolar'));
signals.Psolar = mars.get_sim_output_var(sim_out, local_logging_variable(runner, 'Psolar', 'mars_Psolar'));
end

function local_apply_solution_to_runner(runner, task_cfg, best_result)
load_system(runner.model_file);
solar_block = local_get_block_path(runner.model_name, task_cfg.solar_block);

set_param(solar_block, ...
    task_cfg.series_parameter, mars.format_scalar(best_result.array_solution.N_series), ...
    task_cfg.parallel_parameter, mars.format_scalar(best_result.array_solution.N_parallel));

save_system(runner.model_name, runner.model_file);
end

function fig_handles = local_create_figures(task_cfg, sweep_table, best_result, verification_table, stop_time)
fig_handles = gobjects(0, 1);
fig_handles(end + 1, 1) = local_plot_power_curve(sweep_table, best_result);
fig_handles(end + 1, 1) = local_plot_voltage_curve(sweep_table, best_result);
fig_handles(end + 1, 1) = local_plot_array_summary(task_cfg, best_result, verification_table);
fig_handles(end + 1, 1) = local_plot_best_traces(best_result, stop_time);
end

function fig_handle = local_plot_power_curve(sweep_table, best_result)
fig_handle = figure('Visible', 'off', 'Color', 'w', 'Tag', 'task2_power_curve');
profile_rows = sweep_table(sweep_table.stage ~= "array_verify", :);

hold on;
if ~isempty(profile_rows)
    plot(profile_rows.resistance_ohm, profile_rows.Psolar_mean_W, 'o', 'LineWidth', 1.1, 'MarkerSize', 8, 'DisplayName', 'Cell peak from built-in load profile');
end
if isfield(best_result, 'cell_mpp')
    plot(best_result.cell_mpp.resistance_ohm, best_result.cell_mpp.Psolar_mean_W, 'kp', ...
        'MarkerSize', 12, ...
        'MarkerFaceColor', [1, 0.90, 0.1], ...
        'DisplayName', 'Cell MPP');
end
hold off;
xlabel('Load Resistance (Ohm)');
ylabel('Solar Power (W)');
title('Task 2 Peak Power vs Effective Load Resistance');
grid on;
legend('Location', 'best');
end

function fig_handle = local_plot_voltage_curve(sweep_table, best_result)
fig_handle = figure('Visible', 'off', 'Color', 'w', 'Tag', 'task2_voltage_curve');
profile_rows = sweep_table(sweep_table.stage ~= "array_verify", :);

hold on;
if ~isempty(profile_rows)
    plot(profile_rows.resistance_ohm, profile_rows.Vsolar_mean_V, 'o', 'LineWidth', 1.1, 'MarkerSize', 8, 'DisplayName', 'Cell peak from built-in load profile');
end
if isfield(best_result, 'cell_mpp')
    plot(best_result.cell_mpp.resistance_ohm, best_result.cell_mpp.Vsolar_mean_V, 'kp', ...
        'MarkerSize', 12, ...
        'MarkerFaceColor', [1, 0.90, 0.1], ...
        'DisplayName', 'Cell MPP');
end
hold off;
xlabel('Load Resistance (Ohm)');
ylabel('Solar Voltage (V)');
title('Task 2 Peak Voltage vs Effective Load Resistance');
grid on;
legend('Location', 'best');
end

function fig_handle = local_plot_array_summary(task_cfg, best_result, verification_table)
fig_handle = figure('Visible', 'off', 'Color', 'w', 'Tag', 'task2_array_summary');
axis off;

lines = strings(0, 1);
lines(end + 1, 1) = "Task 2 Array Sizing Summary";
lines(end + 1, 1) = "";
lines(end + 1, 1) = "Target voltage: " + string(task_cfg.target_voltage_v) + " V";
if isfield(best_result, 'power_target_w')
    lines(end + 1, 1) = "Target power: " + sprintf('%.3f kW', best_result.power_target_w / 1000);
end

if best_result.found
    lines(end + 1, 1) = "";
    lines(end + 1, 1) = "Cell MPP";
    lines(end + 1, 1) = sprintf('R = %.3f Ohm, V = %.3f V, I = %.3f A, P = %.3f W', ...
        best_result.cell_mpp.resistance_ohm, ...
        best_result.cell_mpp.Vsolar_mean_V, ...
        best_result.cell_mpp.Isolar_mean_A, ...
        best_result.cell_mpp.Psolar_mean_W);
    lines(end + 1, 1) = "";
    lines(end + 1, 1) = "Chosen array";
    lines(end + 1, 1) = sprintf('N_series = %d, N_parallel = %d', ...
        best_result.array_solution.N_series, ...
        best_result.array_solution.N_parallel);
    lines(end + 1, 1) = sprintf('R_array_mpp = %.3f Ohm', best_result.array_solution.R_array_mpp_ohm);
    lines(end + 1, 1) = sprintf('Verified V = %.3f V, P = %.3f W', ...
        best_result.array_solution.Vsolar_mean_V, ...
        best_result.array_solution.Psolar_mean_W);
    lines(end + 1, 1) = sprintf('Voltage error = %.3f V, Power margin = %.3f W', ...
        best_result.array_solution.voltage_error_V, ...
        best_result.array_solution.power_margin_W);
else
    lines(end + 1, 1) = "";
    lines(end + 1, 1) = "No passing Task 2 array candidate found yet.";
end

if ~isempty(verification_table)
    lines(end + 1, 1) = "";
    lines(end + 1, 1) = "Verified candidates checked: " + string(height(verification_table));
end

text(0.03, 0.97, strjoin(cellstr(lines), newline), ...
    'VerticalAlignment', 'top', ...
    'Interpreter', 'none', ...
    'FontName', 'Courier New');
end

function fig_handle = local_plot_best_traces(best_result, stop_time)
fig_handle = figure('Visible', 'off', 'Color', 'w', 'Tag', 'task2_best_trace');
tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

if ~best_result.found
    nexttile([3, 1]);
    axis off;
    text(0.5, 0.5, 'No passing Task 2 candidate was found.', ...
        'HorizontalAlignment', 'center', ...
        'FontSize', 12);
    return;
end

nexttile;
local_plot_signal(best_result.signals.Vsolar, stop_time, 'Vsolar (V)');
title(sprintf('Best Task 2 Candidate: Ns=%d, Np=%d', ...
    best_result.array_solution.N_series, ...
    best_result.array_solution.N_parallel));

nexttile;
local_plot_signal(best_result.signals.Isolar, stop_time, 'Isolar (A)');

nexttile;
local_plot_signal(best_result.signals.Psolar, stop_time, 'Psolar (W)');
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
    text(0.5, 0.5, ['Signal not available: ', y_label], ...
        'HorizontalAlignment', 'center');
end
end

function stats = local_signal_stats(signal, stop_time, time_tolerance, steady_state_fraction)
stats = struct('mean_abs', NaN, 'peak_abs', NaN, 'final_abs', NaN);
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

window_anchor = min(stop_time, time_value(end));
window_start = max(time_value(1), window_anchor - steady_state_fraction * stop_time - time_tolerance);
mask = time_value >= window_start;
if ~any(mask)
    mask = true(size(time_value));
end

segment = data_value(mask);
stats.mean_abs = mean(abs(segment));
stats.peak_abs = max(abs(segment));
stats.final_abs = abs(data_value(end));
end

function stats = local_profile_stats(v_signal, i_signal, p_signal)
stats = struct();
stats.peak_time = NaN;
stats.V_at_peak = NaN;
stats.I_at_peak = NaN;
stats.P_at_peak = NaN;
stats.resistance_ohm = NaN;

[p_time, p_data] = local_signal_vectors(p_signal);
if isempty(p_time) || isempty(p_data)
    [v_time, v_data] = local_signal_vectors(v_signal);
    [i_time, i_data] = local_signal_vectors(i_signal);
    if isempty(v_time) || isempty(v_data) || isempty(i_time) || isempty(i_data)
        return;
    end

    common_time = v_time;
    current_value = interp1(i_time, i_data, common_time, 'linear', 'extrap');
    power_value = abs(v_data .* current_value);
    [~, peak_idx] = max(power_value);
    stats.peak_time = common_time(peak_idx);
    stats.V_at_peak = abs(v_data(peak_idx));
    stats.I_at_peak = abs(current_value(peak_idx));
    stats.P_at_peak = abs(power_value(peak_idx));
else
    [~, peak_idx] = max(abs(p_data));
    stats.peak_time = p_time(peak_idx);
    stats.P_at_peak = abs(p_data(peak_idx));
    stats.V_at_peak = abs(local_signal_value_at_time(v_signal, stats.peak_time));
    stats.I_at_peak = abs(local_signal_value_at_time(i_signal, stats.peak_time));
end

if isfinite(stats.I_at_peak) && stats.I_at_peak > 0
    stats.resistance_ohm = stats.V_at_peak / stats.I_at_peak;
end
end

function [time_value, data_value] = local_signal_vectors(signal)
time_value = [];
data_value = [];
if ~signal.found || isempty(signal.time) || isempty(signal.data)
    return;
end

time_value = signal.time(:);
data_value = signal.data(:);
valid = isfinite(time_value) & isfinite(data_value);
time_value = time_value(valid);
data_value = data_value(valid);
end

function value = local_signal_value_at_time(signal, sample_time)
value = NaN;
[time_value, data_value] = local_signal_vectors(signal);
if isempty(time_value) || isempty(data_value)
    return;
end

if numel(time_value) == 1
    value = data_value(1);
    return;
end

value = interp1(time_value, data_value, sample_time, 'linear', 'extrap');
end

function [power_target_w, source_note] = local_get_power_target(cfg)
source_note = '';

if isfield(cfg, 'overrides') && isstruct(cfg.overrides)
    if isfield(cfg.overrides, 'task2') && isstruct(cfg.overrides.task2)
        if isfield(cfg.overrides.task2, 'power_target_w') && ~isempty(cfg.overrides.task2.power_target_w)
            power_target_w = cfg.overrides.task2.power_target_w;
            source_note = 'cfg.overrides.task2.power_target_w';
            return;
        end
        if isfield(cfg.overrides.task2, 'power_target_kw') && ~isempty(cfg.overrides.task2.power_target_kw)
            power_target_w = cfg.overrides.task2.power_target_kw * 1000;
            source_note = 'cfg.overrides.task2.power_target_kw';
            return;
        end
    end
end

try
    best_task1 = mars.load_latest_best_result(cfg, 'task1');
catch err
    error('mars:Task2NeedsTask1', ...
        ['Task 2 needs a Task 1 result or cfg.overrides.task2.power_target_kw. ', err.message]);
end

if ~isfield(best_task1, 'found') || ~best_task1.found || ~isfield(best_task1, 'solar_kW')
    error('mars:Task2NeedsPassingTask1', ...
        'Task 2 needs a passing Task 1 best_result with the solar_kW field available.');
end

power_target_w = best_task1.solar_kW * 1000;
source_note = sprintf('Task 1 best_result solar_kW = %.3f', best_task1.solar_kW);
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

function local_print_task2_start(task_cfg, runner, result_dir, stop_time, parallel_enabled, power_target_w, power_target_source)
fprintf('[task2] Starting Task 2 sweep for %s.\n', task_cfg.model);
fprintf('[task2] Result directory: %s\n', result_dir);
fprintf('[task2] Runner model: %s\n', runner.model_name);
fprintf('[task2] Execution mode: %s\n', local_execution_mode(parallel_enabled));
fprintf('[task2] Stop time: %.6g s\n', stop_time);
fprintf('[task2] Power target: %.3f kW (%s)\n', power_target_w / 1000, char(string(power_target_source)));
fprintf('[task2] Target voltage: %.3f V\n', task_cfg.target_voltage_v);
fprintf('[task2] Using the model''s built-in variable-resistor load profile without overriding it.\n');
fprintf('[task2] Array parameters: %s / %s, %s\n', ...
    task_cfg.solar_block, ...
    task_cfg.series_parameter, ...
    task_cfg.parallel_parameter);
end

function local_print_sweep_stage_summary(stage_label, summary_table, max_debug_failures)
stage_label = char(string(stage_label));
if isempty(summary_table)
    fprintf('[task2] %s skipped.\n', stage_label);
    return;
end

num_runs = height(summary_table);
num_pass = nnz(summary_table.task2_pass);
num_fail = num_runs - num_pass;
fprintf('[task2] %s finished. %d/%d runs passed.\n', stage_label, num_pass, num_runs);

best_row = local_select_best_sweep_row(summary_table);
if ~isempty(best_row)
    fprintf('[task2] Best %s point: R=%.3f Ohm, V=%.3f V, I=%.3f A, P=%.3f W.\n', ...
        lower(stage_label), ...
        best_row.resistance_ohm, ...
        best_row.Vsolar_mean_V, ...
        best_row.Isolar_mean_A, ...
        best_row.Psolar_mean_W);
else
    fallback_rows = sortrows(summary_table, {'mission_fraction', 'Psolar_mean_W', 'resistance_ohm'}, {'descend', 'descend', 'descend'});
    if ~isempty(fallback_rows)
        row = fallback_rows(1, :);
        fprintf('[task2] Best %s attempt: R=%.3f Ohm, mission=%.1f %%, P=%.3f W, error=%s\n', ...
            lower(stage_label), ...
            row.resistance_ohm, ...
            100 * row.mission_fraction, ...
            row.Psolar_mean_W, ...
            char(local_single_line_message(row.error_message)));
    end
end

if num_fail > 0
    failed_rows = summary_table(~summary_table.task2_pass, :);
    failed_rows = sortrows(failed_rows, {'mission_fraction', 'Psolar_mean_W', 'resistance_ohm'}, {'descend', 'descend', 'descend'});
    local_print_failed_sweep_rows(stage_label, failed_rows, max_debug_failures);
end
end

function local_print_failed_sweep_rows(stage_label, failed_rows, max_debug_failures)
report_count = min(max_debug_failures, height(failed_rows));
if report_count <= 0
    return;
end

fprintf('[task2] %s closest failing runs (%d shown):\n', char(string(stage_label)), report_count);
for idx = 1:report_count
    row = failed_rows(idx, :);
    message = local_single_line_message(row.error_message);
    if strlength(message) == 0
        message = "no explicit error captured";
    end

    fprintf('[task2]   run=%d R=%.3f Ohm mission=%.1f %% V=%.3f V I=%.3f A P=%.3f W error=%s\n', ...
        row.run_index, ...
        row.resistance_ohm, ...
        100 * row.mission_fraction, ...
        row.Vsolar_mean_V, ...
        row.Isolar_mean_A, ...
        row.Psolar_mean_W, ...
        char(message));
end
end

function local_print_cell_mpp(cell_mpp)
if isempty(cell_mpp)
    fprintf('[task2] Combined sweep did not identify a valid cell MPP candidate.\n');
    return;
end

fprintf('[task2] Selected cell MPP: R=%.3f Ohm, V=%.3f V, I=%.3f A, P=%.3f W.\n', ...
    cell_mpp.resistance_ohm, ...
    cell_mpp.Vsolar_mean_V, ...
    cell_mpp.Isolar_mean_A, ...
    cell_mpp.Psolar_mean_W);
end

function local_print_array_candidate_plan(candidate_table, power_target_w, target_voltage_v)
if isempty(candidate_table)
    fprintf('[task2] No array candidates were built for verification.\n');
    return;
end

fprintf('[task2] Built %d array candidates for %.3f kW at %.3f V.\n', ...
    height(candidate_table), ...
    power_target_w / 1000, ...
    target_voltage_v);
fprintf('[task2] Search window: Ns=%d..%d, Np=%d..%d.\n', ...
    min(candidate_table.N_series), ...
    max(candidate_table.N_series), ...
    min(candidate_table.N_parallel), ...
    max(candidate_table.N_parallel));

preview_count = min(5, height(candidate_table));
for idx = 1:preview_count
    row = candidate_table(idx, :);
    fprintf('[task2]   candidate=%d Ns=%d Np=%d R=%.3f Ohm estV=%.3f V estP=%.3f W margin=%.3f W\n', ...
        row.run_index, ...
        row.N_series, ...
        row.N_parallel, ...
        row.resistance_ohm, ...
        row.Vsolar_mean_V, ...
        row.Psolar_mean_W, ...
        row.power_margin_W);
end
end

function local_print_verification_summary(verification_table, best_array, max_debug_failures)
if isempty(verification_table)
    fprintf('[task2] Array verification skipped because no candidates were available.\n');
    return;
end

num_runs = height(verification_table);
num_pass = nnz(verification_table.task2_pass);
num_fail = num_runs - num_pass;
fprintf('[task2] Array verification finished. %d/%d candidates passed.\n', num_pass, num_runs);

if ~isempty(best_array)
    fprintf('[task2] Best verified array: Ns=%d, Np=%d, R=%.3f Ohm, V=%.3f V, P=%.3f W, margin=%.3f W.\n', ...
        best_array.N_series, ...
        best_array.N_parallel, ...
        best_array.resistance_ohm, ...
        best_array.Vsolar_mean_V, ...
        best_array.Psolar_mean_W, ...
        best_array.power_margin_W);
else
    closest_rows = sortrows(verification_table, {'voltage_error_V', 'power_margin_W', 'N_series', 'N_parallel'}, {'ascend', 'descend', 'ascend', 'ascend'});
    row = closest_rows(1, :);
    fprintf('[task2] No passing verified array found. Closest attempt: Ns=%d, Np=%d, V=%.3f V, P=%.3f W, voltage error=%.3f V, margin=%.3f W.\n', ...
        row.N_series, ...
        row.N_parallel, ...
        row.Vsolar_mean_V, ...
        row.Psolar_mean_W, ...
        row.voltage_error_V, ...
        row.power_margin_W);
end

if num_fail > 0
    failed_rows = verification_table(~verification_table.task2_pass, :);
    failed_rows = sortrows(failed_rows, {'power_margin_W', 'voltage_error_V', 'N_series', 'N_parallel'}, {'descend', 'ascend', 'ascend', 'ascend'});
    local_print_failed_verification_rows(failed_rows, max_debug_failures);
end
end

function local_print_failed_verification_rows(failed_rows, max_debug_failures)
report_count = min(max_debug_failures, height(failed_rows));
if report_count <= 0
    return;
end

fprintf('[task2] Closest failing verification candidates (%d shown):\n', report_count);
for idx = 1:report_count
    row = failed_rows(idx, :);
    message = local_single_line_message(row.error_message);
    if strlength(message) == 0
        message = "no explicit error captured";
    end

    fprintf('[task2]   candidate=%d Ns=%d Np=%d R=%.3f Ohm V=%.3f V P=%.3f W voltage_error=%.3f V margin=%.3f W error=%s\n', ...
        row.run_index, ...
        row.N_series, ...
        row.N_parallel, ...
        row.resistance_ohm, ...
        row.Vsolar_mean_V, ...
        row.Psolar_mean_W, ...
        row.voltage_error_V, ...
        row.power_margin_W, ...
        char(message));
end
end

function local_print_task2_result(best_result, result_dir)
if isfield(best_result, 'found') && best_result.found
    fprintf('[task2] Final solution: cell peak R=%.3f Ohm, Ns=%d, Np=%d, array peak R=%.3f Ohm.\n', ...
        best_result.cell_mpp.resistance_ohm, ...
        best_result.array_solution.N_series, ...
        best_result.array_solution.N_parallel, ...
        best_result.array_solution.R_array_mpp_ohm);
else
    fprintf('[task2] No passing Task 2 solution was found.\n');
end

fprintf('[task2] Summary artifacts will be under %s\n', result_dir);
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

function value = local_get_task2_setting(cfg, task_cfg, field_name, default_value)
value = default_value;

if isfield(task_cfg, field_name) && ~isempty(task_cfg.(field_name))
    value = task_cfg.(field_name);
end

if isfield(cfg, 'overrides') && isstruct(cfg.overrides)
    if isfield(cfg.overrides, 'task2') && isstruct(cfg.overrides.task2) ...
            && isfield(cfg.overrides.task2, field_name) ...
            && ~isempty(cfg.overrides.task2.(field_name))
        value = cfg.overrides.task2.(field_name);
        return;
    end

    if isfield(cfg.overrides, field_name) && ~isempty(cfg.overrides.(field_name))
        value = cfg.overrides.(field_name);
    end
end
end

function table_out = local_empty_sweep_table()
table_out = table( ...
    zeros(0, 1), ...
    strings(0, 1), ...
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
    strings(0, 1), ...
    'VariableNames', local_summary_variable_names());
end

function table_out = local_empty_verification_table()
table_out = local_empty_sweep_table();
end

function names = local_summary_variable_names()
names = { ...
    'run_index', ...
    'stage', ...
    'resistance_ohm', ...
    'N_series', ...
    'N_parallel', ...
    'task2_pass', ...
    'end_time_s', ...
    'mission_fraction', ...
    'Vsolar_mean_V', ...
    'Isolar_mean_A', ...
    'Psolar_mean_W', ...
    'Psolar_peak_W', ...
    'voltage_error_V', ...
    'power_margin_W', ...
    'error_message' ...
};
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
