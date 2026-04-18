function result = run_task2(cfg)
if nargin < 1 || isempty(cfg)
    cfg = mars.default_config();
end

task_cfg = cfg.task2;
mars.ensure_profile_data_loaded(cfg);

[power_target_w, power_target_source] = local_get_power_target(cfg);
result_dir = mars.build_results_dir(cfg, task_cfg.name);
runner = mars.create_runner_model(cfg, task_cfg, result_dir, struct('prepare_fcn', @mars.create_task2_runner_model));

addpath(result_dir);
cleanup = onCleanup(@() local_cleanup(runner.model_name, result_dir));

load_system(runner.model_file);
stop_time = mars.get_stop_time(runner.model_name);

coarse_resistance = task_cfg.coarse_resistance_ohm(:);
[coarse_table, coarse_outputs] = local_run_sweep(cfg, task_cfg, runner, stop_time, "coarse_sweep", coarse_resistance);
coarse_best = local_select_best_sweep_row(coarse_table);

fine_resistance = local_build_fine_resistance_grid(task_cfg, coarse_best);
if isempty(fine_resistance)
    fine_table = local_empty_sweep_table();
    fine_outputs = struct([]);
else
    [fine_table, fine_outputs] = local_run_sweep(cfg, task_cfg, runner, stop_time, "fine_sweep", fine_resistance);
end

sweep_table = [coarse_table; fine_table];
cell_mpp = local_select_best_sweep_row(sweep_table);
array_candidates = local_build_array_candidates(task_cfg, cell_mpp, power_target_w);
[verification_table, verification_outputs] = local_verify_array_candidates(cfg, task_cfg, runner, stop_time, power_target_w, array_candidates);
best_array = local_select_best_array_candidate(verification_table);

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
run_results.coarse_outputs = coarse_outputs;
run_results.fine_outputs = fine_outputs;
run_results.verification_outputs = verification_outputs;
run_results.sweep_table = sweep_table;
run_results.verification_table = verification_table;
run_results.array_candidates = array_candidates;

mars.save_task_outputs(cfg, task_cfg.name, result_dir, summary_table, run_results, best_result, fig_handles);

result = struct();
result.result_dir = result_dir;
result.runner = runner;
result.stop_time = stop_time;
result.power_target_w = power_target_w;
result.summary_table = summary_table;
result.best_result = best_result;
result.figure_files = local_build_figure_files(fig_handles, result_dir);
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

function candidate_table = local_build_array_candidates(task_cfg, cell_mpp, power_target_w)
candidate_table = local_empty_verification_table();

if isempty(cell_mpp) || ~isfinite(cell_mpp.Vsolar_mean_V) || cell_mpp.Vsolar_mean_V <= 0 ...
        || ~isfinite(cell_mpp.Psolar_mean_W) || cell_mpp.Psolar_mean_W <= 0
    return;
end

exact_series = task_cfg.target_voltage_v / cell_mpp.Vsolar_mean_V;
series_candidates = unique(max(1, [floor(exact_series), round(exact_series), ceil(exact_series)]));
num_candidates = numel(series_candidates);

candidate_index = (1:num_candidates).';
stage = repmat("array_verify", num_candidates, 1);
run_index = candidate_index;
resistance_ohm = NaN(num_candidates, 1);
N_series = series_candidates(:);
N_parallel = zeros(num_candidates, 1);
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
    estimated_power = N_series(idx) * cell_mpp.Psolar_mean_W;
    N_parallel(idx) = max(1, ceil(power_target_w / estimated_power));
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
    sim_inputs(idx, 1) = sim_inputs(idx, 1).setVariable(task_cfg.load_variable, candidate_table.resistance_ohm(idx));
    sim_inputs(idx, 1) = sim_inputs(idx, 1).setBlockParameter( ...
        solar_block, ...
        task_cfg.series_parameter, mars.format_scalar(candidate_table.N_series(idx)), ...
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

    v_stats = local_signal_stats(mars.get_sim_output_var(sim_out, v_var), stop_time, cfg.simulation.time_tolerance, task_cfg.steady_state_fraction);
    i_stats = local_signal_stats(mars.get_sim_output_var(sim_out, i_var), stop_time, cfg.simulation.time_tolerance, task_cfg.steady_state_fraction);
    p_stats = local_signal_stats(mars.get_sim_output_var(sim_out, p_var), stop_time, cfg.simulation.time_tolerance, task_cfg.steady_state_fraction);

    verification_table.Vsolar_mean_V(idx) = v_stats.mean_abs;
    verification_table.Isolar_mean_A(idx) = i_stats.mean_abs;
    verification_table.Psolar_mean_W(idx) = p_stats.mean_abs;
    verification_table.Psolar_peak_W(idx) = p_stats.peak_abs;

    if ~isfinite(verification_table.Psolar_mean_W(idx)) ...
            && isfinite(verification_table.Vsolar_mean_V(idx)) ...
            && isfinite(verification_table.Isolar_mean_A(idx))
        verification_table.Psolar_mean_W(idx) = verification_table.Vsolar_mean_V(idx) * verification_table.Isolar_mean_A(idx);
        verification_table.Psolar_peak_W(idx) = verification_table.Psolar_mean_W(idx);
    end

    verification_table.voltage_error_V(idx) = abs(verification_table.Vsolar_mean_V(idx) - task_cfg.target_voltage_v);
    verification_table.power_margin_W(idx) = verification_table.Psolar_mean_W(idx) - power_target_w;
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
best_result.selection_rule = 'maximize cell MPP power, then choose verified array candidate by voltage error and power margin';
best_result.cell_mpp_found = ~isempty(cell_mpp);
best_result.array_solution_found = ~isempty(best_array);

if isempty(cell_mpp) || isempty(best_array)
    best_result.found = false;
    best_result.message = 'Task 2 could not find a complete sweep MPP and a passing array-sizing candidate.';
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
load_block = local_get_block_path(runner.model_name, task_cfg.load_block);
solar_block = local_get_block_path(runner.model_name, task_cfg.solar_block);

set_param(load_block, task_cfg.load_parameter, mars.format_scalar(best_result.array_solution.R_array_mpp_ohm));
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
coarse_rows = sweep_table(sweep_table.stage == "coarse_sweep", :);
fine_rows = sweep_table(sweep_table.stage == "fine_sweep", :);

hold on;
if ~isempty(coarse_rows)
    plot(coarse_rows.resistance_ohm, coarse_rows.Psolar_mean_W, 'o-', 'LineWidth', 1.1, 'DisplayName', 'Coarse sweep');
end
if ~isempty(fine_rows)
    plot(fine_rows.resistance_ohm, fine_rows.Psolar_mean_W, '.-', 'LineWidth', 1.4, 'MarkerSize', 12, 'DisplayName', 'Fine sweep');
end
if isfield(best_result, 'cell_mpp')
    plot(best_result.cell_mpp.resistance_ohm, best_result.cell_mpp.Psolar_mean_W, 'kp', ...
        'MarkerSize', 12, ...
        'MarkerFaceColor', [1, 0.90, 0.1], ...
        'DisplayName', 'Cell MPP');
end
hold off;
set(gca, 'XDir', 'reverse');
xlabel('Load Resistance (Ohm)');
ylabel('Solar Power (W)');
title('Task 2 Power vs Resistance');
grid on;
legend('Location', 'best');
end

function fig_handle = local_plot_voltage_curve(sweep_table, best_result)
fig_handle = figure('Visible', 'off', 'Color', 'w', 'Tag', 'task2_voltage_curve');
coarse_rows = sweep_table(sweep_table.stage == "coarse_sweep", :);
fine_rows = sweep_table(sweep_table.stage == "fine_sweep", :);

hold on;
if ~isempty(coarse_rows)
    plot(coarse_rows.resistance_ohm, coarse_rows.Vsolar_mean_V, 'o-', 'LineWidth', 1.1, 'DisplayName', 'Coarse sweep');
end
if ~isempty(fine_rows)
    plot(fine_rows.resistance_ohm, fine_rows.Vsolar_mean_V, '.-', 'LineWidth', 1.4, 'MarkerSize', 12, 'DisplayName', 'Fine sweep');
end
if isfield(best_result, 'cell_mpp')
    plot(best_result.cell_mpp.resistance_ohm, best_result.cell_mpp.Vsolar_mean_V, 'kp', ...
        'MarkerSize', 12, ...
        'MarkerFaceColor', [1, 0.90, 0.1], ...
        'DisplayName', 'Cell MPP');
end
hold off;
set(gca, 'XDir', 'reverse');
xlabel('Load Resistance (Ohm)');
ylabel('Solar Voltage (V)');
title('Task 2 Voltage vs Resistance');
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
