function sim_out = run_simulations(cfg, sim_inputs)
if nargin < 1 || isempty(cfg)
    cfg = mars.default_config();
end

if isempty(sim_inputs)
    sim_out = Simulink.SimulationOutput.empty(0, 1);
    return;
end

mars.ensure_profile_data_loaded(cfg);

parallel_enabled = mars.detect_parallel_support(cfg);

if cfg.parallel.show_progress
    if parallel_enabled
        fprintf('[mars] Running %d simulations with parsim.\n', numel(sim_inputs));
    else
        fprintf('[mars] Parallel execution unavailable. Running %d simulations sequentially.\n', numel(sim_inputs));
    end
end

if parallel_enabled
    sim_out = parsim(sim_inputs, ...
        'CaptureErrors', cfg.parallel.capture_errors, ...
        'ShowProgress', cfg.parallel.show_progress, ...
        'TransferBaseWorkspaceVariables', 'on', ...
        'SetupFcn', @() mars.ensure_profile_data_loaded(cfg));

    if cfg.parallel.show_progress
        fprintf('[mars] Simulation batch finished.\n');
    end
else
    sim_out = repmat(local_empty_result(), numel(sim_inputs), 1);
    for idx = 1:numel(sim_inputs)
        if cfg.parallel.show_progress
            fprintf('[mars] Starting sequential simulation %d/%d.\n', idx, numel(sim_inputs));
        end

        try
            sim_out(idx, 1).SimulationOutput = sim(sim_inputs(idx));

            if cfg.parallel.show_progress
                end_time = mars.get_simulation_end_time(sim_out(idx, 1));
                fprintf('[mars] Finished sequential simulation %d/%d at t=%.6g s.\n', idx, numel(sim_inputs), end_time);
            end
        catch err
            sim_out(idx, 1).ErrorMessage = local_format_error(err);

            if cfg.parallel.show_progress
                fprintf('[mars] Sequential simulation %d/%d failed: %s\n', ...
                    idx, ...
                    numel(sim_inputs), ...
                    sim_out(idx, 1).ErrorMessage);
            end
        end
    end
end
end

function result = local_empty_result()
result = struct();
result.ErrorMessage = '';
result.SimulationOutput = [];
end

function message = local_format_error(err)
message = '';

if isempty(err)
    return;
end

if isprop(err, 'message') && ~isempty(err.message)
    message = err.message;
else
    message = char(string(err));
end
end
