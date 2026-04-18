function signal = get_sim_output_var(sim_out, var_name)
signal = struct('time', [], 'data', [], 'raw', [], 'found', false);
raw_value = [];

if isa(sim_out, 'Simulink.SimulationOutput')
    try
        raw_value = sim_out.get(var_name);
    catch
        raw_value = [];
    end
    if isempty(raw_value)
        try
            names = sim_out.who;
            if any(strcmp(names, var_name))
                raw_value = sim_out.get(var_name);
            end
        catch
            raw_value = [];
        end
    end
elseif isstruct(sim_out)
    if isfield(sim_out, 'SimulationOutput') && ~isempty(sim_out.SimulationOutput)
        signal = mars.get_sim_output_var(sim_out.SimulationOutput, var_name);
        return;
    end

    if isfield(sim_out, var_name)
        raw_value = sim_out.(var_name);
    end
end

[time_value, data_value] = mars.normalize_signal(raw_value);
signal.time = time_value;
signal.data = data_value;
signal.raw = raw_value;
signal.found = ~isempty(data_value);
end
