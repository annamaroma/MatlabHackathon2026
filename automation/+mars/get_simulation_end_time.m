function end_time = get_simulation_end_time(sim_out)
end_time = NaN;

if isa(sim_out, 'Simulink.SimulationOutput')
    try
        time_value = sim_out.get('tout');
        if ~isempty(time_value)
            end_time = time_value(end);
            return;
        end
    catch
    end
elseif isstruct(sim_out) && isfield(sim_out, 'SimulationOutput') && ~isempty(sim_out.SimulationOutput)
    end_time = mars.get_simulation_end_time(sim_out.SimulationOutput);
    return;
end

try
    signal_names = {'mars_loadVoltage_V', 'mars_SOC', 'mars_Psolar', 'mars_Ps_kW'};
    for idx = 1:numel(signal_names)
        signal = mars.get_sim_output_var(sim_out, signal_names{idx});
        if signal.found
            end_time = signal.time(end);
            return;
        end
    end
catch
end
end
