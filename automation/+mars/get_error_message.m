function message = get_error_message(sim_out)
message = '';

if isa(sim_out, 'Simulink.SimulationOutput')
    try
        message = sim_out.ErrorMessage;
    catch
        message = '';
    end

    if isempty(message)
        try
            message = sim_out.get('ErrorMessage');
        catch
            message = '';
        end
    end
elseif isstruct(sim_out)
    if isfield(sim_out, 'ErrorMessage') && ~isempty(sim_out.ErrorMessage)
        message = char(string(sim_out.ErrorMessage));
        return;
    end

    if isfield(sim_out, 'SimulationOutput') && ~isempty(sim_out.SimulationOutput)
        message = mars.get_error_message(sim_out.SimulationOutput);
    end
end
end
