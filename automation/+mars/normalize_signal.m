function [time_value, data_value] = normalize_signal(raw_value)
time_value = [];
data_value = [];

if isempty(raw_value)
    return;
end

if isa(raw_value, 'timeseries')
    time_value = raw_value.Time(:);
    data_value = squeeze(raw_value.Data);
    data_value = data_value(:);
    return;
end

if isa(raw_value, 'Simulink.SimulationData.Signal')
    [time_value, data_value] = mars.normalize_signal(raw_value.Values);
    return;
end

if isstruct(raw_value)
    if isfield(raw_value, 'time') && isfield(raw_value, 'signals')
        time_value = raw_value.time(:);
        if isfield(raw_value.signals, 'values')
            data_value = squeeze(raw_value.signals.values);
            data_value = data_value(:);
        end
        return;
    end
    if isfield(raw_value, 'Time') && isfield(raw_value, 'Data')
        time_value = raw_value.Time(:);
        data_value = squeeze(raw_value.Data);
        data_value = data_value(:);
        return;
    end
end

if isnumeric(raw_value)
    data_value = raw_value(:);
    time_value = (0:numel(data_value) - 1).';
end
end
