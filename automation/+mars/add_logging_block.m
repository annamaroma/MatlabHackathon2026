function [log_block_path, variable_name] = add_logging_block(model_name, source_relative_path, signal_name)
source_block = fullfile(model_name, source_relative_path);
source_block = strrep(source_block, filesep, '/');
source_handles = get_param(source_block, 'PortHandles');
if ~isfield(source_handles, 'Outport') || isempty(source_handles.Outport)
    error('mars:LoggingSourceHasNoOutport', 'Block "%s" has no Simulink outport.', source_block);
end

source_port = source_handles.Outport(1);
source_position = get_param(source_block, 'Position');

separator_idx = find(source_relative_path == '/', 1, 'last');
if isempty(separator_idx)
    parent_system = model_name;
else
    parent_system = [model_name, '/', source_relative_path(1:separator_idx - 1)];
end

variable_name = matlab.lang.makeValidName(['mars_', signal_name]);
block_name = matlab.lang.makeValidName(['Log_', signal_name]);
log_block_path = [parent_system, '/', block_name];

left = source_position(3) + 40;
top = source_position(2);
right = left + 90;
bottom = top + 20;

add_block('simulink/Sinks/To Workspace', log_block_path, ...
    'VariableName', variable_name, ...
    'SaveFormat', 'Structure With Time', ...
    'Position', [left, top, right, bottom]);

target_handles = get_param(log_block_path, 'PortHandles');
add_line(parent_system, source_port, target_handles.Inport(1), 'autorouting', 'on');
end
