function create_task5_runner_model(cfg, ~, runner_model, runner_file)
load_system(runner_file);

task3_model = cfg.task3.model;
task4_model = cfg.task4.model;
if ~bdIsLoaded(task3_model)
    load_system(cfg.task3.model_file);
end
if ~bdIsLoaded(task4_model)
    load_system(cfg.task4.model_file);
end

solar_subsystem = [runner_model, '/Solar Array'];
battery_subsystem = [runner_model, '/Battery'];

solar_block = [solar_subsystem, '/Solar Array'];
mppt_block = [solar_subsystem, '/MPPT'];
if ~exist_block(solar_block)
    add_block([task3_model, '/Solar Array'], solar_block, 'Position', [225, 215, 365, 295]);
end
if ~exist_block(mppt_block)
    add_block([task3_model, '/MPPT'], mppt_block, 'Position', [500, 70, 680, 185]);
end

connect_if_missing(solar_subsystem, [solar_subsystem, '/Simulink-PS Converter3'], 'Outport', 1, solar_block, 'LConn', 1);
connect_if_missing(solar_subsystem, solar_block, 'LConn', 2, [solar_subsystem, '/Current Sensor1'], 'LConn', 1);
connect_if_missing(solar_subsystem, [solar_subsystem, '/Current Sensor1'], 'RConn', 2, [solar_subsystem, '/Average-Value DC-DC Converter'], 'LConn', 2);
connect_if_missing(solar_subsystem, [solar_subsystem, '/Voltage Sensor1'], 'LConn', 1, [solar_subsystem, '/Current Sensor1'], 'RConn', 2);
connect_if_missing(solar_subsystem, solar_block, 'RConn', 1, [solar_subsystem, '/Voltage Sensor1'], 'RConn', 2);
connect_if_missing(solar_subsystem, [solar_subsystem, '/PS-Simulink Converter4'], 'Outport', 1, mppt_block, 'Inport', 1);
connect_if_missing(solar_subsystem, [solar_subsystem, '/PS-Simulink Converter3'], 'Outport', 1, mppt_block, 'Inport', 2);
connect_if_missing(solar_subsystem, mppt_block, 'Outport', 1, [solar_subsystem, '/Simulink-PS Converter1'], 'Inport', 1);

setpoint_block = [battery_subsystem, '/Voltage Set Point'];
sum_block = [battery_subsystem, '/Sum'];
integrator_block = [battery_subsystem, '/Discrete-Time Integrator'];
if ~exist_block(setpoint_block)
    add_block([task4_model, '/Voltage Set Point'], setpoint_block, 'Position', [705, 205, 735, 235]);
end
if ~exist_block(sum_block)
    add_block([task4_model, '/Sum'], sum_block, 'Position', [760, 208, 780, 228]);
end
if ~exist_block(integrator_block)
    add_block([task4_model, '/Discrete-Time Integrator'], integrator_block, 'Position', [815, 202, 850, 238]);
end

connect_if_missing(battery_subsystem, setpoint_block, 'Outport', 1, sum_block, 'Inport', 1);
connect_if_missing(battery_subsystem, [battery_subsystem, '/Load Voltage'], 'Outport', 1, sum_block, 'Inport', 2);
connect_if_missing(battery_subsystem, sum_block, 'Outport', 1, integrator_block, 'Inport', 1);
connect_if_missing(battery_subsystem, integrator_block, 'Outport', 1, [battery_subsystem, '/Simulink-PS Converter2'], 'Inport', 1);

save_system(runner_model, runner_file);
end

function tf = exist_block(block_path)
try
    get_param(block_path, 'Handle');
    tf = true;
catch
    tf = false;
end
end

function connect_if_missing(system_path, src_block, src_field, src_index, dst_block, dst_field, dst_index)
src_handles = get_param(src_block, 'PortHandles');
src_handle = src_handles.(src_field)(src_index);
existing_line = get_param(src_handle, 'Line');
if existing_line ~= -1
    line_name = get_param(existing_line, 'Name');
    if ~isempty(line_name)
        % keep the existing branch and still attempt to branch below
    end
end

dst_handles = get_param(dst_block, 'PortHandles');
dst_handle = dst_handles.(dst_field)(dst_index);
dst_line = get_param(dst_handle, 'Line');
if dst_line ~= -1
    return;
end

add_line(system_path, src_handle, dst_handle, 'autorouting', 'on');
end
