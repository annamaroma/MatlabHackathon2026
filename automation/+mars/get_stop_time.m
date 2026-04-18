function stop_time = get_stop_time(model_name)
expr = get_param(model_name, 'StopTime');
stop_time = str2double(expr);
if ~isnan(stop_time)
    return;
end

stop_time = evalin('base', expr);
end
