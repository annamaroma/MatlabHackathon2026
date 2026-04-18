function runner = create_runner_model(cfg, task_cfg, result_dir, options)
if nargin < 4
    options = struct();
end

runner = struct();
runner.model_name = [task_cfg.model, '_runner'];
runner.model_file = fullfile(result_dir, [runner.model_name, '.slx']);

if bdIsLoaded(runner.model_name)
    close_system(runner.model_name, 0);
end

copyfile(task_cfg.model_file, runner.model_file);
load_system(runner.model_file);

if isfield(options, 'prepare_fcn') && ~isempty(options.prepare_fcn)
    options.prepare_fcn(cfg, task_cfg, runner.model_name, runner.model_file);
end

if isfield(task_cfg, 'logging')
    for idx = 1:size(task_cfg.logging, 1)
        [~, variable_name] = mars.add_logging_block(runner.model_name, task_cfg.logging{idx, 1}, task_cfg.logging{idx, 2});
        runner.logging.(matlab.lang.makeValidName(task_cfg.logging{idx, 2})) = variable_name;
    end
end

save_system(runner.model_name, runner.model_file);
close_system(runner.model_name);
end
