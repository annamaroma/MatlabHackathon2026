function create_task2_runner_model(~, task_cfg, runner_model, runner_file)
load_system(runner_file);
load_hits = find_system(runner_model, ...
    'FollowLinks', 'on', ...
    'LookUnderMasks', 'all', ...
    'SearchDepth', 1, ...
    'Name', task_cfg.load_block);
if isempty(load_hits)
    load_block = mars.find_one_block(runner_model, task_cfg.load_block);
else
    load_block = load_hits{1};
end

set_param(load_block, task_cfg.load_parameter, task_cfg.load_variable);
save_system(runner_model, runner_file);
end
