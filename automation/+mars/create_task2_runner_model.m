function create_task2_runner_model(~, ~, runner_model, runner_file)
% Task 2 now relies on the model's built-in variable-resistor load profile.
load_system(runner_file);
save_system(runner_model, runner_file);
end
