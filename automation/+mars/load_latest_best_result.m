function best_result = load_latest_best_result(cfg, task_name)
best_file = mars.locate_latest_artifact(cfg, task_name, 'best_result.mat');
loaded = load(best_file, 'best_result');
best_result = loaded.best_result;
end
