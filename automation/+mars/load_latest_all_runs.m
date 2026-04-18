function payload = load_latest_all_runs(cfg, task_name)
all_runs_file = mars.locate_latest_artifact(cfg, task_name, 'all_runs.mat');
payload = load(all_runs_file);
end
