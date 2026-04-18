function file_path = locate_latest_artifact(cfg, task_name, file_name)
task_root = fullfile(cfg.results_root, task_name);
entries = dir(task_root);
entries = entries([entries.isdir]);
entries = entries(~ismember({entries.name}, {'.', '..'}));

if isempty(entries)
    error('mars:NoTaskResults', 'No saved results found for %s.', task_name);
end

[~, order] = sort([entries.datenum], 'descend');
entries = entries(order);

file_path = '';
for idx = 1:numel(entries)
    candidate = fullfile(task_root, entries(idx).name, file_name);
    if exist(candidate, 'file')
        file_path = candidate;
        return;
    end
end

error('mars:ArtifactMissing', 'Could not find %s for %s.', file_name, task_name);
end
