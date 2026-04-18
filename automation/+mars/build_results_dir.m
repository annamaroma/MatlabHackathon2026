function result_dir = build_results_dir(cfg, task_name, suffix)
if nargin < 3
    suffix = '';
end

timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
if ~isempty(suffix)
    safe_suffix = regexprep(suffix, '[^A-Za-z0-9_\-]', '_');
    leaf = [timestamp, '_', safe_suffix];
else
    leaf = timestamp;
end

result_dir = fullfile(cfg.results_root, task_name, leaf);
mkdir(result_dir);
mkdir(fullfile(result_dir, 'figures'));
end
