function save_task_outputs(cfg, task_name, result_dir, summary_table, run_results, best_result, fig_handles)
if nargin < 7
    fig_handles = gobjects(0);
end

if ~istable(summary_table)
    summary_table = struct2table(summary_table);
end

writetable(summary_table, fullfile(result_dir, 'summary.csv'));
save(fullfile(result_dir, 'all_runs.mat'), 'cfg', 'summary_table', 'run_results', '-v7.3');
save(fullfile(result_dir, 'best_result.mat'), 'best_result');

fig_dir = fullfile(result_dir, 'figures');
for idx = 1:numel(fig_handles)
    if ~ishandle(fig_handles(idx))
        continue;
    end
    file_name = get(fig_handles(idx), 'Tag');
    if isempty(file_name)
        file_name = sprintf('%s_%02d', task_name, idx);
    end
    file_name = regexprep(file_name, '[^A-Za-z0-9_\-]', '_');
    exportgraphics(fig_handles(idx), fullfile(fig_dir, [file_name, '.png']));
end
end
