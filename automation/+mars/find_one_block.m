function block_path = find_one_block(root_path, candidate_names, varargin)
if ischar(candidate_names) || isstring(candidate_names)
    candidate_names = cellstr(candidate_names);
end

matches = {};
for idx = 1:numel(candidate_names)
    hits = find_system(root_path, ...
        'FollowLinks', 'on', ...
        'LookUnderMasks', 'all', ...
        'RegExp', 'off', ...
        'Name', candidate_names{idx}, ...
        varargin{:});
    matches = [matches; hits(:)]; %#ok<AGROW>
end

if isempty(matches)
    error('mars:BlockNotFound', 'No block found under "%s".', root_path);
end

matches = unique(matches);
path_lengths = cellfun(@strlength, matches);
[~, best_idx] = min(path_lengths);
block_path = matches{best_idx};
end
