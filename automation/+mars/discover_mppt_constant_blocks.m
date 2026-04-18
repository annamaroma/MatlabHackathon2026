function block_paths = discover_mppt_constant_blocks(subsystem_path)
block_paths = find_system(subsystem_path, ...
    'SearchDepth', 1, ...
    'FollowLinks', 'on', ...
    'LookUnderMasks', 'all', ...
    'BlockType', 'Constant');

if numel(block_paths) ~= 4
    error('mars:UnexpectedMPPTLayout', 'Expected 4 MPPT parameter constants under %s.', subsystem_path);
end

positions = zeros(numel(block_paths), 1);
for idx = 1:numel(block_paths)
    pos = get_param(block_paths{idx}, 'Position');
    positions(idx) = pos(2);
end

[~, order] = sort(positions, 'ascend');
block_paths = block_paths(order);
end
