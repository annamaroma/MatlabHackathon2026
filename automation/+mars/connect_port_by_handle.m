function connect_port_by_handle(system_path, src_block, src_field, src_index, dst_block, dst_field, dst_index)
src_handles = get_param(src_block, 'PortHandles');
dst_handles = get_param(dst_block, 'PortHandles');
add_line(system_path, src_handles.(src_field)(src_index), dst_handles.(dst_field)(dst_index), 'autorouting', 'on');
end
