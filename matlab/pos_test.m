% Example script to control and read-out position

x_pos = calllib('smx','umanipulatorctl_x_position',smx_handle);
y_pos = calllib('smx','umanipulatorctl_y_position',smx_handle);
z_pos = calllib('smx','umanipulatorctl_z_position',smx_handle);

figure
plot3(x_pos, y_pos, z_pos,'X')
axis([0 20000 0 20000 0 20000])
grid on
hold on

final_target_x = 10000;
final_target_y = 1000;
final_target_z = 10000;

calllib('smx','umanipulatorctl_store_mem_position',smx_handle, final_target_x, final_target_y, final_target_z);
calllib('smx','umanipulatorctl_goto_mem_position',smx_handle);


