
out_file = '../occupancy.csv';

x_size = 101;
y_size = 101;
z_size = 26;
size_arr = [x_size, y_size, z_size];

x_res = 0.2;
y_res = 0.2;
z_res = 0.2;
res_arr = [x_res, y_res, z_res];

writematrix(size_arr, out_file, 'WriteMode', 'overwrite');
writematrix(res_arr, out_file, 'WriteMode', 'append');

for zi = 1:z_size

    layer = occupancy_grid(:,:,zi);
    layer = layer';

    writematrix(layer, out_file, 'WriteMode', 'append');
end