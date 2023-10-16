
out_file = '../sdf.csv';

x_size = size(SDF, 1);
y_size = size(SDF, 2);
z_size = size(SDF, 3);
size_arr = [x_size, y_size, z_size];

x_res = resolution;
y_res = resolution;
z_res = resolution;
res_arr = [x_res, y_res, z_res];

writematrix(size_arr, out_file, 'WriteMode', 'overwrite');
writematrix(res_arr, out_file, 'WriteMode', 'append');

for zi = 1:z_size

    layer = SDF(:,:,zi);

    writematrix(layer, out_file, 'WriteMode', 'append');
end