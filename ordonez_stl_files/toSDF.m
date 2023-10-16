
occ = occupancy_grid == 1;
X_occ = X(occ);
Y_occ = Y(occ);
Z_occ = Z(occ);

SDF = inf(size(occupancy_grid));
for i = 1:numel(X_occ)
    Xi = X_occ(i);
    Yi = Y_occ(i);
    Zi = Z_occ(i);
    SDFi = sqrt((X - Xi).^2 + (Y - Yi).^2 + (Z - Zi).^2);
    SDF = min(SDF, SDFi);
end

figure
for L = 1:size(occ, 3)
    contourf(X(:,:,L), Y(:,:,L), SDF(:,:,L))
    pause(0.2)
end