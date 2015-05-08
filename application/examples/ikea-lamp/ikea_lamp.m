[V,F] = load_mesh('ikea-lamp-original.off');
V = V*axisangle2matrix([0 0 -1],pi*0.0499);
V = V*axisangle2matrix([1 0 0],pi*0.0115);
V = V*axisangle2matrix([0 0 -1],pi*0.5);
V = bsxfun(@rdivide,bsxfun(@minus,V,min(V)),max(max(V)-min(V)));
C = connected_components(F)';
G4 = [1:8 10:15];
G3 = [9 17:23];
G2 = [24 16 27 30:36 38];
I = 1:size(V,1);
G1 = setdiff(I,[G2 G3 G4]);
W = 1*[ismember(C,G1) ismember(C,G2) ismember(C,G3) ismember(C,G4)];
% remove wires
H = [8 23 27];
NH = I(~ismember(C,H));
FF = limit_faces(F,NH);
[VV,I] = remove_unreferenced(V,FF);
FF = I(FF);
WW(I,:) = W;
WW = WW(1:size(VV,1),:);
C = [      0.792-0.39913  0.00055681   0.21949
           0.792-0.3957   0.15832      0.21949
           0.792-0.063877 0.55616      0.21949
           0.792-0.46515  0.8734       0.21949
           0.792-0.61091  0.72679      0.21949];
BE = [1:4;2:5]';
tsurf(FF,VV,'FaceAlpha',0.1,'FaceColor','r');
hold on;
plot_edges(C,BE,'LineWidth',5);
hold off;
writeOFF('ikea-lamp-model.off',VV,FF);
RP = [1:4]';
P = [0:3]';
save('ikea-lamp-skeleton.mat','C','BE','RP','P');
writeDMAT('ikea-lamp-weights.dmat',WW);
