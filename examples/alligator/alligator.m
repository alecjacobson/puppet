[V,F] = load_mesh('alligator-orig-model.obj');
CC = connected_components(F);
[WV,WF] = winding_number_clean(V,F(CC(F(:,1))==33,:),'SurfaceOnly',true);
[WV,I] = remove_unreferenced(WV,WF);
WF = I(WF);
load('alligator-skeleton.mat');

samples_per_edge = 20;
% compute samples along bones
CI = sample_edges(C,BE,samples_per_edge);
% use tetgen to mesh interior with tetrahedra
[TV,TT,TF] = tetgen([WV;CI],WF,[],false);
[b,bc] = boundary_conditions(TV,TT,C,[],BE,[]);
A = sparse(1:numel(RP),RP,1,size(bc,2),max(RP));
bc = bc*A;

OW = biharmonic_bounded(TV,TT,b,bc);
OW = bsxfun(@rdivide,OW,sum(OW,2));

U = TV;
G = fliplr(boundary_faces(TT));
W = OW;

TVBC = [TV;barycenter(TV,TT)];

% loop over components
for c = 1:max(CC)
  if c ~= 33
    CF = F(CC(F(:,1))==c,:);
    O = outline(CF);
    % take centroid or a point on the boundary
    if isempty(O)
      P = mean(barycenter(V,CF));
      fprintf('empty.\n');
    else
      P = mean(barycenter(V,O));
    end
    % Snap to original tet mesh points
    tp = snap_points(P,TVBC);
    if tp>size(TV,1)
      tp = tp-size(TV,1);
      tp = TT(tp,1);
    end
    Wp = OW(tp,:);
    [CV,I] = remove_unreferenced(V,CF);
    CF = I(CF);
    G = [G;size(U,1)+CF];
    U = [U;CV];
    W = [W;repmat(Wp,size(CV,1),1)];
    %tsurf(G,U);
    %axis equal;
    %title(sprintf('%d',c));
    %input('');
  end
end

[U,I] = remove_unreferenced(U,G);
G = I(G);
W(I,:) = W;
W = W(1:size(U,1),:);
writeOBJ('alligator-model.obj',U,G);
writeDMAT('alligator-weights.dmat',W);
