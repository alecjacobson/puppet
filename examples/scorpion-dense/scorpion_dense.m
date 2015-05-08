prefix = 'scorpion-dense';
[V,F] = load_mesh([prefix '-model.obj']);
[C,BE] = readTGF([prefix '-skeleton.tgf']);
P = bone_parents(BE);

% don't group chain in belly
RP = zeros(size(BE,1),1);
RP([1 29 30 31]) = 1;
RP([28 4 3]) = 2;
RP([2 7 6]) = 3;
RP([5 10 9]) = 4;
RP([8 12 11]) = 5;
RP(RP==0) = max(RP) + (1:sum(RP==0));

%[BE P RP]

samples_per_edge = 10;
% compute samples along bones
CI = sample_edges(C,BE,samples_per_edge);
% use tetgen to mesh interior with tetrahedra
[TV,TT,TF] = tetgen([V;CI],F,[],false);
[b,bc] = boundary_conditions(TV,TT,C,[],BE,[]);
A = sparse(1:numel(RP),RP,1,size(bc,2),max(RP));
bc = bc*A;

W = biharmonic_bounded(TV,TT,b,bc);
W = bsxfun(@rdivide,W,sum(W,2));
writeMESH([prefix '-model.mesh'],TV,TT,boundary_faces(TT));
writeDMAT([prefix '-weights.dmat'],W);
save([prefix '-skeleton.mat'],'C','BE','RP','P');

