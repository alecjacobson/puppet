%prefix = 'scorpion';
%[V,F] = load_mesh([prefix '-model.obj']);
%[C,BE] = readTGF([prefix '-skeleton.tgf']);
%P = bone_parents(BE);
%
%% rigid part in middle
%RP = 1*~((abs(C(BE(:,1),3))<0.01 | abs(C(BE(:,2),3))<0.01) & ...
%  (C(BE(:,1),1)>-0.2 & C(BE(:,2),1)>-0.2));
%
%RP(RP==1) = 1:sum(RP==1);
%RP = RP+1;
%
%%[BE P RP]
%
%samples_per_edge = 10;
%% compute samples along bones
%CI = sample_edges(C,BE,samples_per_edge);
%% use tetgen to mesh interior with tetrahedra
%[TV,TT,TF] = tetgen([V;CI],F,[],false);
%[b,bc] = boundary_conditions(TV,TT,C,[],BE,[]);
%A = sparse(1:numel(RP),RP,1,size(bc,2),max(RP));
%bc = bc*A;
%
%W = biharmonic_bounded(TV,TT,b,bc);
W = bsxfun(@rdivide,W,sum(W,2));
writeMESH([prefix '-model.mesh'],TV,TT,boundary_faces(TT));
writeDMAT([prefix '-weights.dmat'],W);
save([prefix '-skeleton.mat'],'C','BE','RP','P');

