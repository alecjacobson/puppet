%prefix = 'chimpanzee-hand';
%[V,F] = load_mesh([prefix '-model.obj']);
%[C,BE] = readTGF([prefix '-skeleton.tgf']);
%P = bone_parents(BE);
%
%% first rigid piece is first two levels
%RP = 1*(P==0);
%RP(P~=0) = RP(P~=0) | RP(P(P~=0));
%RP(RP==0) = 1+(1:sum(RP==0));
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
writeMESH([prefix '-model.mesh'],TV,TT,boundary_faces(TT));
writeDMAT([prefix '-weights.dmat'],W);
save([prefix '-skeleton.mat'],'C','BE','RP','P');
