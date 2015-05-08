%[V,F] = load_mesh('half-knight-model.off');
%[Co,BEo] = readTGF('half-knight-skeleton.tgf');
%[C,BE] = readTGF('half-knight-skeleton-centered.tgf');
%[f,I] = ismember(sort(BEo,2),sort(BE,2),'rows');
%assert(all(f));
%% remap new to old (could just set BE = BEo)
%BE = BE(I,:);
%writeTGF('half-knight-skeleton-centered.tgf',C,BE);
%RP = readDMAT('half-knight-RP.dmat')+1;
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
%W = bsxfun(@rdivide,W,sum(W,2));
%writeDMAT('half-knight-skeleton-centered.dmat',W);
[b,bc] = boundary_conditions(TV,TT,C,[],BE,[]);
Wall = biharmonic_bounded(TV,TT,b,bc);
Wall = bsxfun(@rdivide,Wall,sum(Wall,2));
writeDMAT('half-knight-skeleton-centered-all.dmat',Wall(1:size(V,1),1));
tgf2bf('half-knight-skeleton-centered.tgf','half-knight-skeleton-centered.bf');
