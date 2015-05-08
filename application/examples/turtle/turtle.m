[V,F] = load_mesh('turtle-orig.obj');
C = connected_components(F);
C(C==4 | C==8) = 2;
OW = full(sparse(1:size(V,1),C,1,size(V,1),max(C)));
W = OW(:,[2 7 1 5 3 6]);
[C,BE] = readTGF('turtle-skeleton.tgf');
RP = (1:size(BE,1))';
%for w = 1:size(W,2)
%tsurf(F,V,'CData',W(:,w),'FaceAlpha',0.2);
%hold on;plot_edges(C,BE,'LineWidth',2,'Color','k');plot_edges(C,BE(w,:),'LineWidth',5,'Color','y');hold off
%input('');
%end
P = [0 3 1 3 3 3];
writeDMAT('turtle-weights.dmat',W);
save('turtle-skeleton.mat',C,BE,RP,P);
save('turtle-skeleton.mat','C','BE','RP','P');
