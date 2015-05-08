w = 256;
h = 1472;

% Body (without pelvis)
[BV,BF] = png2mesh('uniped-01.png',1,700);

% Pelvis
[PV,PF] = png2mesh('uniped-02.png',1,200);
% height of pelvis texture
ph = 196;
% y-offset of pelvis texture in original
py = 836;

V = [BV;PV];
F = [BF;size(BV,1)+PF];
UV = [BV;PV(:,1) PV(:,2)+py];
UV = bsxfun(@rdivide,UV,[w h+ph]);
% flip y
UV(:,2) = 1-UV(:,2);

CC = connected_components(F);
tsurf(F,V,'CData',CC);axis equal
V(:,3) = max(CC)-CC;
writeOBJ('uniped.obj',V,F,UV,F);
%writeOBJ('uniped.obj',BV,BF,BUV,BF);
%!~/Dropbox/texture/texture uniped.obj uniped.png

% parts in body half,upper leg, knee, lower leg, ankle foot
body = unique(CC(V(:,2)>800));
upper = unique(CC(V(:,2)>550 & V(:,2)<600));
knee = CC(snap_points([152 386],V(:,1:2)));
lower = unique(CC(V(:,2)>200 & V(:,2)<250));
ankle = CC(snap_points([51 72],V(:,1:2)));
foot = CC(snap_points([92 25],V(:,1:2)));
G = ...
  1*ismember(CC,body ) + ...
  2*ismember(CC,upper) + ...
  3*ismember(CC,knee) + ...
  4*ismember(CC,lower) + ...
  5*ismember(CC,ankle) + ...
  6*ismember(CC,foot);
tsurf(F,V,'CData',G);

W = zeros(size(V,1),4);

W(ismember(CC,body),1) = 1;
W(ismember(CC,upper),2) = 1;
W(ismember(CC,knee),[2 3]) = 0.5;
W(ismember(CC,lower),3) = 1;
W(ismember(CC,ankle),[3 4]) = 0.5;
W(ismember(CC,foot),4) = 1;
assert(all(sum(W,2) == 1));

C = [ ...
   99.99 1323.7
  176.52 666.48
  166.01 361.87
  38.469 82.778
     139 12.254];
BE = [1 2;2 3;3 4;4 5];
%deform(V(:,1:2),F,C,'BoneEdges',BE,'DeformMethod','skinning',[zeros(size(V,1),5) W],'InterpMode','DQLBS');

writeDMAT('uniped.V.dmat',V,false);
writeDMAT('uniped.F.dmat',F-1,false);
writeDMAT('uniped.UV.dmat',UV,false);
writeDMAT('uniped.G.dmat',G-1,false);
writeDMAT('uniped.C.dmat',C,false);
writeDMAT('uniped.BE.dmat',BE-1,false);
