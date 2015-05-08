%w = 512;
%h = 1516;
%
%% Body (without pelvis)
%[BV,BF] = png2mesh('unipedli-01.png',1,1000);
%
%% Pelvis
%[PV,PF] = png2mesh('unipedli-02.png',1,500);
%% height of pelvis texture
%ph = 152;
%% y-offset of pelvis texture in original
%px = 7;
%py = 149;
%
%V = [BV;PV];
%F = [BF;size(BV,1)+PF];
%UV = [BV;PV(:,1)+px PV(:,2)+py];
%UV = bsxfun(@rdivide,UV,[w h+ph]);
%% flip y
%UV(:,2) = 1-UV(:,2);
%
%CC = connected_components(F);
%tsurf(F,V,'CData',CC);axis equal
%V(:,3) = max(CC)-CC;
%writeOBJ('unipedli.obj',V,F,UV,F);
%!~/Dropbox/texture/texture unipedli.obj unipedli.png
%
%% parts in body half,upper leg, knee, lower leg, ankle foot
%P = get_control_points(V(:,1:2),F);
body =  CC(snap_points(P(1,:),V(:,1:2)));
upper = CC(snap_points(P(2,:),V(:,1:2)));
knee =  CC(snap_points(P(3,:),V(:,1:2)));
lower = CC(snap_points(P(4,:),V(:,1:2)));
ankle = CC(snap_points(P(5,:),V(:,1:2)));
foot =  CC(snap_points(P(6,:),V(:,1:2)));
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

%imshow(imread('unipedli-01.png'));
%[Cx,Cy] = getpts();
C = [Cx,(h+ph)-Cy];
%%C = [ ...
%%   99.99 1323.7
%%  176.52 666.48
%%  166.01 361.87
%%  38.469 82.778
%%     139 12.254];
%BE = [1 2;2 3;3 4;4 5];
deform(V(:,1:2),F,C,'BoneEdges',BE,'DeformMethod','skinning',[zeros(size(V,1),5) W],'InterpMode','DQLBS');

writeDMAT('unipedli.V.dmat',V,false);
writeDMAT('unipedli.F.dmat',F-1,false);
writeDMAT('unipedli.UV.dmat',UV,false);
writeDMAT('unipedli.G.dmat',G-1,false);
writeDMAT('unipedli.C.dmat',C,false);
writeDMAT('unipedli.BE.dmat',BE-1,false);

