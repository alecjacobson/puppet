[V,F] = load_mesh('wackeldackel.obj');
C = [0 0.28 0.125;0 0.3 0.25];
E = 1:2;
tol = 0.025;
S = sample_edges(C,E,10);
T = linspace(0,2*pi,7)';
T = T(1:end-1);

[w,a] = axisanglebetween([0 0 1],C(2,:)-C(1,:));
R = axisangle2matrix(w,a);
SS = zeros(size(S,1)*numel(t)+2,3);
for s = 1:size(S,1)
  for ti = 1:numel(T)
    t = T(ti);
    SS((s-1)*numel(T) + ti,:) = [tol*[cos(t) sin(t)] sqrt(sum((S(s,:)-C(1,:)).^2))];
  end
end
SS(end-1,:) = [0 0 sqrt(sum((C(2,:)-C(1,:)).^2))+tol];
SS(end,:) = [0 0 -tol];
SS = bsxfun(@plus,SS*R',C(1,:));

tsurf(F,V,'FaceAlpha',0.1,'EdgeAlpha',0.1);
hold on;
scatter3(S(:,1),S(:,2),S(:,3),'r')
scatter3(SS(:,1),SS(:,2),SS(:,3),'b')
scatter3(C(:,1),C(:,2),C(:,3),'g','SizeData',100)
hold off;
axis equal
input('');



[TV,TT,TF] = tetgen([V;S;SS],F,'Flags','-Yq2');
writeMESH('wackeldackel.mesh',TV,TT,TF);

[t,sqr_d] = project_to_lines(TV,C(1,:),C(2,:));
t_tol = tol/sqrt(sum((C(2,:)-C(1,:)).^2));
b_neck= find((abs(sqr_d) < (1e-6+tol).^2) & ((t > -(1e-6+t_tol)) & (t < (1+(1e-6+t_tol)))));
b_hind = find(TV(:,3)<0);

tsurf(F,V,'FaceAlpha',0.1,'EdgeAlpha',0.1);
hold on;
scatter3(TV(b_neck,1),TV(b_neck,2),TV(b_neck,3),'r')
hold off;
axis equal;

%
assert(isempty(intersect(b_hind,b_neck)));
sel = -1 + full(sparse(b_hind,1,1,size(TV,1),1)+2*sparse(b_neck,1,1,size(TV,1),1));
writeDMAT('wackeldackel-selection.dmat',sel);

CC = farthest_points(TV,30);
[b,bc] = boundary_conditions(TV,TT,CC);
W = kharmonic(TV,TT,b,bc);
G = partition(W,100);
writeDMAT('wackeldackel-partition.dmat',G);
