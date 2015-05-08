function B = bounce_anim(A)
  % sort by time
  A = sortrows(A,[2 1]);
  C = repmat(A(1:3,:),2,1);
  for f = 2:size(A,1)/3
    prev = A((f-1-1)*3+(1:3),:);
    cur = A((f-1)*3+(1:3),:);
    C = [C;cur;prev;cur;cur];
  end
  % back
  C = [C;flipud(C(4:end-3,:))];
  C(:,2) = reshape(repmat(0:size(C,1)/3-1,3,1),[],1);
  % sort by theta
  B = sortrows(C,[1 2]);
end
