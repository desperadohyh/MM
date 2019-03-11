function pos=CapPos3(base,DH,RoCap,link_id)
% get the position for 3rd arm

M=cell(1,link_id+1); M{1}=eye(4);
for i=1:link_id
    R=[cos(DH(i,1)) -sin(DH(i,1))*cos(DH(i,4)) sin(DH(i,1))*sin(DH(i,4));
        sin(DH(i,1)) cos(DH(i,1))*cos(DH(i,4)) -cos(DH(i,1))*sin(DH(i,4));
        0  sin(DH(i,4)) cos(DH(i,4))];
    T=[DH(i,3)*cos(DH(i,1));DH(i,3)*sin(DH(i,1));DH(i,2)];
    M{i+1}=M{i}*[R T; zeros(1,3) 1];
end
% position for link_id arm 
for k=1:2
    pos.p(:,k)=M{link_id+1}(1:3,1:3)*RoCap{link_id}.p(:,k)+M{link_id+1}(1:3,4)+base;
end
end