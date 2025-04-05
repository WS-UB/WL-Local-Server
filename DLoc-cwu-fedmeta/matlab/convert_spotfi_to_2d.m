function P_out=convert_spotfi_to_2d(P,theta_vals,d_vals,d1,d2,ap_pos)
% Treat d1 and d2 as x,y positions.
% ap_pos: n_ant by 2 arrays for all the antennas on an AP
% Supply the antenna positions in the same order as the channels used for
% spotfi code
if(size(P,1)~=length(theta_vals) && size(P,2)~=length(d_vals))
    fprintf('Size does not match. Size(P,1) should be length(theta_vals) \n');
end
if(isrow(d1))
    d1=d1';
end
if(isrow(d2))
    d2=d2';
end
if(isrow(theta_vals))
    theta_vals=theta_vals';
end
if(isrow(d_vals))
    d_vals=d_vals';
end
P_out=zeros(length(d2),length(d1));
ap_center = mean(ap_pos);
X=repmat(d1',length(d2),1)-ap_center(1);
Y=repmat(d2,1,length(d1))-ap_center(2);

ap_vec = ap_pos(1,:)-ap_pos(end,:);
for i=1:length(X(:))
    T(i) = sign(sum([X(i),Y(i)].*ap_vec))*(pi/2-acos(abs(sum([X(i),Y(i)].*ap_vec))/norm([X(i),Y(i)])/norm(ap_vec)));
end
D=sqrt(X.^2+Y.^2);
[~,T_IDX]=min(abs(repmat(T(:),1,length(theta_vals))-repmat(theta_vals',length(T(:)),1)),[],2);
[~,D_IDX]=min(abs(repmat(D(:),1,length(d_vals))-repmat(d_vals',length(D(:)),1)),[],2);
IDX=sub2ind(size(P),T_IDX,D_IDX);
P_out(:)=P(IDX);
end