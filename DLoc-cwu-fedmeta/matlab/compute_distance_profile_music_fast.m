function P=compute_distance_profile_music_fast(h,lambda,p_factor,d_vals,thresh)

   if(isempty(h))
       P=[];
       return;
   end
   if(isrow(lambda))
       lambda=lambda';
   end
   if(iscolumn(h))
       h=h.';
   end
%    idx=find(~isnan(h));
%    h=h(idx);
%    lambda=lambda(idx);

   H = h*h';
   [V, D] = eig(H);
   d = d_vals;
%    thresh=10;
   nelem = sum(diag(D) > max(diag(D))*thresh);
%    P=zeros(size(d));
%    for ii=1:length(d)
%        e = exp(-1i*p_factor*pi*d(ii)./lambda);
%        P(ii) = 1./abs(e'*V(:, 1:end-nelem)*V(:, 1:end-nelem)'*e);
%    end 
   
   e = exp(-1i*(p_factor*pi./lambda)*d);
   P = diag(1./abs(e'*V(:, 1:end-nelem)*V(:, 1:end-nelem)'*e));
end