function features = generate_features_abs(channels,ap,theta_vals,d_vals,d1,d2,AP_INDEX,opt)

n_ap=length(ap);

features = zeros(n_ap,length(d2),length(d1));

for j=1:n_ap
    if(AP_INDEX==2)
        P = compute_multipath_profile2d_fast_edit(squeeze(channels(:,j,:)),theta_vals,d_vals,opt);
    elseif(AP_INDEX==3)
        P = compute_multipath_profile2d_fast_edit(squeeze(channels(:,:,j)),theta_vals,d_vals,opt);
    end
    P_out = convert_spotfi_to_2d(P,theta_vals,d_vals,d1,d2,ap{j});
    features(j,:,:) = abs(P_out)./max(abs(P_out(:)));
end

end