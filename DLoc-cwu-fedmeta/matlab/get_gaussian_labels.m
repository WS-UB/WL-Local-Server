function labels_gaussian = get_gaussian_labels(labels,sigma,x_vals,y_vals)

    %x_vals and y_vals are 1 X N arrays of potential grid valuess
    n_xlabels = length(x_vals);
    n_ylabels =length(y_vals);
    labels_gaussian = zeros(size(labels,1),n_ylabels,n_xlabels);
    map_X = repmat(x_vals,n_ylabels,1);
    map_Y = repmat(y_vals',1,n_xlabels);
    n_points = size(labels,1);
    for i=1:n_points
        d = (map_X-labels(i,1)).^2+(map_Y-labels(i,2)).^2;
        cur_gaussian = exp(-d/sigma/sigma);%*1/sqrt(2*pi)/output_sigma;        
        labels_gaussian(i,:)=cur_gaussian(:);
    
    end

    
end