% Generate DLoc datasets from P2SLAm data
% size(channels_cli) = n_times*n_sub*n_ap*n_rx(ap_ant)*n_tx(bot_ant)
clearvars;
close all;
warning('off','signal:findpeaks:largeMinPeakHeight');
%%
N_CSI_CHUNKS = 20;
STORE_TYPE = 'individual'; % store each datapoint 'individual'/'batch'/'chunks'
BATCH_SIZE = 32; % batch size for the 'batch' storage
N_CHUNKS = 50; % number of chunks for the 'chunks' storage
DATA_LOAD_TOP = '/media/ehdd_8t1/aarun/Research/data/p2slam_realworld/p2slam_atk'; % top level data directory
DATA_SAVE_TOP = '/media/ehdd_8t1/chenfeng/DLoc_data/'; % top level data directory
THETA_VALS = -pi/2:0.01:pi/2;
D_VALS = -5:0.25:65;
ANT_SEP = 0.0259;
SUB_INDCS = [-122:-104,-102:-76,-74:-40,-38:-12,-10:-2,2:10,12:38,40:74,76:102,104:122];% 80MHz
GRID_SIZE = 0.25;
CHAN = 155;
BW = 80e6;
AP_INDEX = 2;
OUTPUT_SIGMA = 0.25;
FREQ = double(5e9 + 5*CHAN*1e6) + SUB_INDCS.*BW./256; % 80MHz
LAMBDA = 3e8./FREQ;
opt.lambda = LAMBDA;
opt.freq = FREQ;
opt.ant_sep = ANT_SEP;
PROCESS_CHANNELS = 1;
PROCESS_FEATURES = 1;
%%
% list of all the available dataq collections
datasets = {'8-4-atkinson2','8-25-atkinson-4th-oneloop','8-26-atkinson-4th','8-28-edge-aps-3'};

for data_number = 4
disp(['Working with ', datasets{data_number}, ' dataset'])
dataset = datasets{data_number};
if(PROCESS_CHANNELS)
    load(fullfile(DATA_LOAD_TOP,dataset,'channels_atk.mat'),'channels_cli','labels','ap');

    switch dataset
        case '8-26-atkinson-4th'
            n_points = 30000;
        otherwise
            dataset = [dataset, '_1ant'];
            n_points = size(channels_cli,1);
    end
    [~,n_sub,n_ap,n_ant,n_bot_ant] = size(channels_cli(1:n_points,:,:,:,:));
%     channels_all = zeros(n_points*n_bot_ant, n_sub, n_ap, n_ant);
%     labels_new = zeros(n_points*n_bot_ant, 2);
%     gt_tof = zeros(n_points*n_bot_ant, n_ap);
%     channels_all = zeros(n_points, n_sub, n_ap, n_ant);
%     labels_new = zeros(n_points, 2);
    gt_tof = zeros(n_points, n_ap);
%     for i=1:n_bot_ant
%         channels_all(i:n_bot_ant:end-n_bot_ant+i, :, :, :) = squeeze(channels_cli(1:n_points,:,:,:,i));
%         labels_new(i:n_bot_ant:end-n_bot_ant+i, :) = labels(1:n_points, 1:2);
%     end
    channels_all = squeeze(channels_cli(1:n_points,:,:,:,1));
    labels_new = labels(1:n_points, 1:2);
    for i = 1:n_ap
        gt_tof(:,i) = vecnorm(labels_new-mean(ap{i}),2,2);
    end
    clearvars channels_cli labels;
    %%
    x_width = max(labels_new(:,1)) - min(labels_new(:,1));
    y_width = max(labels_new(:,2)) - min(labels_new(:,2));

    x_min_new = min(labels_new(:,1)) - 0.5 * x_width;
    x_max_new = max(labels_new(:,1)) + 0.5 * x_width;

    y_min_new = min(labels_new(:,2)) - 0.5 * y_width;
    y_max_new = max(labels_new(:,2)) + 0.5 * y_width;

    x_values = x_min_new:GRID_SIZE:x_max_new; % x axis grid points
    y_values = y_min_new:GRID_SIZE:y_max_new; % y axis grid points

    fprintf('original (x_min, x_max): (%f,%f), new (x_min, x_max): (%f,%f)\n',...
        min(labels_new(:,1)),...
        max(labels_new(:,1)),...
        x_min_new,...
        x_max_new)
    fprintf('original (y_min, y_max): (%f,%f), new (y_min, y_max): (%f,%f)\n',...
        min(labels_new(:,2)),...
        max(labels_new(:,2)),...
        y_min_new,...
        y_max_new)
    %%
    n_points = size(channels_all,1);
    channels_wo_offset_all = zeros(size(channels_all));
    for i=1:n_points
        for j=1:n_ap
            P_tof = compute_distance_profile_music_fast(squeeze(channels_all(i,:,j,:)),LAMBDA,2,D_VALS,0.1);   
            thresh = 0.5*max(abs(P_tof));
            [pks_tof,vec_tof] = findpeaks(abs(P_tof),'MinPeakHeight',thresh);
            if(isempty(vec_tof))
                [~,vec_tof_temp] = max(abs(P_tof));
                vec_tof = vec_tof_temp(1);
            end

            channels_wo_offset_all(i,:,j,:) = squeeze(channels_all(i,:,j,:)).*exp( 1j*2*pi*FREQ.'*( D_VALS(vec_tof(1)) - gt_tof(i,j) )./3e8 );
        end

        if(mod(i,1000)==0)
            fprintf('Generating channels, sample %d\n',i);
        end
    end

    if ~exist(fullfile(DATA_SAVE_TOP,dataset), 'dir')
       mkdir(fullfile(DATA_SAVE_TOP,dataset))
       mkdir(fullfile(DATA_SAVE_TOP,dataset,'channels'))
    elseif ~exist(fullfile(DATA_SAVE_TOP,dataset,'channels'), 'dir')
        mkdir(fullfile(DATA_SAVE_TOP,dataset,'channels'))
    end
    n_points = size(channels_all,1);
    n_points_per_set = ceil(n_points/N_CSI_CHUNKS);
    for n_set=1:N_CSI_CHUNKS
        if n_set<N_CSI_CHUNKS
            channels = channels_all((n_set-1)*n_points_per_set+(1:n_points_per_set),:,:,:);
            channels_wo_offset = channels_wo_offset_all((n_set-1)*n_points_per_set+(1:n_points_per_set),:,:,:);
            labels = labels_new((n_set-1)*n_points_per_set+(1:n_points_per_set),:);
        elseif n_set==N_CSI_CHUNKS
            channels = channels_all((n_set-1)*n_points_per_set+1:end,:,:,:);
            channels_wo_offset = channels_wo_offset_all((n_set-1)*n_points_per_set+1:end,:,:,:);
            labels = labels_new((n_set-1)*n_points_per_set+1:end,:);
        end
        save(fullfile(DATA_SAVE_TOP,dataset,'channels',['subset',num2str(n_set),'.mat']),'channels','channels_wo_offset','labels','ap','opt','x_values','y_values','-v7.3')
    end
    clear channels channels_all channels_wo_offset channels_wo_offset_all
end
%% Get features
if(PROCESS_FEATURES)
if ~exist(fullfile(DATA_SAVE_TOP,dataset), 'dir')
   mkdir(fullfile(DATA_SAVE_TOP,dataset))
elseif ~exist(fullfile(DATA_SAVE_TOP,dataset,'features'), 'dir')
    mkdir(fullfile(DATA_SAVE_TOP,dataset,'features'))
end
n_start = 0;
for n_set = 1:N_CSI_CHUNKS
    disp(['Working with ',num2str(n_set),' CSI chunk of data'])
    load(fullfile(DATA_SAVE_TOP,dataset,'channels',['subset',num2str(n_set),'.mat']));
    [n_points,n_sub,n_ap,n_ant] = size(channels);
    tic
    features_w_offset_all = zeros(n_points,n_ap,length(y_values),length(x_values));
    features_wo_offset_all = zeros(n_points,n_ap,length(y_values),length(x_values));
    parfor i=1:n_points
        features_w_offset_all(i,:,:,:) = generate_features_abs(squeeze(channels(i,:,:,:)),...
            ap,...
            THETA_VALS,...
            D_VALS,...
            x_values,...
            y_values,...
            AP_INDEX,...
            opt);

        features_wo_offset_all(i,:,:,:) = generate_features_abs(squeeze(channels_wo_offset(i,:,:,:)),...
            ap,...
            THETA_VALS,...
            D_VALS,...
            x_values,...
            y_values,...
            AP_INDEX,...
            opt);

        if(mod(i,500)==0)
            fprintf('Generating features, sample %d\n',i);
        end
    end
    toc

    clearvars channels channels_wo_offset
    %% create ground truth label for training
    labels_gaussian_2d_all = get_gaussian_labels(labels,...
        OUTPUT_SIGMA,...
        x_values,...
        y_values);
    %% saving files
    %save types 'individual'/'batch'/'chunks'
    switch STORE_TYPE
        case 'chunks'
            try
                h5create(fullfile(DATA_SAVE_TOP,dataset,'features',['subset',num2str(n_set),'.h5']),'/features_w_offset',size(features_w_offset_all));
                h5create(fullfile(DATA_SAVE_TOP,dataset,'features',['subset',num2str(n_set),'.h5']),'/features_wo_offset',size(features_wo_offset_all));
                h5create(fullfile(DATA_SAVE_TOP,dataset,'features',['subset',num2str(n_set),'.h5']),'/labels',size(labels));
                h5create(fullfile(DATA_SAVE_TOP,dataset,'features',['subset',num2str(n_set),'.h5']),'/labels_gaussian_2d',size(labels_gaussian_2d_all));
            catch
                disp('Files already Exist')
            end

            h5write(fullfile(DATA_SAVE_TOP,dataset,'features',['subset',num2str(n_set),'.h5']),'/features_w_offset',features_w_offset_all);
            h5write(fullfile(DATA_SAVE_TOP,dataset,'features',['subset',num2str(n_set),'.h5']),'/features_wo_offset',features_wo_offset_all);
            h5write(fullfile(DATA_SAVE_TOP,dataset,'features',['subset',num2str(n_set),'.h5']),'/labels',labels);
            h5write(fullfile(DATA_SAVE_TOP,dataset,'features',['subset',num2str(n_set),'.h5']),'/labels_gaussian_2d',labels_gaussian_2d_all);
        case 'batch'
            if ~exist(fullfile(DATA_SAVE_TOP,dataset), 'dir')
               mkdir(fullfile(DATA_SAVE_TOP,dataset))
            elseif ~exist(fullfile(DATA_SAVE_TOP,dataset,'features'), 'dir')
                mkdir(fullfile(DATA_SAVE_TOP,dataset,'features'))
            elseif ~exist(fullfile(DATA_SAVE_TOP,dataset,'features','batch'), 'dir')
                mkdir(fullfile(DATA_SAVE_TOP,dataset,'features','batch'))
            end
            for i=1:BATCH_SIZE:n_points-BATCH_SIZE
                start_idx = i;
                stop_idx = i+BATCH_SIZE-1;
                features_wo_offset = features_wo_offset_all(start_idx:stop_idx,:,:,:);
                features_w_offset = features_w_offset_all(start_idx:stop_idx,:,:,:);
                labels_gaussian_2d = labels_gaussian_2d_all(start_idx:stop_idx,:,:);
                labels_discrete = labels(start_idx:stop_idx,:,:);
                if (mod(i,1000)==0)
                    fprintf('Saving....BAtch%d.h5\n',int32(stop_idx/BATCH_SIZE));
                end
                fname = [num2str(int32(stop_idx/BATCH_SIZE)),'.h5'];
                try
                    h5create(fullfile(DATA_SAVE_TOP,dataset,'features','batch',fname),...
                        '/features_w_offset',size(features_w_offset));
                    h5create(fullfile(DATA_SAVE_TOP,dataset,'features','batch',fname),...
                        '/features_wo_offset',size(features_wo_offset));
                    h5create(fullfile(DATA_SAVE_TOP,dataset,'features','batch',fname),...
                        '/labels',size(labels_discrete));
                    h5create(fullfile(DATA_SAVE_TOP,dataset,'features','batch',fname),...
                        '/labels_gaussian_2d',size(labels_gaussian_2d));
                catch
                    disp('Files already exist');
                end

                h5write(fullfile(DATA_SAVE_TOP,dataset,'features','batch',fname),...
                    '/features_w_offset',features_w_offset);
                h5write(fullfile(DATA_SAVE_TOP,dataset,'features','batch',fname),...
                    '/features_wo_offset',features_wo_offset);
                h5write(fullfile(DATA_SAVE_TOP,dataset,'features','batch',fname),...
                    '/labels',labels_discrete);
                h5write(fullfile(DATA_SAVE_TOP,dataset,'features','batch',fname),...
                    '/labels_gaussian_2d',labels_gaussian_2d);

            end
        case 'individual'
            if ~exist(fullfile(DATA_SAVE_TOP,dataset), 'dir')
               mkdir(fullfile(DATA_SAVE_TOP,dataset))
            elseif ~exist(fullfile(DATA_SAVE_TOP,dataset,'features'), 'dir')
                mkdir(fullfile(DATA_SAVE_TOP,dataset,'features'))
            elseif ~exist(fullfile(DATA_SAVE_TOP,dataset,'features','ind'), 'dir')
                mkdir(fullfile(DATA_SAVE_TOP,dataset,'features','ind'))
            end
            for i=1:n_points
                features_wo_offset = features_wo_offset_all(i,:,:,:);
                features_w_offset = features_w_offset_all(i,:,:,:);
                labels_gaussian_2d = labels_gaussian_2d_all(i,:,:);
                labels_discrete = labels(i,:,:);
                if (mod(i,1000)==0)
                    fprintf('Saving....%d.h5\n',i+n_start);
                end
                fname = [num2str(i+n_start),'.h5'];
                try
                    h5create(fullfile(DATA_SAVE_TOP,dataset,'features','ind',fname),...
                        '/features_w_offset',size(features_w_offset));
                    h5create(fullfile(DATA_SAVE_TOP,dataset,'features','ind',fname),...
                        '/features_wo_offset',size(features_wo_offset));
                    h5create(fullfile(DATA_SAVE_TOP,dataset,'features','ind',fname),...
                        '/labels',size(labels_discrete));
                    h5create(fullfile(DATA_SAVE_TOP,dataset,'features','ind',fname),...
                        '/labels_gaussian_2d',size(labels_gaussian_2d));
                catch
                    disp('Files already exist');
                end

                h5write(fullfile(DATA_SAVE_TOP,dataset,'features','ind',fname),...
                    '/features_w_offset',features_w_offset);
                h5write(fullfile(DATA_SAVE_TOP,dataset,'features','ind',fname),...
                    '/features_wo_offset',features_wo_offset);
                h5write(fullfile(DATA_SAVE_TOP,dataset,'features','ind',fname),...
                    '/labels',labels_discrete);
                h5write(fullfile(DATA_SAVE_TOP,dataset,'features','ind',fname),...
                    '/labels_gaussian_2d',labels_gaussian_2d);

            end
    end
    clearvars features_w_offset_all features_wo_offset_all labels_gaussian_2d
    n_start = n_start + n_points;
%             save(fullfile(DATA_SAVE_TOP,dataset,'features',['subset',num2str(n_set),'.mat']),'channels','channels_wo_offset','labels','x_values','y_values','-v7.3')
end
end

end
