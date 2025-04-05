#!/usr/bin/python
'''
Script for both training and evaluating the DLoc network
Automatically imports the parameters from params.py.
For further details onto which params file to load
read the README in `params_storage` folder.
'''

from comet_ml import Experiment
import torch
from torchvision import transforms
import warnings
with warnings.catch_warnings():
    warnings.filterwarnings("ignore",category=FutureWarning)
from utils import *
from modelADT import ModelADT
from Generators import *
from data_loader import load_data
from data_loader import DLocDataset
from data_loader import ToTensor
from joint_model import Enc_2Dec_Network
from joint_model import Enc_Dec_Network
from params import *
import trainer

torch.manual_seed(0)
np.random.seed(0)


hyper_params = {
    "exp_time_name": opt_exp.save_name,
    "input_data": opt_exp.data,
    "learning_rate": opt_exp.lr,
    "num_layers": opt_exp.n_decoders,
    "decoder_loss": opt_decoder.loss_type,
    "decoder_lambda": opt_decoder.lambda_L,
    "decoder_reg": opt_decoder.lambda_reg,
    "batch_size": opt_exp.batch_size,
    "num_epochs": opt_exp.n_epochs,
    "offset_decoder_loss": opt_offset_decoder.loss_type,
    "offset_decoder_lambda": opt_offset_decoder.lambda_L,
    "offset_decoder_reg": opt_offset_decoder.lambda_reg
}


experiment = Experiment(
    api_key= "FK3LU1vW3ze0aSqSrvbX6ctSL",
    project_name="DLoc_s2r")
experiment.log_parameters(hyper_params)

'''
Defining the paths from where to Load Data.
Assumes that the data is stored in a subfolder called data in the current data folder
'''
# trainpath = ['/media/datadisk/Roshan/datasets/DLoc_sim_data/features/jacobs_July28/',
#              '/media/datadisk/Roshan/datasets/DLoc_sim_data/features/jacobs_July28_2/']

# validpath = ['/media/ehdd_8t1/chenfeng/    `/dataset_jacobs_July28/features/ind_valid/']

# testpath = ['/media/ehdd_8t1/chenfeng/DLoc_data/dataset_jacobs_July28/features/ind_test/']

name = 'dataset_jacobs_July28'
# Loading Training and Evaluation Data into their respective Dataloaders

# datapath = [f'../../DLoc_data_split/{'dataset_jacobs_July28'}/features_aoa/ind']
# trainpath =  [f'../../DLoc_data_split/{'dataset_jacobs_July28'}/f_aoa_non-disjoint_10/train']
# # validpath = [f'../datasets/{name}/features/ind_valid']
# testpath =  [f'../../DLoc_data_split/{'dataset_jacobs_July28'}/f_aoa_non-disjoint_10/test']
# datapath = ['./data/{name}/features_aoa/ind']
trainpath =  ['./data/dataset_jacobs_July28/features/ind_train']
validpath = ['./data/dataset_jacobs_July28/features/ind_valid']
testpath =  ['./data/dataset_jacobs_July28/features/ind_test']
# Loading Training and Evaluation Data into their respective Dataloaders
if not opt_exp.disjoint:
    dataset = DLocDataset(datapath,
                                transform=transforms.Compose([ToTensor()]),
                                opt = opt_exp
                            )
    train_data = torch.utils.data.Subset(dataset, range(0, int(0.7 * len(dataset))))  # 70% for training
    valid_data = torch.utils.data.Subset(dataset, range(int(0.7 * len(dataset)), int(.9 * len(dataset))))  # 20% for validation
    test_data = torch.utils.data.Subset(dataset, range(int(.9 * len(dataset)), len(dataset)))
    train_size = int(0.7 * len(train_data))
    valid_size = int(0.1 * len(train_data))
    test_size = len(train_data) - train_size - valid_size
    train_data, valid_data, test_data = torch.utils.data.random_split(train_data, [train_size, valid_size, test_size])
    train_loader = torch.utils.data.DataLoader(train_data,
                                num_workers=int(opt_exp.num_threads),
                                batch_size=opt_exp.batch_size, drop_last=True, shuffle=False)
    valid_loader = torch.utils.data.DataLoader(valid_data,
                                num_workers=int(opt_exp.num_threads),
                                batch_size=opt_exp.batch_size, drop_last=True, shuffle=False)
    test_loader = torch.utils.data.DataLoader(test_data,
                                num_workers=int(opt_exp.num_threads),
                                batch_size=opt_exp.batch_size, drop_last=True, shuffle=False)
else:
    train_data = DLocDataset(trainpath,
                              transform=transforms.Compose([ToTensor()]))
    valid_data = DLocDataset(validpath,
                              transform=transforms.Compose([ToTensor()]))
    test_data = DLocDataset(testpath,
                              transform=transforms.Compose([ToTensor()]))
    # train_size = int(.8 * len(train_data))
    # valid_size = len(train_data) - train_size
    test_size = len(test_data)
    # train_data, valid_data = torch.utils.data.random_split(train_data, [train_size, valid_size])
    train_loader =torch.utils.data.DataLoader(train_data,
                              num_workers=int(opt_exp.num_threads),
                              batch_size=opt_exp.batch_size, drop_last=True)
    valid_loader =torch.utils.data.DataLoader(valid_data,
                              num_workers=int(opt_exp.num_threads),
                              batch_size=opt_exp.batch_size, drop_last=True)
    test_loader = torch.utils.data.DataLoader(test_data,
                              num_workers=int(opt_exp.num_threads),
                              batch_size=opt_exp.batch_size, drop_last = True)

# # load traning datadatapath = [f'../../DLoc_data_split/{name}/features_aoa/ind']
# trainpath =  [f'../../DLoc_data_split/{name}/f_aoa_non-disjoint_10/train']
# # validpath = [f'../datasets/{name}/features/ind_valid']
# testpath =  [f'../../DLoc_data_split/{name}/f_aoa_non-disjoint_10/test']
# # Loading Training and Evaluation Data into their respective Dataloaders
# if not opt_exp.disjoint:
#     dataset = DLocDataset(datapath,
#                                 transform=transforms.Compose([ToTensor()]),
#                                 opt = opt_exp
#                             )
#     train_data = torch.utils.data.Subset(dataset, range(0, int(0.7 * len(dataset))))  # 70% for training
#     valid_data = torch.utils.data.Subset(dataset, range(int(0.7 * len(dataset)), int(.9 * len(dataset))))  # 20% for validation
#     test_data = torch.utils.data.Subset(dataset, range(int(.9 * len(dataset)), len(dataset)))
#     # train_size = int(0.7 * len(train_data))
#     # valid_size = int(0.1 * len(train_data))
#     # test_size = len(train_data) - train_size - valid_size
#     # train_data, valid_data, test_data = torch.utils.data.random_split(train_data, [train_size, valid_size, test_size])
#     train_loader = torch.utils.data.DataLoader(train_data,
#                                 num_workers=int(opt_exp.num_threads),
#                                 batch_size=opt_exp.batch_size, drop_last=True, shuffle=False)
#     valid_loader = torch.utils.data.DataLoader(valid_data,
#                                 num_workers=int(opt_exp.num_threads),
#                                 batch_size=opt_exp.batch_size, drop_last=True, shuffle=False)
#     test_loader = torch.utils.data.DataLoader(test_data,
#                                 num_workers=int(opt_exp.num_threads),
#                                 batch_size=opt_exp.batch_size, drop_last=True, shuffle=False)
# else:
#     train_data = DLocDataset(trainpath,
#                               transform=transforms.Compose([ToTensor()]), opt=opt_exp)
#     test_data = DLocDataset(testpath,
#                               transform=transforms.Compose([ToTensor()]), opt=opt_exp)
#     train_size = int(.8 * len(train_data))
#     valid_size = len(train_data) - train_size
#     test_size = len(test_data)
#     train_data, valid_data = torch.utils.data.random_split(train_data, [train_size, valid_size])
#     train_loader =torch.utils.data.DataLoader(train_data,
#                               num_workers=int(opt_exp.num_threads),
#                               batch_size=opt_exp.batch_size, drop_last=True)
#     valid_loader =torch.utils.data.DataLoader(valid_data,
#                               num_workers=int(opt_exp.num_threads),
#                               batch_size=opt_exp.batch_size, drop_last=True)
#     test_loader = torch.utils.data.DataLoader(test_data,
#                               num_workers=int(opt_exp.num_threads),
#                               batch_size=opt_exp.batch_size, drop_last = True)



train_data = DLocDataset(root_dir=trainpath,
                          transform=transforms.Compose([ToTensor()]))

valid_data = DLocDataset(root_dir=validpath,
                          transform=transforms.Compose([ToTensor()]))


train_loader =torch.utils.data.DataLoader(train_data,
                                          batch_size=opt_exp.batch_size,
                                          shuffle=True,
                                          num_workers=opt_exp.num_threads)
sample = train_data[[1]]

print('Length of training data = %d' % len(train_data))
print(f"Size of features wo offset {sample['features_wo_offset'].size()}")
print(f"Size of features w offset  {sample['features_w_offset'].size()}")
print(f"Size of labels             {sample['labels_gaussian_2d'].size()}")
print('# training mini batch = %d' % len(train_loader))

# Load Validation data
valid_loader =torch.utils.data.DataLoader(valid_data,
                                          batch_size=opt_exp.batch_size,
                                          shuffle=True,
                                          num_workers=opt_exp.num_threads)
sample = valid_data[[1]]

print('Length of Validation data = %d' % len(valid_data))
print(f"Size of features wo offset {sample['features_wo_offset'].size()}")
print(f"Size of features w offset  {sample['features_w_offset'].size()}")
print(f"Size of labels             {sample['labels_gaussian_2d'].size()}")
print('# validation mini batch = %d' % len(valid_loader))

'''
Initiate the Network and build the graph
'''

# init encoder
enc_model = ModelADT()
enc_model.initialize(opt_encoder)
enc_model.setup(opt_encoder)

# init decoder1
dec_model = ModelADT()
dec_model.initialize(opt_decoder)
dec_model.setup(opt_decoder)

if opt_exp.n_decoders == 2:
    # init decoder2
    offset_dec_model = ModelADT()
    offset_dec_model.initialize(opt_offset_decoder)
    offset_dec_model.setup(opt_offset_decoder)

    # join all models
    print('Making the joint_model')
    joint_model = Enc_2Dec_Network()
    joint_model.initialize(opt_exp, enc_model, dec_model, offset_dec_model, gpu_ids=opt_exp.gpu_ids)

elif opt_exp.n_decoders == 1:
    # join all models
    print('Making the joint_model')
    joint_model = Enc_Dec_Network()
    joint_model.initialize(opt_exp, enc_model, dec_model, gpu_ids=opt_exp.gpu_ids)

else:
    print('Incorrect number of Decoders specified in the parameters')

if opt_exp.continue_train:
    enc_model.load_networks(opt_encoder.starting_epoch_count)
    dec_model.load_networks(opt_decoder.starting_epoch_count)
    if opt_exp.n_decoders == 2:
        offset_dec_model.load_networks(opt_offset_decoder.starting_epoch_count)

# train the model
'''
Trainig the network
'''
if opt_exp.isTrain:
    trainer.train(joint_model, train_loader, valid_loader, experiment)

'''
Model Evaluation at the best epoch
'''


# Load Testing data
test_data = DLocDataset(root_dir=testpath,
                          transform=transforms.Compose([ToTensor()]))

test_loader =torch.utils.data.DataLoader(test_data,
                                          batch_size=opt_exp.batch_size,
                                          shuffle=False,
                                          num_workers=opt_exp.num_threads)
sample = test_data[[1]]

print('Length of Testing data = %d' % len(test_data))
print(f"Size of features wo offset {sample['features_wo_offset'].size()}")
print(f"Size of features w offset  {sample['features_w_offset'].size()}")
print(f"Size of labels             {sample['labels_gaussian_2d'].size()}")
print('# testing mini batch = %d' % len(test_loader))


epoch = "best"  # int/"best"/"latest"
# load network
enc_model.load_networks(epoch)
dec_model.load_networks(epoch)
if opt_exp.n_decoders == 2:
    offset_dec_model.load_networks(epoch)
    joint_model.initialize(opt_exp, enc_model, dec_model, offset_dec_model, gpu_ids = opt_exp.gpu_ids)
elif opt_exp.n_decoders == 1:
    joint_model.initialize(opt_exp, enc_model, dec_model, gpu_ids = opt_exp.gpu_ids)

# pass data through model
total_loss, median_error = trainer.test(joint_model,
    test_loader, 
    save_output=True,
    save_dir=f"{opt_exp.results_dir}/best",
    save_name=f"decoder_test_result_epoch_{epoch}",
    log=True,
    experiment=experiment)
print(f"total_loss: {total_loss}, median_error: {median_error}")
