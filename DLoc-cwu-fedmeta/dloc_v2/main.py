"""Script for training, validation, and testing of the model."""

from pytorch_lightning.loggers import CometLogger
import pytorch_lightning as pl
from data_module import DLocDataModule
from model import TrigAOAResNetModel
from torchvision import transforms
from utils.schema import LoggerParameters
from utils.config import ExperimentConfig
from pytorch_lightning.callbacks import ModelCheckpoint

#/home/wiloc/Documents/WL-Local-Server/DLoc-sp25-cse302/dloc_v2
#/home/csdue/chongzep/DLoc-cwu-fedmeta/dloc_v2
def main() -> None:
    # config setting
    config = ExperimentConfig(
        experiment_name="testing_ml_model",
        batch_size=16, 
        num_workers=8,
        pre_fetch_factor=2,
        max_epochs=100,
        lr=5e-5,
        train_data_path="/home/csdue/chongzep/DLoc-cwu-fedmeta/dloc_v2/data/train_index.csv",
        val_data_path="/home/csdue/chongzep/DLoc-cwu-fedmeta/dloc_v2/data/validation_index.csv",
    )

    # model setting
    model = TrigAOAResNetModel(lr=config.lr)

    # create data module
    transform = transforms.Compose([transforms.Normalize(mean=[0.1175, 0.1391, 0.1213],
                                                         std=[0.1548, 0.1658, 0.1651])
                                   ])
    data_module = DLocDataModule(train_data_paths=config.train_data_path,
                                 val_data_paths=config.val_data_path,
                                 batch_size=config.batch_size,
                                 num_workers=config.num_workers,
                                 transform=transform,
                                 prefetch_factor=config.pre_fetch_factor)

    # logger setting
    logger_params = LoggerParameters(experiment_name=config.experiment_name)
    comet_logger = CometLogger(
        api_key=logger_params.api_key,
        workspace=logger_params.workspace,
        save_dir=logger_params.save_dir,
        project_name=logger_params.project_name,
        experiment_name=logger_params.experiment_name,
    )   
    comet_logger.log_hyperparams(config.model_dump())


    # checkpoint setting, save a copy of the trained model
    checkpoint_callback = ModelCheckpoint(
        monitor="val_loss",          # Metric to monitor (e.g., validation loss)
        dirpath="saved_models/",     # Directory to save checkpoints
        filename="best_model",       # Checkpoint filename
        save_top_k=1,                # Save only the best model
        mode="min",                  # "min" for loss, "max" for accuracy
    )

    # trainer setting run locally
    # trainer = pl.Trainer(accelerator="cpu",
    #                      logger=comet_logger,
    #                      devices= 1,
    #                      max_epochs=config.max_epochs,
    #                      strategy="single_device",)

    trainer = pl.Trainer(accelerator="gpu",
                         logger=comet_logger,
                         devices=[0],
                         max_epochs=config.max_epochs,
                         strategy="ddp_find_unused_parameters_true",
                         log_every_n_steps=50,
                         callbacks=[checkpoint_callback]
                         )
    
    # training
    trainer.fit(model, data_module)


if __name__ == "__main__":
    main()