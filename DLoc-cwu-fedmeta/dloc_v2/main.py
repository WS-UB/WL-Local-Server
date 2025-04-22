"""Script for training, validation, and testing of the model."""

from pytorch_lightning.loggers import CometLogger
import pytorch_lightning as pl
from data_module import DLocDataModule
from model import TrigAOAResNetModel
from torchvision import transforms
from utils.schema import LoggerParameters
from utils.config import ExperimentConfig

def main() -> None:
    # config setting
    config = ExperimentConfig(
        experiment_name="testing_ml_model",
        batch_size=16,
        num_workers=8,
        pre_fetch_factor=2,
        max_epochs=100,
        lr=5e-5,
        train_data_path="C:\Users\chong\Documents\GitHub\WL-Local-Server\DLoc-cwu-fedmeta\dloc_v2\data\test_data_path\2025-04-20_18-01-50.508.parquet",
        val_data_path="C:\Users\chong\Documents\GitHub\WL-Local-Server\DLoc-cwu-fedmeta\dloc_v2\data\test_data_path\2025-04-20_18-02-51.197.parquet",
    )

    # model setting
    model = TrigAOAResNetModel(lr=config.lr)

    # create data module
    transform = transforms.Compose([transforms.Normalize(mean=[0.1175, 0.1391, 0.1213, 0.1927],
                                                         std=[0.1548, 0.1658, 0.1651, 0.2104])
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

    # trainer setting
    trainer = pl.Trainer(accelerator="gpu",
                         logger=comet_logger,
                         devices=[0],
                         max_epochs=config.max_epochs,
                         strategy="ddp_find_unused_parameters_true",)

    # training
    trainer.fit(model, data_module)

if __name__ == "__main__":
    main()
