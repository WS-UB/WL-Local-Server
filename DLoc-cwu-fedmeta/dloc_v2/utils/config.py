import subprocess
from pydantic import BaseModel, Field
from typing import Optional
from pathlib import Path


class ExperimentConfig(BaseModel):
    # training config
    experiment_name: str = Field(..., description="Name of the experiment")
    batch_size: int = Field(16, description="Batch size for training")
    num_workers: int = Field(8, description="Number of workers for data loading")
    pre_fetch_factor: int = Field(2, description="Prefetch factor for data loading")
    max_epochs: int = Field(50, description="Maximum number of epochs for training")
    lr: float = Field(5e-5, description="Learning rate for the optimizer")

    # data config
    train_data_path: Path = Field(..., description="Path to the training data")
    val_data_path: Path = Field(..., description="Path to the validation data")
    test_data_path: Optional[Path] = Field(None, description="Path to the test data")

    # git info
    git_hash: Optional[str] = Field(
        None, description="Git commit hash of the current codebase"
    )
    git_branch: Optional[str] = Field(
        None, description="Git branch of the current codebase"
    )

    def __init__(self, **data):
        super().__init__(**data)
        self.git_hash = self.get_git_hash()
        self.git_branch = self.get_git_branch()

    @staticmethod
    def get_git_hash() -> Optional[str]:
        try:
            return (
                subprocess.check_output(
                    ["git", "rev-parse", "HEAD"], stderr=subprocess.DEVNULL
                )
                .decode("utf-8")
                .strip()
            )
        except Exception:
            return None

    @staticmethod
    def get_git_branch() -> Optional[str]:
        try:
            return (
                subprocess.check_output(
                    ["git", "rev-parse", "--abbrev-ref", "HEAD"],
                    stderr=subprocess.DEVNULL,
                )
                .decode("utf-8")
                .strip()
            )
        except Exception:
            return None
