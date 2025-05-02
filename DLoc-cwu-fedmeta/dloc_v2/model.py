"""Model definition and training and validation logic."""
import torch
import torch.nn as nn
import torch.nn.functional as F
import pytorch_lightning as pl
from torchvision import models
from utils.schema import APMetadata, GTlabel, ModelOutput, LossTerms, AoAVisualizationSample
from utils.ray_intersection_solver import solve_ray_intersection_batch
from utils.geometry_utils import cos_angle_sum, sin_angle_sum
from torchmetrics import MetricCollection
from metrics_calculator import AoAAccuracy, LocationAccuracy, MetricNames
from utils.plot_utils import plot_location_pred_vs_gt, plot_aoa_visualization, plot_aoa_error_histograms, plot_gt_vs_pred_aoa
from collections import deque
import pdb

ANGLE_LOSS_MULTIPLIER = 5

def compute_loss(model_output: ModelOutput, gt_label: GTlabel) -> LossTerms:
    """Compute loss function given model output and ground truth label.

    Args:
        model_output: dataclass that hold output of the model
        gt_label: dataclass that hold ground truth label

    Returns:
        LossTerms dataclass that contain all loss terms.
    """
    # compute the loss term
    cos_loss = F.smooth_l1_loss(model_output.cos_aoa, gt_label.cos_aoa)
    sin_loss = F.smooth_l1_loss(model_output.sin_aoa, gt_label.sin_aoa)
    location_loss = F.smooth_l1_loss(model_output.location, gt_label.location)

    # Total loss
    total_loss = cos_loss * ANGLE_LOSS_MULTIPLIER + sin_loss * ANGLE_LOSS_MULTIPLIER + location_loss

    return LossTerms(total_loss=total_loss,
                     cos_loss=cos_loss,
                     sin_loss=sin_loss,
                     location_loss=location_loss)


class ResNetEncoder(nn.Module):
    """ResNet Encoder for single AP"""
    def __init__(self, in_channels: int = 1):
        """Constructor for ResNetEncoder.

        Args:
            in_channels: Number of Conv channel in first conv layer. Defaults to 1.
        """
        super().__init__()
        self.resnet_encoder = models.resnet34(weights=None)
        self.resnet_output_dim = self.resnet_encoder.fc.in_features

        # Customize the first convolutional layer to accept `in_channels` channels
        self.resnet_encoder.conv1 = nn.Conv2d(in_channels, 64, kernel_size=(7, 7), stride=(2, 2), padding=(3, 3), bias=False)

        # Remove the final classification layer
        self.resnet_encoder.fc = nn.Identity()

        # Add linear layer to reduce the output dimension
        self.mlp_output_dim = 64
        self.mlp = nn.Sequential(
            nn.Linear(self.resnet_output_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, self.mlp_output_dim),
            nn.ReLU(),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """Forward pass through the ResNet encoder.

        Args:
            x: AoA-TOF data tensor for single AP, shape (batch_size, 1, height, width)

        Returns:
            ResNet features tensor, shape (batch_size, resnet_output_dim)
        """
        resnet_output = self.resnet_encoder(x) # shape (batch_size, resnet_output_dim)
        mlp_output = self.mlp(resnet_output) # shape (batch_size, mlp_output_dim)
        return mlp_output


class Decoder(nn.Module):
    """Decoder for AoA prediction"""
    def __init__(self, in_features: int, n_ap: int = 1):
        super().__init__()

        # Cos Decoder
        self.cos_decoder = nn.Sequential(
            nn.Linear(in_features, n_ap),
            nn.Sigmoid()
        )

        # Sin Decoder
        self.sin_decoder = nn.Sequential(
            nn.Linear(in_features, n_ap),
            nn.Tanh()
        )

        # Confidence Decoder
        self.confidence_decoder = nn.Sequential(
            nn.Linear(in_features, n_ap),
            nn.Sigmoid()
        )

    def forward(self, x: torch.Tensor) -> ModelOutput:
        """Forward pass through the decoder.

        Args:
            x: ResNet features tensor, shape (batch_size, resnet_output_dim)

        Returns:
            ModelOutput dataclass that contains the model output.
        """
        cos_aoa = self.cos_decoder(x) # shape (batch_size, n_ap)
        sin_aoa = self.sin_decoder(x) # shape (batch_size, n_ap)
        confidence = self.confidence_decoder(x) # shape (batch_size, n_ap)

        return ModelOutput(cos_aoa=cos_aoa,
                           sin_aoa=sin_aoa,
                           confidence=confidence,
                           aoa=None,
                           location=None)


class TrigAOAResNetModel(pl.LightningModule):
    def __init__(self, lr: float = 1e-3):
        super().__init__()
        self.ap_metadata = APMetadata()
        self.lr = lr

        # Initialize metrics calculator
        # Ref: https://lightning.ai/docs/torchmetrics/stable/pages/overview.html#metriccollection
        self.val_metrics = MetricCollection([
            AoAAccuracy(n_aps=self.ap_metadata.n_aps),
            LocationAccuracy()
            ])
        # train metrics is only used to collect location pred and ground truth for plotting
        self.train_metrics = LocationAccuracy()

        # Buffer to store visualization data
        self.val_visualization_data = deque(maxlen=10)

        # validation epoch counter, only increment in validation loop. used to determine when to display visualization data
        self.epoch_counter = 1
        self.display_viz_data_epoch_interval = 5

        # Register buffers to pin the data to the device
        self.register_buffer('ap_locations', self.ap_metadata.ap_locations)
        self.register_buffer('cos_ap_orientations', self.ap_metadata.cos_ap_orientations)
        self.register_buffer('sin_ap_orientations', self.ap_metadata.sin_ap_orientations)

        # ResNet Encoder for single AP
        self.resnet_encoder_list = nn.ModuleList([ResNetEncoder(1) for _ in range(self.ap_metadata.n_aps)])
        encoder_mlp_output_dim = self.resnet_encoder_list[0].mlp_output_dim

        # Initialize decoder
        self.decoder = Decoder(encoder_mlp_output_dim, n_ap=1)

    def forward(self, x: torch.Tensor) -> ModelOutput:
        """Forward pass through the model.

        Args:
            x: input tensor, shape (batch_size, n_aps, height, width)

        Returns:
            A dataclass that contains the model output.
        """
        # permute order of ap during training to improve robustness
        n_aps = self.ap_metadata.n_aps

        # Initialize lists to store the results
        batch_size = x.shape[0]
        cos_aoa_preds = torch.zeros(batch_size, n_aps, device=x.device)
        sin_aoa_preds = torch.zeros(batch_size, n_aps, device=x.device)
        confidence_preds = torch.zeros(batch_size, n_aps, device=x.device)

        for ap_index in range(n_aps):
            x_ap = x[:, ap_index, :, :].unsqueeze(1) # shape (batch_size, 1, height, width)

            # Forward pass through ResNet Encoder
            resnet_features_single_ap = self.resnet_encoder_list[ap_index](x_ap)

            # Decoders, output shape is (batch_size, 1)
            decoder_output: ModelOutput = self.decoder(resnet_features_single_ap)
            # decoder_output: ModelOutput = self.decoder_list[ap_index](resnet_features_single_ap)

            # store the results in original order
            cos_aoa_preds[:, ap_index] = decoder_output.cos_aoa.squeeze(1)
            sin_aoa_preds[:, ap_index] = decoder_output.sin_aoa.squeeze(1)
            confidence_preds[:, ap_index] = decoder_output.confidence.squeeze(1)

        # convert AoA from AP frame to map frame
        cos_aoa_preds_map_frame = cos_angle_sum(self.cos_ap_orientations,
                                                self.sin_ap_orientations,
                                                cos_aoa_preds,
                                                sin_aoa_preds)
        sin_aoa_preds_map_frame = sin_angle_sum(self.cos_ap_orientations,
                                                self.sin_ap_orientations,
                                                cos_aoa_preds,
                                                sin_aoa_preds)

        # compute AoA prediction
        aoa_preds = torch.atan(sin_aoa_preds / cos_aoa_preds)

        confidence_score = torch.ones_like(confidence_preds)

        # compute the intersection point
        location_preds = solve_ray_intersection_batch(self.ap_locations,
                                                      cos_aoa_preds_map_frame,
                                                      sin_aoa_preds_map_frame,
                                                      confidence_score)

        return ModelOutput(cos_aoa=cos_aoa_preds,
                           sin_aoa=sin_aoa_preds,
                           location=location_preds,
                           confidence=confidence_preds,
                           aoa=aoa_preds)

    def _common_step(self, batch):
        # Unpack batch
        features_2d, aoa_label, location_label = batch

        # Forward pass
        model_pred: ModelOutput = self.forward(features_2d)

        # construct
        gt_label = GTlabel(cos_aoa=torch.cos(aoa_label),
                           sin_aoa=torch.sin(aoa_label),
                           location=location_label,
                           aoa=aoa_label)

        # compute loss
        loss_all = compute_loss(model_pred, gt_label)

        return model_pred, gt_label, loss_all

    def training_step(self, batch, batch_idx):
        model_pred, gt_label, train_loss = self._common_step(batch)
        self.train_metrics.update(model_pred, gt_label)
        self.log('train_loss', train_loss.total_loss.item())
        self.log_dict({'train_cos_loss': train_loss.cos_loss.item(),
                       'train_sin_loss': train_loss.sin_loss.item(),
                       'train_location_loss': train_loss.location_loss.item()})

        # log learning rate
        lr = self.trainer.optimizers[0].param_groups[0]['lr']
        self.log('learning_rate', lr)

        return train_loss.total_loss

    def on_train_epoch_end(self):
        # compute metrics
        metrics_result = self.train_metrics.compute()

        # plot pred vs ground truth location
        if self.epoch_counter % self.display_viz_data_epoch_interval == 0:
            loc_plot = plot_location_pred_vs_gt(metrics_result[MetricNames.LOCATION_PREDS].detach().cpu().numpy(),
                                                metrics_result[MetricNames.LOCATION_TARGETS].detach().cpu().numpy(),
                                                ap_locations=self.ap_metadata.ap_locations,
                                                ap_orientations=self.ap_metadata.ap_orientations)
            self.logger.experiment.log_figure(figure_name='train_location_pred_vs_gt', figure=loc_plot)
        # reset metrics
        self.train_metrics.reset()

    def validation_step(self, batch, batch_idx):
        model_pred, gt_label, val_loss = self._common_step(batch)
        self.val_metrics.update(model_pred, gt_label)

        # log the validation loss
        self.log('val_loss', val_loss.total_loss.item())
        self.log_dict({'val_cos_loss': val_loss.total_loss.item(),
                       'val_sin_loss': val_loss.sin_loss.item(),
                       'val_location_loss': val_loss.location_loss.item()})

        # store visualization data, visualize the first sample of the batch
        if len(self.val_visualization_data) < self.val_visualization_data.maxlen:
            self.val_visualization_data.append(AoAVisualizationSample(aoa_tof_plots=batch[0][0].detach().cpu(),
                                                                      aoa_pred=torch.rad2deg(model_pred.aoa[0].detach().cpu()),
                                                                      aoa_gt=torch.rad2deg(gt_label.aoa[0].detach().cpu()),
                                                                      confidence=model_pred.confidence[0].detach().cpu()))
        return val_loss.total_loss

    def on_validation_epoch_end(self):
        # compute metrics
        metrics_result = self.val_metrics.compute()

        # log AoA error metrics
        for ap_index in range(self.ap_metadata.n_aps):
            self.log_dict({f'val_aoa_error_mean_ap{ap_index}': metrics_result[MetricNames.AOA_ERROR_MEAN][ap_index].item(),
                           f'val_aoa_error_std_ap{ap_index}': metrics_result[MetricNames.AOA_ERROR_STD][ap_index].item(),
                           f'val_aoa_error_mse_ap{ap_index}': metrics_result[MetricNames.AOA_ERROR_MSE][ap_index].item()})

        # log location error metrics
        for metrics_name, metrics_value in metrics_result.items():
            if metrics_name.startswith('location_error'):
                self.log(f"val_{metrics_name}", metrics_value.item())

        # log the visualization data very self.display_viz_data_epoch_interval epoch

        if self.epoch_counter % self.display_viz_data_epoch_interval == 0:
            # plot pred vs ground truth location
            loc_plot = plot_location_pred_vs_gt(metrics_result[MetricNames.LOCATION_PREDS].detach().cpu(),
                                                metrics_result[MetricNames.LOCATION_TARGETS].detach().cpu(),
                                                ap_locations=self.ap_metadata.ap_locations,
                                                ap_orientations=self.ap_metadata.ap_orientations)
            self.logger.experiment.log_figure(figure_name='val_location_pred_vs_gt', figure=loc_plot)

            # log AoA-TOF visualization
            sample_index = 0
            while self.val_visualization_data:
                sample_viz = self.val_visualization_data.popleft()
                aoa_plot = plot_aoa_visualization(sample_viz, title=f"AoA Visualization, sample index {sample_index}", degree=True)
                self.logger.experiment.log_figure(figure_name=f'val_aoa_viz_sample{sample_index}', figure=aoa_plot)
                sample_index += 1

            # log AoA error distribution for each AP
            aoa_error_dist_plot = plot_aoa_error_histograms(metrics_result[MetricNames.AOA_ERRORS_ALL].detach().cpu().numpy(),
                                                            metrics_result[MetricNames.AOA_ERROR_MEAN].detach().cpu().numpy(),
                                                            metrics_result[MetricNames.AOA_ERROR_STD].detach().cpu().numpy(),
                                                            plot_in_degrees=True)
            self.logger.experiment.log_figure(figure_name='val_aoa_error_distribution', figure=aoa_error_dist_plot)

            # plot AoA prediction vs ground truth
            aoa_gt_vs_pred_plot = plot_gt_vs_pred_aoa(metrics_result[MetricNames.AOA_PREDS].detach().cpu().numpy(),
                                                      metrics_result[MetricNames.AOA_TARGETS].detach().cpu().numpy(),
                                                      delta=10,
                                                      plot_in_degrees=True)
            self.logger.experiment.log_figure(figure_name='val_aoa_gt_vs_pred', figure=aoa_gt_vs_pred_plot)


        # reset metrics
        self.val_metrics.reset()

        # increment the validation epoch counter
        self.epoch_counter += 1

    def configure_optimizers(self):
        optimizer = torch.optim.Adam(self.parameters(), lr=self.lr)
        lr_scheduler = {
            "scheduler": torch.optim.lr_scheduler.OneCycleLR(
                optimizer,
                max_lr=self.lr,
                epochs=self.trainer.max_epochs,
                steps_per_epoch=len(self.trainer.datamodule.train_dataloader()),
            ),
            "interval": "step",  # or "epoch"
            "frequency": 1,  # Update the scheduler every step
        }
        return [optimizer], [lr_scheduler]


if __name__ == '__main__':
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model = TrigAOAResNetModel().to(device)
    sample_input = torch.randn(5, 4, 224, 224).to(device)
    model_output = model(sample_input)
    print(f"device is {model.device}")
    print(f"Model output cos aoa: {model_output.cos_aoa}")
    print(f"Model output sin aoa: {model_output.sin_aoa}")
    print(f"Model output location: {model_output.location.shape}")
    print(f"Model output confidence: {model_output.confidence.shape}")