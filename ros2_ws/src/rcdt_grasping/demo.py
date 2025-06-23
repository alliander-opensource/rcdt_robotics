import os
from pathlib import Path

import gdown
import numpy as np
import open3d as o3d
import scipy.io as scio
import torch
from graspnetAPI import GraspGroup
from graspnetpy_models.graspnet import GraspNet, pred_decode
from graspnetpy_utils.collision_detector import ModelFreeCollisionDetector
from graspnetpy_utils.data_utils import CameraInfo, create_point_cloud_from_depth_image
from PIL import Image

CHECKPOINT_DIR = "/home/rcdt/rcdt_robotics/.cache/graspnet/"
CHECKPOINT_FILE = "checkpoint-rs.tar"
CHECKPOINT = CHECKPOINT_DIR + CHECKPOINT_FILE
DRIVE_LINK = "https://drive.google.com/uc?id=1hd0G8LN6tRpi4742XOTEisbTXNZ-1jmk"

NUM_POINT = 20000
NUM_VIEW = 300
COLLISION_TRESH = 0.01
VOXEL_SIZE = 0.01


def get_net():
    # Init the model
    net = GraspNet(
        input_feature_dim=0,
        num_view=NUM_VIEW,
        num_angle=12,
        num_depth=4,
        cylinder_radius=0.05,
        hmin=-0.02,
        hmax_list=[0.01, 0.02, 0.03, 0.04],
        is_training=False,
    )
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    net.to(device)
    # Load checkpoint
    if not os.path.isfile(CHECKPOINT):
        Path(CHECKPOINT_DIR).mkdir(parents=True, exist_ok=True)
        gdown.download(url=DRIVE_LINK, output=CHECKPOINT)
    checkpoint = torch.load(CHECKPOINT)
    net.load_state_dict(checkpoint["model_state_dict"])
    # set model to eval mode
    net.eval()
    return net


def get_and_process_data(data_dir):
    # load data
    color = (
        np.array(Image.open(os.path.join(data_dir, "color.png")), dtype=np.float32)
        / 255.0
    )
    depth = np.array(Image.open(os.path.join(data_dir, "depth.png")))
    # workspace_mask = np.array(Image.open(os.path.join(data_dir, "workspace_mask.png")))
    # meta = scio.loadmat(os.path.join(data_dir, "meta.mat"))
    # intrinsic = meta["intrinsic_matrix"]
    # factor_depth = meta["factor_depth"]

    # generate cloud
    camera = CameraInfo(
        320.0,
        240.0,
        232.8014373779297,
        232.80142307281494,
        160.0,
        120.0,
        50,
    )
    cloud = create_point_cloud_from_depth_image(depth, camera, organized=True)

    # get valid points
    # mask = workspace_mask & (depth > 0)
    mask = depth > 0
    cloud_masked = cloud[mask]
    color_masked = color[mask]

    # sample points
    if len(cloud_masked) >= NUM_POINT:
        idxs = np.random.choice(len(cloud_masked), NUM_POINT, replace=False)
    else:
        idxs1 = np.arange(len(cloud_masked))
        idxs2 = np.random.choice(
            len(cloud_masked), NUM_POINT - len(cloud_masked), replace=True
        )
        idxs = np.concatenate([idxs1, idxs2], axis=0)
    cloud_sampled = cloud_masked[idxs]
    color_sampled = color_masked[idxs]

    # convert data
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(cloud_masked.astype(np.float32))
    cloud.colors = o3d.utility.Vector3dVector(color_masked.astype(np.float32))
    end_points = dict()
    cloud_sampled = torch.from_numpy(cloud_sampled[np.newaxis].astype(np.float32))
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    cloud_sampled = cloud_sampled.to(device)
    end_points["point_clouds"] = cloud_sampled
    end_points["cloud_colors"] = color_sampled

    return end_points, cloud


def get_grasps(net, end_points):
    # Forward pass
    with torch.no_grad():
        end_points = net(end_points)
        grasp_preds = pred_decode(end_points)
    gg_array = grasp_preds[0].detach().cpu().numpy()
    gg = GraspGroup(gg_array)
    return gg


def collision_detection(gg, cloud):
    mfcdetector = ModelFreeCollisionDetector(cloud, voxel_size=VOXEL_SIZE)
    collision_mask = mfcdetector.detect(
        gg, approach_dist=0.05, collision_thresh=COLLISION_TRESH
    )
    gg = gg[~collision_mask]
    return gg


def vis_grasps(gg, cloud):
    gg.nms()
    gg.sort_by_score()
    gg = gg[:10]
    grippers = gg.to_open3d_geometry_list()
    o3d.visualization.draw_geometries([cloud, *grippers])


def demo(data_dir):
    net = get_net()
    end_points, cloud = get_and_process_data(data_dir)
    gg = get_grasps(net, end_points)
    if COLLISION_TRESH > 0:
        gg = collision_detection(gg, np.array(cloud.points))
    vis_grasps(gg, cloud)


if __name__ == "__main__":
    # data_dir = "/home/rcdt/graspnet-baseline/doc/example_data"
    data_dir = "/home/rcdt/rcdt_robotics/ros2_ws/src/rcdt_grasping"
    demo(data_dir)
