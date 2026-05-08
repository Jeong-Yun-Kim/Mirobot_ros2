Optional calibration files go here.

Default package mode is dynamic pixel-to-robot mapping, so no file is required.

If you want the older static homography mode, place:

  static_board_pose.npz

in this folder. The file must include:

  H_inv
  T_base_board

Then set in config/order_delivery.yaml:

  calibration.mode: "static"
  calibration.use_static: true
