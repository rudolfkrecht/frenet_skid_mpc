---
layout: default
---


## Abstract
<div style="text-align: justify;">
This paper presents a Frenet-frame model predictive motion controller for curvature-constrained skid-steer mobile robots. Skid-steer platforms are particularly sensitive to excessive curvature due to wheel slip and drivetrain stress, which makes curvature-aware motion control essential for reliable path following. The proposed controller operates in a path-local coordinate system and regulates lateral and heading errors with respect to a geometric reference path while explicitly accounting for curvature feasibility. To ensure a fair and consistent evaluation, a curvature-feasible path generation method is introduced, producing smooth reference paths and velocity profiles that respect yaw-rate limitations. The proposed Frenet-frame motion controller is compared against a curvature-clipped point-to-point baseline controller under identical constraints and reference paths. Experimental results on multiple types of trajectory demonstrate that the proposed approach achieves improved path tracking accuracy while reducing the average curvature demand and overall steering activity. The results indicate that Frenet-frame predictive motion control provides a robust and efficient solution for curvature-constrained path following on skid-steer mobile robots.
</div>

---

## Links

- 💻 **Code**: https://github.com/rudolfkrecht/frenet_skid_mpc
- 📊 **Dataset**: https://github.com/rudolfkrecht/frenet_skid_mpc/tree/main/datasets

---

## Method


<div style="display:flex; justify-content:center; align-items:center; gap:20px; flex-wrap:wrap; text-align:center;">
  <div style="width:45%; max-width:500px;">
    <img src="assets/path_Scurve.png" style="width:100%;"><br>
    <em>S-curve path</em>
  </div>
  <div style="width:45%; max-width:500px;">
    <img src="assets/path_boxy.png" style="width:100%;"><br>
    <em>Boxy path</em>
  </div>
</div>



---

## Results



<div style="display:flex; justify-content:center; gap:20px; flex-wrap:wrap;">
  <video autoplay loop muted playsinline style="width:45%;" src="assets/S_path.mp4"></video>
  <video autoplay loop muted playsinline style="width:45%;" src="assets/Boxy_path.mp4"></video>
</div>



<div style="display:flex; justify-content:center; align-items:center; gap:20px; flex-wrap:wrap; text-align:center;">
  <div style="width:45%;">
    <img src="assets/compare_xy_s_curve_path.png" style="width:100%;"><br>
    <em></em>
  </div>
  <div style="width:45%;">
    <img src="assets/compare_xy_boxy_path.png" style="width:100%;"><br>
    <em></em>
  </div>
</div>

<div style="display:flex; justify-content:center; gap:20px; flex-wrap:wrap; text-align:center;">
  <div style="width:45%;">
    <img src="assets/compare_cmd_vel_s_curve_path.png" style="width:100%;"><br>
    <em>S-curve path </em>
  </div>
  <div style="width:45%;">
    <img src="assets/compare_cmd_vel_boxy_path.png" style="width:100%;"><br>
    <em>Boxy path</em>
  </div>
</div>