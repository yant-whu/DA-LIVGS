# Reproduction notes

This pre-acceptance snapshot documents the full reproduction workflow while releasing only selected implementation components. The complete source tree will be added after paper acceptance.

The complete paper pipeline uses two ROS nodes. The front end publishes synchronized sensor packets, pose feedback, and degradation status. The mapping node consumes those messages and writes the Gaussian map and held-out rendering metrics.

## Source provenance

- Front end snapshot `7d819d8`
- Mapping snapshot `d628789`
- Livox driver snapshot `638e47994bdf9bbe64888eb6cbd6d04167bacc0c`
- Profile values `benchmarks/source_provenance.json`
- Paper metrics `benchmarks/final_profile_results.json`

Absolute experiment paths are removed from the compact manifest. Input data and generated maps are not included.

Direct mapping launches use `/tmp/da_livgs_outputs` unless `DA_LIVGS_OUTPUT_ROOT` is set. The wrapper scripts accept explicit output paths, and `run_odometry.sh` accepts `--start` and `--duration` for short smoke tests; omit them for a complete sequence.

## SPNet engines

The mapping node loads `spnet_480_640.engine` or `spnet_512_640.engine` from `catkin_ws/src/da_livgs_mapping/ckpt/`. These generated files are excluded from the repository. Download `Large_300.pth` from the upstream model distribution, place it in that directory, and run `setup_spnet.sh`, `export_onnx.sh`, and `build_trt.sh` there before the first mapping run.

## Dataset mapping

| Paper group | Front-end config | Mapping config |
| --- | --- | --- |
| R3LIVE Deg.00/01/02 | `da_livgs_r3live.yaml` | `da_livgs_r3live_balanced.yaml` |
| FAST-LIVO Visual Challenge / LiDAR Degenerate | `da_livgs_fastlivo.yaml` | `da_livgs_fastlivo_balanced.yaml` |
| FAST-LIVO2 Bright / CBD / Retail | `da_livgs_fastlivo2.yaml` | `da_livgs_fastlivo2_balanced.yaml` |
| GEODE Flat / Offroad / Tunneling | `da_livgs_geode_gamma.yaml` | `da_livgs_geode_gamma_balanced.yaml` |
| Campus Field / Slope / Trees | `da_livgs_airy.yaml` or right-camera variant | `da_livgs_airy_balanced.yaml` |

Use the GEODE B1 variant when the dataset recording requires its alternate calibration.

## Checks

1. Build from a clean shell with ROS Noetic sourced.
2. Run a short bag segment before a full sequence. Confirm the mapping end-of-stream marker and `per_view_metrics.csv`.
3. Confirm `sh_degree=1`, `rank=16`, `feature_dimension=16`, `embedding_dimension=16`, `residual_scale=0.48`, `knot_interval_frames=10`, and `random_seed=12345`.
4. Run `scripts/validate_run.py`.
5. Compare with `benchmarks/final_profile_results.json` only after matching the bag, calibration, frame split, seed, and machine settings.

The paper used fixed same-sequence train/test frame selection and matched trajectory conditions for rendering comparisons. Published baselines and external trajectories are not re-run by this repository.
