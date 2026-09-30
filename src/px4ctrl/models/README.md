# Offline priors

`utils/offline_GANs_Train.py` writes the frozen generative priors here:

| File | Axis | Interface |
| --- | --- | --- |
| `generator_prior_X.pt` | x | `float32[N,6] -> (f0[N,1], g0[N,1])` |
| `generator_prior_Y.pt` | y | same |
| `generator_prior_Z.pt` | z | same |
| `manifest.json` | all | training configuration, data splits and validation/test metrics |

The input is the inertial state `[p_x, p_y, p_z, v_x, v_y, v_z]` in SI units; the
state normalization is embedded in each TorchScript module. `run_ctrl.launch`
loads the three files from this directory for configurations A, B and D.
