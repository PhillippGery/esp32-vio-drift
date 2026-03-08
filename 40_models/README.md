# 40_models — TinyML Models

Week 5 stretch goal: motion classifier trained on Edge Impulse and deployed
as TFLite Micro on Node 1.

| Folder | Contents |
|---|---|
| `tflite_micro/` | Quantized INT8 `.tflite` files + generated C arrays (`model.h`) |
| `training_logs/` | Edge Impulse training logs, confusion matrices, model cards |

## Deployment Workflow

1. Collect labeled data → `30_data/labeled_datasets/`
2. Upload to Edge Impulse → train DSP + NN pipeline
3. Export as **Arduino library** (ZIP) → extract to `40_models/tflite_micro/`
4. Include `model.h` in `10_src/node1_vio/src/tinyml_inference.cpp`
5. Flash with `pio run --target upload`

## Model Card Template

Each model should be accompanied by a `model_card_vN.md` in `training_logs/`:
- Training dataset size and split
- DSP features used (e.g., spectral analysis)
- NN architecture
- INT8 quantization accuracy vs. float baseline
- Inference latency on ESP32 (ms)
