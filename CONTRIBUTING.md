# Contributing to PROJECT DRIFT

Thank you for contributing! Please read this guide before opening a pull
request or pushing any code.

---

## Branching Strategy

### Protected Branches

| Branch | Description | Who can push |
|---|---|---|
| `main` | Stable, demo-ready code. Tagged releases only. | Team Lead (Phillipp) via PR |
| `dev` | Integration branch. All feature PRs target `dev`. | Merge via PR, reviewed by ≥ 1 teammate |

### Feature Branches

Create a new branch off `dev` for every piece of work. Use the naming
convention below — keep names lowercase and hyphen-separated.

| Branch Name | Owner / Purpose |
|---|---|
| `feature/node1-camera` | Panchtio — ArduCAM SPI frame capture |
| `feature/kalman-filter` | Phillipp — EKF position & orientation fusion |
| `feature/node2-5-imu` | Sam — IMU-only node firmware |
| `feature/wifi-transport` | Sam — WiFi UDP/TCP sensor data transport |
| `feature/tinyml` | Vedant — Edge Impulse pipeline & TFLite Micro |
| `feature/sensor-fusion` | Vedant — IMU + camera fusion logic |
| `feature/evaluation` | Jack — drift analysis scripts & plots |
| `feature/dashboard` | Jack — real-time visualization dashboard |
| `hotfix/<description>` | Critical bug fixes against `main` |

### Typical Workflow

```bash
# 1. Always branch from the latest dev
git checkout dev
git pull origin dev
git checkout -b feature/your-feature-name

# 2. Commit often with clear messages
git add <files>
git commit -m "feat(node1): add ArduCAM frame buffer flush on overrun"

# 3. Push and open a PR against dev
git push origin feature/your-feature-name
# → open PR on GitHub targeting dev
```

---

## Commit Message Format

Follow [Conventional Commits](https://www.conventionalcommits.org/):

```
<type>(<scope>): <short description>

[optional body]
[optional footer]
```

**Types:** `feat`, `fix`, `docs`, `style`, `refactor`, `test`, `chore`

**Scopes:** `node1`, `node2`, `node3-5`, `kalman`, `tinyml`, `scripts`,
`docs`, `ci`

**Examples:**

```
feat(kalman): implement predict step with gyro bias correction
fix(node1): resolve SPI clock conflict between ArduCAM and MPU-6050
docs(wiring): add I2C pull-up resistor notes to node1 pin map
chore(pio): pin ArduCAM library to v1.0.6
```

---

## Pull Request Checklist

Before requesting review, confirm:

- [ ] Branch is up to date with `dev` (`git rebase dev` or merge)
- [ ] Code compiles without warnings (`pio run`)
- [ ] Serial output is tested on real hardware (or emulator if unavailable)
- [ ] New public functions have Doxygen comment blocks
- [ ] Any new data files are added to `30_data/` (not committed to `10_src/`)
- [ ] Large binary files (models, datasheets) use Git LFS
- [ ] PR description explains **what** changed and **why**

---

## Code Style

### C++ (Firmware)

- Arduino/PlatformIO style: 2-space indentation, `camelCase` for variables,
  `PascalCase` for classes/structs, `UPPER_SNAKE_CASE` for macros/constants.
- Keep `.ino`-style logic out of `src/` — use proper `.cpp`/`.h` split.
- Document all public functions with Doxygen:

```cpp
/**
 * @brief Read accelerometer and gyroscope data from MPU-6050.
 *
 * @param[out] ax  Acceleration X in m/s²
 * @param[out] ay  Acceleration Y in m/s²
 * @param[out] az  Acceleration Z in m/s²
 * @param[out] gx  Angular rate X in rad/s
 * @param[out] gy  Angular rate Y in rad/s
 * @param[out] gz  Angular rate Z in rad/s
 * @return true on success, false if I²C read failed
 */
bool readIMU(float &ax, float &ay, float &az,
             float &gx, float &gy, float &gz);
```

### Python (Scripts)

- Follow [PEP 8](https://pep8.org/) — use `black` for auto-formatting.
- Type hints required for all function signatures.
- Place reusable helpers in `60_scripts/utils/`.

---

## Git LFS

Large binary assets must use Git LFS:

```bash
git lfs track "*.tflite"
git lfs track "*.pdf"
git lfs track "*.png"
git lfs track "*.csv"
git add .gitattributes
```

---

## Questions?

Open a GitHub Issue or ping the team on the course Slack channel.
