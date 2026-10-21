# ARX5 X5 Offline Model Asset

`X5.urdf` is a content-preserving copy of `models/X5.urdf` from
[`real-stanford/arx5-sdk`](https://github.com/real-stanford/arx5-sdk) at commit
`8ccbf0aa8d4288f2721c41a735e5ac4c389bb13b`.

The only normalization is removal of an upstream trailing whitespace character.
The upstream project is MIT licensed, copyright 2024 Yihuai Gao. Its license
applies to this copied asset; see the repository-level [LICENSE](../../../../third_party/phone_input/LICENSE)
for this project's license and `docs/references.md` for attribution details.

This asset is for offline FK, Jacobian, and IK feasibility checks only. It does
not identify the installed physical arm, authorize its joint limits, load an
ARX5 SDK, or enable CAN/hardware motion.
