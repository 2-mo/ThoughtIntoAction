import * as THREE from 'three';

// Component ranges belong to the bundled X5/link6.STL, in STL triangle order.
// Keep the original surfaces: each moving assembly includes its finger, slider,
// rack and attached fasteners. The housing, guide and end stops stay stationary.
const SOURCE_SHA256 = '1105bf55921cffce9bd189342ad2a59946484ef20aad02db0c0f5224d5892698';
const POSITIVE = [[0, 542], [5070, 5926], [40958, 41582], [47950, 48198]];
const NEGATIVE = [[542, 1166], [5926, 6782], [47206, 47454], [48198, 48446],
  [48694, 48942], [50118, 50666]];
// Distance between the two original inner fingertip surfaces, in mesh metres.
const REFERENCE_WIDTH = 2 * 0.030142907053232193;

export async function splitGripperGeometry(source, geometry, scale) {
  const digest = [...new Uint8Array(await crypto.subtle.digest('SHA-256', source))]
    .map(value => value.toString(16).padStart(2, '0')).join('');
  // A different URDF/mesh must never inherit this model's component assignment.
  if (digest !== SOURCE_SHA256 || !Number.isFinite(scale[1]) || scale[1] === 0) return null;
  const groups = [[], [], []];
  for (let face = 0; face < geometry.attributes.position.count / 3; face++) {
    const side = POSITIVE.some(([start, end]) => face >= start && face < end) ? 1
      : NEGATIVE.some(([start, end]) => face >= start && face < end) ? 2 : 0;
    groups[side].push(face);
  }
  const parts = groups.map(faces => {
    const part = new THREE.BufferGeometry();
    for (const [name, attribute] of Object.entries(geometry.attributes)) {
      const size = attribute.itemSize * 3;
      const values = new attribute.array.constructor(faces.length * size);
      faces.forEach((face, index) => values.set(attribute.array.subarray(face * size, (face + 1) * size), index * size));
      part.setAttribute(name, new THREE.BufferAttribute(values, attribute.itemSize, attribute.normalized));
    }
    part.computeBoundingBox();
    part.computeBoundingSphere();
    return part;
  });
  return {body: parts[0], positive: parts[1], negative: parts[2],
    referenceWidth: REFERENCE_WIDTH * Math.abs(scale[1]), direction: Math.sign(scale[1])};
}

export function attachGripper(mesh, parts) {
  const fingers = [parts.positive, parts.negative].map((geometry, index) => {
    const finger = new THREE.Mesh(geometry, mesh.material);
    finger.name = index === 0 ? 'gripper-positive' : 'gripper-negative';
    finger.castShadow = mesh.castShadow;
    finger.receiveShadow = mesh.receiveShadow;
    mesh.add(finger);
    return finger;
  });
  return {
    setOpening(width, maximum) {
      const valid = Number.isFinite(width) && Number.isFinite(maximum) && maximum > 0;
      const opening = valid ? Math.min(maximum, Math.max(0, width)) : parts.referenceWidth;
      const offset = parts.direction * (opening - parts.referenceWidth) / 2;
      fingers[0].position.y = offset;
      fingers[1].position.y = -offset;
      // Missing feedback uses the reference geometry, never a fabricated zero.
      mesh.userData.gripperOpening = valid ? opening : null;
    },
  };
}
