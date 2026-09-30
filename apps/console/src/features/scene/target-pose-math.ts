// @ts-nocheck
import * as THREE from "three";

export const TARGET_EDIT_STATE = Object.freeze({
  IDLE: "idle",
  EDIT_POSITION: "edit_position",
  TARGET_READY: "target_ready",
  EDIT_ORIENTATION: "edit_orientation",
});

export const LOCAL_APPROACH_AXIS = Object.freeze([1, 0, 0]);

export function normalizeQuaternion(quaternion) {
  const result = quaternion.clone();
  if (!Number.isFinite(result.lengthSq()) || result.lengthSq() < 1e-12) {
    return new THREE.Quaternion();
  }
  return result.normalize();
}

export function quaternionFromArray(value) {
  if (!Array.isArray(value) || value.length !== 4 || !value.every(Number.isFinite)) {
    return new THREE.Quaternion();
  }
  return normalizeQuaternion(new THREE.Quaternion(...value));
}

export function quaternionToArray(quaternion) {
  const normalized = normalizeQuaternion(quaternion);
  return [normalized.x, normalized.y, normalized.z, normalized.w];
}

export function quaternionFromRpy(rpy) {
  if (!Array.isArray(rpy) || rpy.length !== 3 || !rpy.every(Number.isFinite)) {
    return new THREE.Quaternion();
  }
  return new THREE.Quaternion()
    .setFromEuler(new THREE.Euler(rpy[0], rpy[1], rpy[2], "ZYX"))
    .normalize();
}

export function quaternionToRpy(quaternion) {
  const euler = new THREE.Euler().setFromQuaternion(
    normalizeQuaternion(quaternion),
    "ZYX",
  );
  return [euler.x, euler.y, euler.z];
}

export function closestQuaternion(candidate, reference) {
  const result = normalizeQuaternion(candidate);
  if (reference && result.dot(reference) < 0.0) {
    result.set(-result.x, -result.y, -result.z, -result.w);
  }
  return result;
}

function perpendicularAxis(direction, preferredUp) {
  const projected = preferredUp.clone().addScaledVector(
    direction,
    -preferredUp.dot(direction),
  );
  if (projected.lengthSq() > 1e-8) return projected.normalize();
  const fallback = Math.abs(direction.z) < 0.8
    ? new THREE.Vector3(0, 0, 1)
    : new THREE.Vector3(0, 1, 0);
  return fallback.addScaledVector(direction, -fallback.dot(direction)).normalize();
}

export function alignApproachPreserveRoll(
  previousQuaternion,
  targetDirection,
  localApproachAxis = new THREE.Vector3(...LOCAL_APPROACH_AXIS),
  preferredUp = new THREE.Vector3(0, 0, 1),
) {
  const previous = normalizeQuaternion(previousQuaternion);
  const from = localApproachAxis.clone().applyQuaternion(previous).normalize();
  const to = targetDirection.clone().normalize();
  if (to.lengthSq() < 1e-10) return previous;
  const dot = THREE.MathUtils.clamp(from.dot(to), -1, 1);
  let delta;
  if (dot < -0.9999) {
    delta = new THREE.Quaternion().setFromAxisAngle(
      perpendicularAxis(from, preferredUp),
      Math.PI,
    );
  } else {
    delta = new THREE.Quaternion().setFromUnitVectors(from, to);
  }
  return closestQuaternion(delta.multiply(previous), previous);
}

export function applyLocalRoll(
  quaternion,
  deltaRadians,
  localApproachAxis = new THREE.Vector3(...LOCAL_APPROACH_AXIS),
) {
  const previous = normalizeQuaternion(quaternion);
  const roll = new THREE.Quaternion().setFromAxisAngle(
    localApproachAxis.clone().normalize(),
    deltaRadians,
  );
  return closestQuaternion(previous.clone().multiply(roll), previous);
}

export function cloneTargetPose(pose) {
  if (!pose) return null;
  return {
    position: pose.position.map(Number),
    orientation: quaternionToArray(quaternionFromArray(pose.orientation)),
    ...(Number.isFinite(pose.gripperWidth)
      ? { gripperWidth: Number(pose.gripperWidth) }
      : {}),
  };
}
