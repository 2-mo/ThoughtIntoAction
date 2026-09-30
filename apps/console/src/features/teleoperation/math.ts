// @ts-nocheck
const EPSILON = 1e-9;

function vector(a, b) {
  return [b.x - a.x, b.y - a.y, b.z - a.z];
}

function dot(a, b) {
  return a.reduce((sum, value, index) => sum + value * b[index], 0);
}

function cross(a, b) {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}

function normalize(value) {
  const norm = Math.hypot(...value);
  if (!Number.isFinite(norm) || norm < EPSILON) {
    throw new RangeError('hand basis is degenerate');
  }
  return value.map((component) => component / norm);
}

function distance(a, b) {
  return Math.hypot(b.x - a.x, b.y - a.y, b.z - a.z);
}

function jointAngleDegrees(a, b, c) {
  const first = normalize(vector(b, a));
  const second = normalize(vector(b, c));
  const cosine = Math.max(-1, Math.min(1, dot(first, second)));
  return Math.acos(cosine) * 180 / Math.PI;
}

export function isOpenPalm(hand, minimumDegrees = 155) {
  if (!Array.isArray(hand) || hand.length !== 21) return false;

  try {
    return [[5, 6, 7], [9, 10, 11], [13, 14, 15], [17, 18, 19]]
      .every(([mcp, pip, dip]) => jointAngleDegrees(hand[mcp], hand[pip], hand[dip]) >= minimumDegrees);
  } catch (error) {
    if (error instanceof RangeError) return false;
    throw error;
  }
}

export function isOpenHandGesture(
  hand,
  minimumDegrees = 145,
  minimumExtendedFingers = 3,
) {
  if (!Array.isArray(hand) || hand.length !== 21) return false;
  let extended = 0;
  for (const [mcp, pip, dip] of [[5, 6, 7], [9, 10, 11], [13, 14, 15], [17, 18, 19]]) {
    try {
      if (jointAngleDegrees(hand[mcp], hand[pip], hand[dip]) >= minimumDegrees) {
        extended += 1;
      }
    } catch (error) {
      if (!(error instanceof RangeError)) throw error;
    }
  }
  return extended >= minimumExtendedFingers;
}

export function isClosedFist(hand, maximumDegrees = 110) {
  if (!Array.isArray(hand) || hand.length !== 21) return false;

  try {
    return [[5, 6, 7], [9, 10, 11], [13, 14, 15], [17, 18, 19]]
      .every(([mcp, pip, dip]) => jointAngleDegrees(hand[mcp], hand[pip], hand[dip]) <= maximumDegrees);
  } catch (error) {
    if (error instanceof RangeError) return false;
    throw error;
  }
}

export function palmScale(hand) {
  const values = [
    distance(hand[0], hand[5]),
    distance(hand[0], hand[17]),
    distance(hand[5], hand[17]),
    distance(hand[0], hand[9]),
  ].sort((a, b) => a - b);
  return (values[1] + values[2]) / 2;
}

export function pinchRatio(hand) {
  const denominator = distance(hand[5], hand[17]);
  return denominator <= EPSILON ? Number.NaN : distance(hand[4], hand[8]) / denominator;
}

export function relativeOperatorTranslation(origin, current) {
  const values = [origin?.x, origin?.y, origin?.scale, current?.x, current?.y, current?.scale];
  if (!values.every(Number.isFinite) || origin.scale <= 0 || current.scale <= 0) {
    throw new TypeError('hand translation input is invalid');
  }

  return [
    0.25 * Math.log(origin.scale / current.scale),
    0.40 * (current.x - origin.x),
    0.40 * (origin.y - current.y),
  ];
}

function matrixQuaternion(matrix) {
  const [m00, m01, m02] = matrix[0];
  const [m10, m11, m12] = matrix[1];
  const [m20, m21, m22] = matrix[2];
  const trace = m00 + m11 + m22;
  let quaternion;

  if (trace > 0) {
    const s = 2 * Math.sqrt(trace + 1);
    quaternion = [(m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s];
  } else if (m00 > m11 && m00 > m22) {
    const s = 2 * Math.sqrt(1 + m00 - m11 - m22);
    quaternion = [0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s];
  } else if (m11 > m22) {
    const s = 2 * Math.sqrt(1 + m11 - m00 - m22);
    quaternion = [(m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s];
  } else {
    const s = 2 * Math.sqrt(1 + m22 - m00 - m11);
    quaternion = [(m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s];
  }

  return normalize(quaternion);
}

export function palmFrameQuaternion(hand, handedness) {
  const across = normalize(vector(hand[17], hand[5]));
  const forwardRaw = normalize(vector(hand[0], hand[9]));
  const projection = dot(forwardRaw, across);
  const forward = normalize(forwardRaw.map((value, index) => value - projection * across[index]));
  let normal = normalize(cross(across, forward));
  if (String(handedness).toLowerCase() === 'left') normal = normal.map((value) => -value);
  const canonicalForward = normalize(cross(normal, across));

  return matrixQuaternion([
    [across[0], canonicalForward[0], normal[0]],
    [across[1], canonicalForward[1], normal[1]],
    [across[2], canonicalForward[2], normal[2]],
  ]);
}

function midpoint(first, second) {
  return {
    x: (first.x + second.x) / 2,
    y: (first.y + second.y) / 2,
    z: (first.z + second.z) / 2,
  };
}

export function tigerMouthFrameQuaternion(hand, handedness) {
  if (!Array.isArray(hand) || hand.length !== 21) {
    throw new TypeError('tiger-mouth frame requires 21 landmarks');
  }
  const scale = palmScale(hand);
  const openingDistance = distance(hand[4], hand[8]);
  if (!Number.isFinite(scale) || scale <= EPSILON
      || openingDistance / scale < 0.15) {
    throw new RangeError('tiger-mouth opening axis is degenerate');
  }
  const opening = normalize(vector(hand[4], hand[8]));
  const root = midpoint(hand[2], hand[5]);
  const tips = midpoint(hand[4], hand[8]);
  const longitudinalRaw = normalize(vector(root, tips));
  const projection = dot(longitudinalRaw, opening);
  const longitudinal = normalize(longitudinalRaw.map(
    (value, index) => value - projection * opening[index],
  ));
  let normal = normalize(cross(opening, longitudinal));
  if (String(handedness).toLowerCase() === 'left') {
    normal = normal.map((value) => -value);
  }
  const canonicalLongitudinal = normalize(cross(normal, opening));
  return matrixQuaternion([
    [opening[0], canonicalLongitudinal[0], normal[0]],
    [opening[1], canonicalLongitudinal[1], normal[1]],
    [opening[2], canonicalLongitudinal[2], normal[2]],
  ]);
}

export function closestQuaternion(candidate, reference) {
  const similarity = candidate.reduce((sum, value, index) => sum + value * reference[index], 0);
  return similarity < 0 ? candidate.map((value) => -value) : [...candidate];
}

export class TigerMouthOrientationTracker {
  constructor({ holdSeconds = 0.1 } = {}) {
    this.holdSeconds = holdSeconds;
    this.reset();
  }

  reset() {
    this.quaternion = null;
    this.lastValidTimestamp = null;
    this.timestamp = null;
  }

  update(hand, handedness, timestamp) {
    if (!Number.isFinite(timestamp)) {
      throw new TypeError('orientation timestamp must be finite');
    }
    if (this.timestamp !== null && timestamp <= this.timestamp) {
      throw new RangeError('orientation timestamp must increase');
    }
    this.timestamp = timestamp;
    try {
      const candidate = tigerMouthFrameQuaternion(hand, handedness);
      this.quaternion = this.quaternion
        ? closestQuaternion(candidate, this.quaternion)
        : candidate;
      this.lastValidTimestamp = timestamp;
      return this.quaternion.slice();
    } catch (error) {
      if (!(error instanceof RangeError)) throw error;
      if (this.quaternion === null) {
        this.quaternion = palmFrameQuaternion(hand, handedness);
        this.lastValidTimestamp = timestamp;
        return this.quaternion.slice();
      }
      if (timestamp - this.lastValidTimestamp <= this.holdSeconds) {
        return this.quaternion.slice();
      }
      throw new RangeError('tiger-mouth frame remained degenerate');
    }
  }
}

export class ClutchLatch {
  constructor(engageFrames = 5, releaseFrames = 2) {
    this.engageFrames = engageFrames;
    this.releaseFrames = releaseFrames;
    this.reset();
  }

  reset() {
    this.engaged = false;
    this.positive = 0;
    this.negative = 0;
  }

  update(open, reset = false) {
    const engageRequested = open && !reset;
    const resetRequested = reset && !open;
    this.positive = engageRequested ? this.positive + 1 : 0;
    this.negative = resetRequested ? this.negative + 1 : 0;
    if (!this.engaged && this.positive >= this.engageFrames) this.engaged = true;
    if (this.engaged && this.negative >= this.releaseFrames) this.engaged = false;
    return this.engaged;
  }
}

export class PinchLatch {
  constructor(closeThreshold = 0.32, openThreshold = 0.48, frames = 3) {
    this.closeThreshold = closeThreshold;
    this.openThreshold = openThreshold;
    this.frames = frames;
    this.state = 'open';
    this.count = 0;
  }

  reset(state = 'open') {
    this.state = state;
    this.count = 0;
  }

  update(ratio) {
    const requested = ratio < this.closeThreshold
      ? 'close'
      : ratio > this.openThreshold ? 'open' : this.state;
    this.count = requested === this.state ? 0 : this.count + 1;
    if (requested !== this.state && this.count >= this.frames) {
      this.state = requested;
      this.count = 0;
    }
    return this.state;
  }
}

export class HandOpenCloseLatch {
  constructor(frames = 3) {
    this.frames = frames;
    this.reset();
  }

  reset(state = 'open') {
    this.state = state;
    this.requested = state;
    this.count = 0;
  }

  update(open, closed) {
    const requested = binaryHandState(open, closed, this.state);
    if (requested !== this.requested) {
      this.requested = requested;
      this.count = 0;
    }
    this.count = requested === this.state ? 0 : this.count + 1;
    if (requested !== this.state && this.count >= this.frames) {
      this.state = requested;
      this.count = 0;
    }
    return this.state;
  }
}

export function binaryHandState(open, closed, fallback = 'open') {
  if (typeof open !== 'boolean' || typeof closed !== 'boolean') {
    throw new TypeError('hand posture flags must be boolean');
  }
  if (!['open', 'close'].includes(fallback)) {
    throw new RangeError('fallback hand state must be open or close');
  }
  return open === closed ? fallback : closed ? 'close' : 'open';
}

function normalizedQuaternion(value) {
  if (!Array.isArray(value) || value.length !== 4 || !value.every(Number.isFinite)) {
    throw new TypeError('quaternion must contain four finite values');
  }
  return normalize(value);
}

export function quaternionMultiply(first, second) {
  const [ax, ay, az, aw] = normalizedQuaternion(first);
  const [bx, by, bz, bw] = normalizedQuaternion(second);
  return normalizedQuaternion([
    aw * bx + ax * bw + ay * bz - az * by,
    aw * by - ax * bz + ay * bw + az * bx,
    aw * bz + ax * by - ay * bx + az * bw,
    aw * bw - ax * bx - ay * by - az * bz,
  ]);
}

export function quaternionInverse(value) {
  const [x, y, z, w] = normalizedQuaternion(value);
  return [-x, -y, -z, w];
}

export function quaternionSlerp(first, second, fraction) {
  const start = normalizedQuaternion(first);
  let target = normalizedQuaternion(second);
  let cosine = dot(start, target);
  if (cosine < 0) {
    target = target.map((value) => -value);
    cosine = -cosine;
  }
  cosine = Math.max(-1, Math.min(1, cosine));
  if (cosine > 0.9995) {
    return normalizedQuaternion(start.map(
      (value, index) => value + fraction * (target[index] - value),
    ));
  }
  const angle = Math.acos(cosine);
  const sine = Math.sin(angle);
  return normalizedQuaternion(start.map((value, index) => (
    Math.sin((1 - fraction) * angle) / sine * value
    + Math.sin(fraction * angle) / sine * target[index]
  )));
}

export function quaternionDistanceRadians(first, second) {
  const start = normalizedQuaternion(first);
  const target = normalizedQuaternion(second);
  return 2 * Math.acos(Math.max(-1, Math.min(1, Math.abs(dot(start, target)))));
}

function clampVectorStep(previous, desired, maximumStep) {
  const step = desired.map((value, index) => value - previous[index]);
  const distance = Math.hypot(...step);
  const scale = distance > maximumStep ? maximumStep / distance : 1;
  return previous.map((value, index) => value + scale * step[index]);
}

function quaternionMatrix(value) {
  const [x, y, z, w] = normalizedQuaternion(value);
  return [
    [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
  ];
}

function multiplyMatrices(first, second) {
  return first.map((row) => second[0].map((_, column) => (
    row.reduce((sum, value, index) => sum + value * second[index][column], 0)
  )));
}

export function transformQuaternionFrame(value, frame) {
  if (!Array.isArray(frame) || frame.length !== 3
      || !frame.every((row) => Array.isArray(row) && row.length === 3)) {
    throw new TypeError('quaternion frame must be a 3x3 matrix');
  }
  const transpose = frame[0].map((_, column) => frame.map((row) => row[column]));
  return matrixQuaternion(multiplyMatrices(
    multiplyMatrices(frame, quaternionMatrix(value)),
    transpose,
  ));
}

class OneEuroScalarFilter {
  constructor(minCutoff = 1.5, beta = 0.20, derivativeCutoff = 1.0) {
    this.minCutoff = minCutoff;
    this.beta = beta;
    this.derivativeCutoff = derivativeCutoff;
    this.value = null;
    this.rawValue = null;
    this.derivative = 0;
    this.timestamp = null;
  }

  alpha(cutoff, deltaSeconds) {
    const tau = 1 / (2 * Math.PI * cutoff);
    return 1 / (1 + tau / deltaSeconds);
  }

  update(value, timestamp) {
    if (this.timestamp === null) {
      this.value = value;
      this.rawValue = value;
      this.timestamp = timestamp;
      return value;
    }
    const deltaSeconds = timestamp - this.timestamp;
    if (!(deltaSeconds > 0)) throw new RangeError('filter timestamp must increase');
    const rawDerivative = (value - this.rawValue) / deltaSeconds;
    const derivativeAlpha = this.alpha(this.derivativeCutoff, deltaSeconds);
    this.derivative += derivativeAlpha * (rawDerivative - this.derivative);
    const cutoff = this.minCutoff + this.beta * Math.abs(this.derivative);
    const valueAlpha = this.alpha(cutoff, deltaSeconds);
    this.value += valueAlpha * (value - this.value);
    this.rawValue = value;
    this.timestamp = timestamp;
    return this.value;
  }
}

export class OneEuroVectorFilter {
  constructor() {
    this.filters = [0, 1, 2].map(() => new OneEuroScalarFilter());
  }

  update(value, timestamp) {
    if (!Array.isArray(value) || value.length !== 3 || !value.every(Number.isFinite)) {
      throw new TypeError('filter value must contain three finite values');
    }
    if (!Number.isFinite(timestamp)) {
      throw new TypeError('filter timestamp must be finite');
    }
    return this.filters.map((filter, index) => filter.update(value[index], timestamp));
  }
}

export class TeleoperationMapper {
  constructor({
    operatorToBase,
    translationGain,
    rotationGain,
    maxLinearVelocityMps = 0.03,
    maxAngularVelocityRadS = 0.35,
    translationDeadzoneM = 0.006,
    translationFullScaleM = 0.06,
    rotationDeadzoneRad = 4 * Math.PI / 180,
    rotationFullScaleRad = 30 * Math.PI / 180,
    maxTranslationStepM = 0.005,
    maxRotationStepRad = 3 * Math.PI / 180,
    rotationTimeConstantS = 0.08,
  }) {
    this.operatorToBase = operatorToBase;
    this.translationGain = translationGain;
    this.rotationGain = rotationGain;
    this.maxLinearVelocityMps = maxLinearVelocityMps;
    this.maxAngularVelocityRadS = maxAngularVelocityRadS;
    this.translationDeadzoneM = translationDeadzoneM;
    this.translationFullScaleM = translationFullScaleM;
    this.rotationDeadzoneRad = rotationDeadzoneRad;
    this.rotationFullScaleRad = rotationFullScaleRad;
    this.maxTranslationStepM = maxTranslationStepM;
    this.maxRotationStepRad = maxRotationStepRad;
    this.rotationTimeConstantS = rotationTimeConstantS;
    this.translationFilter = new OneEuroVectorFilter();
    this.origin = null;
    this.previousTranslation = [0, 0, 0];
    this.previousQuaternion = [0, 0, 0, 1];
    this.filteredControlQuaternion = [0, 0, 0, 1];
    this.previousTimestamp = null;
  }

  engage(handPose, timestamp) {
    this.origin = structuredClone(handPose);
    this.translationFilter = new OneEuroVectorFilter();
    this.translationFilter.update([0, 0, 0], timestamp);
    this.previousTranslation = [0, 0, 0];
    this.previousQuaternion = [0, 0, 0, 1];
    this.filteredControlQuaternion = [0, 0, 0, 1];
    this.previousTimestamp = timestamp;
  }

  update(handPose, timestamp) {
    if (!this.origin) throw new Error('teleoperation mapper is not engaged');
    const elapsed = timestamp - this.previousTimestamp;
    if (!(elapsed > 0)) throw new RangeError('teleoperation timestamp must increase');
    const integrationPeriod = Math.min(elapsed, 0.1);
    const operator = relativeOperatorTranslation(
      { x: this.origin.wrist.x, y: this.origin.wrist.y, scale: this.origin.scale },
      { x: handPose.wrist.x, y: handPose.wrist.y, scale: handPose.scale },
    );
    const base = this.operatorToBase.map(
      (row) => this.translationGain * row.reduce(
        (sum, value, index) => sum + value * operator[index],
        0,
      ),
    );
    const rawDistance = Math.hypot(...base);
    let linearVelocity = [0, 0, 0];
    if (rawDistance <= this.translationDeadzoneM) {
      this.translationFilter = new OneEuroVectorFilter();
      this.translationFilter.update([0, 0, 0], timestamp);
    } else {
      const filtered = this.translationFilter.update(base, timestamp);
      const distance = Math.hypot(...filtered);
      if (distance > this.translationDeadzoneM) {
        const magnitude = Math.min(
          1,
          (distance - this.translationDeadzoneM)
            / (this.translationFullScaleM - this.translationDeadzoneM),
        );
        linearVelocity = filtered.map(
          (value) => this.maxLinearVelocityMps * magnitude * value / distance,
        );
      }
    }
    const desiredTranslation = this.previousTranslation.map(
      (value, index) => value + linearVelocity[index] * integrationPeriod,
    );
    this.previousTranslation = clampVectorStep(
      this.previousTranslation,
      desiredTranslation,
      this.maxTranslationStepM,
    );
    const operatorRotation = quaternionMultiply(
      handPose.quaternion,
      quaternionInverse(this.origin.quaternion),
    );
    const rawRotation = closestQuaternion(
      transformQuaternionFrame(operatorRotation, this.operatorToBase),
      [0, 0, 0, 1],
    );
    const controlRotation = closestQuaternion(quaternionSlerp(
      [0, 0, 0, 1],
      rawRotation,
      this.rotationGain,
    ), [0, 0, 0, 1]);
    const rawAngle = quaternionDistanceRadians([0, 0, 0, 1], controlRotation);
    if (rawAngle <= this.rotationDeadzoneRad) {
      this.filteredControlQuaternion = [0, 0, 0, 1];
    } else {
      this.filteredControlQuaternion = closestQuaternion(quaternionSlerp(
        this.filteredControlQuaternion,
        controlRotation,
        1 - Math.exp(-elapsed / this.rotationTimeConstantS),
      ), [0, 0, 0, 1]);
      const filteredAngle = quaternionDistanceRadians(
        [0, 0, 0, 1],
        this.filteredControlQuaternion,
      );
      if (filteredAngle > this.rotationDeadzoneRad) {
        const magnitude = Math.min(
          1,
          (filteredAngle - this.rotationDeadzoneRad)
            / (this.rotationFullScaleRad - this.rotationDeadzoneRad),
        );
        const stepAngle = Math.min(
          this.maxAngularVelocityRadS * magnitude * integrationPeriod,
          this.maxRotationStepRad,
        );
        const sine = Math.sin(filteredAngle / 2);
        if (stepAngle > 0 && Math.abs(sine) > EPSILON) {
          const axis = this.filteredControlQuaternion.slice(0, 3).map(
            (value) => value / sine,
          );
          const increment = [
            ...axis.map((value) => value * Math.sin(stepAngle / 2)),
            Math.cos(stepAngle / 2),
          ];
          this.previousQuaternion = closestQuaternion(
            quaternionMultiply(increment, this.previousQuaternion),
            this.previousQuaternion,
          );
        }
      }
    }
    this.previousTimestamp = timestamp;
    return {
      delta_xyz_m: this.previousTranslation.slice(),
      delta_orientation_xyzw: this.previousQuaternion.slice(),
    };
  }
}

export function selectControlAndClutchHands(result, controlSide) {
  const hands = result.landmarks.map((landmarks, index) => ({
    landmarks,
    worldLandmarks: result.worldLandmarks[index],
    side: swapRawMediaPipeHandSide(
      result.handedness[index]?.[0]?.categoryName,
    ),
    score: Number(result.handedness[index]?.[0]?.score ?? 0),
  }));
  const normalizedSide = String(controlSide).toLowerCase();
  const clutchSide = normalizedSide === 'left' ? 'right' : 'left';
  const control = hands.find(
    (hand) => hand.side === normalizedSide && hand.score >= 0.70,
  );
  const clutch = hands.find(
    (hand) => hand.side === clutchSide && hand.score >= 0.70,
  );
  return control && clutch ? { control, clutch } : null;
}

export function selectLeftAndRightHands(result) {
  const hands = result.landmarks.map((landmarks, index) => ({
    landmarks,
    worldLandmarks: result.worldLandmarks[index],
    side: swapRawMediaPipeHandSide(
      result.handedness[index]?.[0]?.categoryName,
    ),
    score: Number(result.handedness[index]?.[0]?.score ?? 0),
  }));
  const left = hands.find((hand) => hand.side === 'left' && hand.score >= 0.70);
  const right = hands.find((hand) => hand.side === 'right' && hand.score >= 0.70);
  return left && right ? { left, right } : null;
}

function swapRawMediaPipeHandSide(side) {
  const normalized = String(side ?? '').toLowerCase();
  if (normalized === 'left') return 'right';
  if (normalized === 'right') return 'left';
  return normalized;
}

export function isLateTeleoperationStart(message, cancelledSequences) {
  return message?.action === 'start'
    && cancelledSequences instanceof Set
    && cancelledSequences.has(message.seq);
}

export function handPoseFromLandmarks(hand, orientationTracker = null, timestamp = null) {
  const quaternion = orientationTracker
    ? orientationTracker.update(hand.worldLandmarks, hand.side, timestamp)
    : palmFrameQuaternion(hand.worldLandmarks, hand.side);
  return {
    wrist: { x: 1 - hand.landmarks[0].x, y: hand.landmarks[0].y },
    scale: palmScale(hand.landmarks),
    quaternion,
  };
}
