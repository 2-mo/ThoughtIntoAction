// @ts-nocheck
const DEFAULT_CORE_LANDMARKS = [11, 12, 13, 14, 15, 16, 23, 24];

function finiteConfidence(landmark) {
  const values = [landmark?.visibility, landmark?.presence].filter(Number.isFinite);
  return values.length ? Math.min(...values) : 1;
}

function cloneLandmarks(landmarks) {
  return landmarks.map((landmark) => ({ ...landmark }));
}

class OneEuroScalarFilter {
  constructor({ minCutoff, beta, derivativeCutoff }) {
    this.minCutoff = minCutoff;
    this.beta = beta;
    this.derivativeCutoff = derivativeCutoff;
    this.reset();
  }

  reset() {
    this.lastTimestampMs = null;
    this.lastRaw = null;
    this.lastFiltered = null;
    this.lastDerivative = 0;
  }

  static alpha(cutoff, deltaSeconds) {
    const tau = 1 / (2 * Math.PI * cutoff);
    return 1 / (1 + tau / deltaSeconds);
  }

  filter(value, timestampMs) {
    if (!Number.isFinite(value)) return this.lastFiltered ?? 0;
    if (this.lastTimestampMs === null || timestampMs <= this.lastTimestampMs) {
      this.lastTimestampMs = timestampMs;
      this.lastRaw = value;
      this.lastFiltered = value;
      this.lastDerivative = 0;
      return value;
    }

    const deltaSeconds = Math.min(
      1,
      Math.max(1 / 120, (timestampMs - this.lastTimestampMs) / 1000),
    );
    const derivative = (value - this.lastRaw) / deltaSeconds;
    const derivativeAlpha = OneEuroScalarFilter.alpha(
      this.derivativeCutoff,
      deltaSeconds,
    );
    const filteredDerivative =
      derivativeAlpha * derivative + (1 - derivativeAlpha) * this.lastDerivative;
    const cutoff = this.minCutoff + this.beta * Math.abs(filteredDerivative);
    const valueAlpha = OneEuroScalarFilter.alpha(cutoff, deltaSeconds);
    const filtered =
      valueAlpha * value + (1 - valueAlpha) * this.lastFiltered;

    this.lastTimestampMs = timestampMs;
    this.lastRaw = value;
    this.lastFiltered = filtered;
    this.lastDerivative = filteredDerivative;
    return filtered;
  }
}

class LandmarkOneEuroFilter {
  constructor(options) {
    this.options = options;
    this.filters = [];
    this.lastLandmarks = null;
  }

  reset() {
    this.filters = [];
    this.lastLandmarks = null;
  }

  ensureFilters(count) {
    if (this.filters.length === count) return;
    this.filters = Array.from({ length: count }, () => ({
      x: new OneEuroScalarFilter(this.options),
      y: new OneEuroScalarFilter(this.options),
      z: new OneEuroScalarFilter(this.options),
    }));
    this.lastLandmarks = null;
  }

  update(landmarks, timestampMs, confidenceLandmarks = landmarks) {
    this.ensureFilters(landmarks.length);
    const filtered = landmarks.map((landmark, index) => {
      const confidence = finiteConfidence(confidenceLandmarks[index]);
      const previous = this.lastLandmarks?.[index];
      if (previous && confidence < this.options.landmarkConfidence) {
        return {
          ...previous,
          ...(Number.isFinite(landmark.visibility)
            ? { visibility: landmark.visibility }
            : {}),
          ...(Number.isFinite(landmark.presence)
            ? { presence: landmark.presence }
            : {}),
        };
      }
      const filters = this.filters[index];
      return {
        ...landmark,
        x: filters.x.filter(landmark.x, timestampMs),
        y: filters.y.filter(landmark.y, timestampMs),
        z: filters.z.filter(landmark.z, timestampMs),
      };
    });
    this.lastLandmarks = cloneLandmarks(filtered);
    return filtered;
  }
}

export class PoseTemporalStabilizer {
  constructor({
    minCutoff = 1.15,
    beta = 0.35,
    derivativeCutoff = 1,
    landmarkConfidence = 0.45,
    poseConfidence = 0.5,
    dropoutHoldMs = 250,
    coreLandmarks = DEFAULT_CORE_LANDMARKS,
  } = {}) {
    const filterOptions = {
      minCutoff,
      beta,
      derivativeCutoff,
      landmarkConfidence,
    };
    this.normalizedFilter = new LandmarkOneEuroFilter(filterOptions);
    this.worldFilter = new LandmarkOneEuroFilter(filterOptions);
    this.poseConfidence = poseConfidence;
    this.dropoutHoldMs = dropoutHoldMs;
    this.coreLandmarks = coreLandmarks;
    this.lastStable = null;
    this.lastDetectedAtMs = -Infinity;
  }

  reset() {
    this.normalizedFilter.reset();
    this.worldFilter.reset();
    this.lastStable = null;
    this.lastDetectedAtMs = -Infinity;
  }

  confidence(landmarks) {
    const samples = this.coreLandmarks
      .map((index) => landmarks[index])
      .filter(Boolean)
      .map(finiteConfidence);
    return samples.length
      ? samples.reduce((total, value) => total + value, 0) / samples.length
      : 0;
  }

  update(result, timestampMs) {
    const landmarks = result.landmarks?.[0];
    const worldLandmarks = result.worldLandmarks?.[0];
    const detected =
      landmarks?.length > 0 && this.confidence(landmarks) >= this.poseConfidence;

    if (!detected) {
      if (
        this.lastStable &&
        timestampMs - this.lastDetectedAtMs <= this.dropoutHoldMs
      ) {
        return {
          landmarks: [cloneLandmarks(this.lastStable.landmarks[0])],
          worldLandmarks: this.lastStable.worldLandmarks.length
            ? [cloneLandmarks(this.lastStable.worldLandmarks[0])]
            : [],
        };
      }
      this.reset();
      return { landmarks: [], worldLandmarks: [] };
    }

    const filteredLandmarks = this.normalizedFilter.update(
      landmarks,
      timestampMs,
    );
    const filteredWorld = worldLandmarks?.length
      ? this.worldFilter.update(worldLandmarks, timestampMs, landmarks)
      : null;
    const stable = {
      landmarks: [filteredLandmarks],
      worldLandmarks: filteredWorld ? [filteredWorld] : [],
    };
    this.lastStable = {
      landmarks: [cloneLandmarks(filteredLandmarks)],
      worldLandmarks: filteredWorld ? [cloneLandmarks(filteredWorld)] : [],
    };
    this.lastDetectedAtMs = timestampMs;
    return stable;
  }
}
