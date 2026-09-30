// @ts-nocheck
const EPSILON = 1e-9;

function finiteMatrix4(value) {
  return Array.isArray(value) && value.length === 4
    && value.every((row) => Array.isArray(row) && row.length === 4
      && row.every((component) => Number.isFinite(Number(component))));
}

function fiveCoefficients(value) {
  const result = [0, 0, 0, 0, 0];
  if (Array.isArray(value)) {
    value.slice(0, 5).forEach((component, index) => {
      if (Number.isFinite(Number(component))) result[index] = Number(component);
    });
  }
  return result;
}

function inverseBrownToUndistorted(xd, yd, coefficients) {
  const [k1, k2, p1, p2, k3] = fiveCoefficients(coefficients);
  let x = xd;
  let y = yd;
  for (let index = 0; index < 10; index += 1) {
    const radiusSquared = x * x + y * y;
    const inverseRadial = 1 / (
      1 + ((k3 * radiusSquared + k2) * radiusSquared + k1) * radiusSquared
    );
    const xq = x / inverseRadial;
    const yq = y / inverseRadial;
    const deltaX = 2 * p1 * xq * yq + p2 * (radiusSquared + 2 * xq * xq);
    const deltaY = 2 * p2 * xq * yq + p1 * (radiusSquared + 2 * yq * yq);
    x = (xd - deltaX) * inverseRadial;
    y = (yd - deltaY) * inverseRadial;
  }
  return [x, y];
}

function forwardBrown(x, y, coefficients) {
  const [k1, k2, p1, p2, k3] = fiveCoefficients(coefficients);
  const radiusSquared = x * x + y * y;
  const radial = 1 + k1 * radiusSquared + k2 * radiusSquared ** 2
    + k3 * radiusSquared ** 3;
  return [
    x * radial + 2 * p1 * x * y + p2 * (radiusSquared + 2 * x * x),
    y * radial + 2 * p2 * x * y + p1 * (radiusSquared + 2 * y * y),
  ];
}

function brownToUndistorted(xd, yd, coefficients) {
  const [k1, k2, p1, p2, k3] = fiveCoefficients(coefficients);
  let x = xd;
  let y = yd;
  for (let index = 0; index < 10; index += 1) {
    const radiusSquared = x * x + y * y;
    const radial = 1 + k1 * radiusSquared + k2 * radiusSquared ** 2
      + k3 * radiusSquared ** 3;
    const deltaX = 2 * p1 * x * y + p2 * (radiusSquared + 2 * x * x);
    const deltaY = 2 * p2 * x * y + p1 * (radiusSquared + 2 * y * y);
    x = (xd - deltaX) / Math.max(radial, EPSILON);
    y = (yd - deltaY) / Math.max(radial, EPSILON);
  }
  return [x, y];
}

export function pixelToCameraRay(pixel, intrinsics) {
  if (!Array.isArray(pixel) || pixel.length !== 2) throw new Error("像素坐标无效");
  const fx = Number(intrinsics?.fx);
  const fy = Number(intrinsics?.fy);
  const ppx = Number(intrinsics?.ppx);
  const ppy = Number(intrinsics?.ppy);
  if (![fx, fy, ppx, ppy].every(Number.isFinite) || fx <= 0 || fy <= 0) {
    throw new Error("D455 RGB 内参无效");
  }
  const xd = (Number(pixel[0]) - ppx) / fx;
  const yd = (Number(pixel[1]) - ppy) / fy;
  const model = String(intrinsics.model ?? "none").toLowerCase().replaceAll(" ", "_");
  let normalized;
  if (model.endsWith("inverse_brown_conrady")) {
    normalized = inverseBrownToUndistorted(xd, yd, intrinsics.coeffs);
  } else if (model.endsWith("brown_conrady")) {
    normalized = brownToUndistorted(xd, yd, intrinsics.coeffs);
  } else {
    normalized = [xd, yd];
  }
  return [normalized[0], normalized[1], 1];
}

function cameraPointToPixel(point, intrinsics) {
  if (!Array.isArray(point) || point.length !== 3 || point[2] <= 0.01) {
    throw new Error("桌面角点位于相机后方");
  }
  const fx = Number(intrinsics?.fx);
  const fy = Number(intrinsics?.fy);
  const ppx = Number(intrinsics?.ppx);
  const ppy = Number(intrinsics?.ppy);
  if (![fx, fy, ppx, ppy].every(Number.isFinite) || fx <= 0 || fy <= 0) {
    throw new Error("D455 RGB 内参无效");
  }
  const normalized = [point[0] / point[2], point[1] / point[2]];
  const model = String(intrinsics.model ?? "none").toLowerCase().replaceAll(" ", "_");
  let distorted = normalized;
  if (model.endsWith("inverse_brown_conrady")) {
    distorted = normalized.slice();
    for (let index = 0; index < 12; index += 1) {
      const estimate = inverseBrownToUndistorted(
        distorted[0],
        distorted[1],
        intrinsics.coeffs,
      );
      distorted[0] -= estimate[0] - normalized[0];
      distorted[1] -= estimate[1] - normalized[1];
    }
  } else if (model.endsWith("brown_conrady")) {
    distorted = forwardBrown(normalized[0], normalized[1], intrinsics.coeffs);
  }
  return [distorted[0] * fx + ppx, distorted[1] * fy + ppy];
}

function basePointToPixel(point, transform, intrinsics) {
  return cameraPointToPixel(basePointToCamera(point, transform), intrinsics);
}

function basePointToCamera(point, transform) {
  const delta = [0, 1, 2].map((row) => point[row] - Number(transform[row][3]));
  return [0, 1, 2].map((column) => (
    Number(transform[0][column]) * delta[0]
    + Number(transform[1][column]) * delta[1]
    + Number(transform[2][column]) * delta[2]
  ));
}

export function projectTablePolygon(table, TBaseCamera, intrinsics) {
  if (!finiteMatrix4(TBaseCamera)) throw new Error("固定相机标定尚未启用");
  const position = table?.position_mm?.map((value) => Number(value) / 1000);
  const size = table?.size_mm?.map((value) => Number(value) / 1000);
  const yaw = Number(table?.yaw_deg) * Math.PI / 180;
  if (!Array.isArray(position) || position.length !== 3 || !position.every(Number.isFinite)
      || !Array.isArray(size) || size.length !== 2 || !size.every((value) => Number.isFinite(value) && value > 0)
      || !Number.isFinite(yaw)) {
    throw new Error("桌面标定参数无效");
  }
  const cosine = Math.cos(yaw);
  const sine = Math.sin(yaw);
  const localCorners = [
    [-size[0] / 2, -size[1] / 2],
    [size[0] / 2, -size[1] / 2],
    [size[0] / 2, size[1] / 2],
    [-size[0] / 2, size[1] / 2],
  ];
  const visibleCorners = localCorners.flatMap(([localX, localY]) => {
    const base = [
      position[0] + cosine * localX - sine * localY,
      position[1] + sine * localX + cosine * localY,
      position[2],
    ];
    const cameraPoint = basePointToCamera(base, TBaseCamera);
    if (cameraPoint[2] <= 0.01) return [];
    const pixel = cameraPointToPixel(cameraPoint, intrinsics);
    return pixel.every(Number.isFinite) ? [pixel] : [];
  });
  if (visibleCorners.length === 4) return visibleCorners;
  if (visibleCorners.length < 2) {
    throw new Error("桌面可见区域不足，无法生成 ROI");
  }

  // A wide tabletop can extend behind the camera. Use its two lowest visible
  // projected corners as the front edge and close the polygon along the image
  // bottom, instead of rejecting the entire ROI and searching the background.
  const frontEdge = visibleCorners
    .slice()
    .sort((first, second) => second[1] - first[1])
    .slice(0, 2)
    .sort((first, second) => first[0] - second[0]);
  const width = Number(intrinsics?.width) || 1280;
  const height = Number(intrinsics?.height) || 720;
  return [frontEdge[0], frontEdge[1], [width, height], [0, height]];
}

function clipCameraSegmentToNear(first, second, near = 0.02) {
  const firstVisible = first[2] >= near;
  const secondVisible = second[2] >= near;
  if (!firstVisible && !secondVisible) return null;
  if (firstVisible && secondVisible) return [first, second];
  const amount = (near - first[2]) / (second[2] - first[2]);
  const intersection = first.map((value, index) => (
    value + (second[index] - value) * amount
  ));
  return firstVisible ? [first, intersection] : [intersection, second];
}

function clipCameraPolygonToNear(points, near = 0.02) {
  const clipped = [];
  points.forEach((current, index) => {
    const previous = points[(index + points.length - 1) % points.length];
    const currentVisible = current[2] >= near;
    const previousVisible = previous[2] >= near;
    if (currentVisible !== previousVisible) {
      const amount = (near - previous[2]) / (current[2] - previous[2]);
      clipped.push(previous.map((value, component) => (
        value + (current[component] - value) * amount
      )));
    }
    if (currentVisible) clipped.push(current);
  });
  return clipped;
}

export function projectTableCalibrationEdges(table, TBaseCamera, intrinsics) {
  if (!finiteMatrix4(TBaseCamera)) throw new Error("固定相机标定尚未启用");
  const position = table?.position_mm?.map((value) => Number(value) / 1000);
  const size = table?.size_mm?.map((value) => Number(value) / 1000);
  const yaw = Number(table?.yaw_deg) * Math.PI / 180;
  if (!Array.isArray(position) || position.length !== 3 || !position.every(Number.isFinite)
      || !Array.isArray(size) || size.length !== 2
      || !size.every((value) => Number.isFinite(value) && value > 0)
      || !Number.isFinite(yaw)) {
    throw new Error("桌面标定参数无效");
  }
  const cosine = Math.cos(yaw);
  const sine = Math.sin(yaw);
  const cameraCorners = [
    [-size[0] / 2, -size[1] / 2],
    [size[0] / 2, -size[1] / 2],
    [size[0] / 2, size[1] / 2],
    [-size[0] / 2, size[1] / 2],
  ].map(([localX, localY]) => basePointToCamera([
    position[0] + cosine * localX - sine * localY,
    position[1] + sine * localX + cosine * localY,
    position[2],
  ], TBaseCamera));
  const projectSegment = (firstIndex, secondIndex) => {
    const clipped = clipCameraSegmentToNear(
      cameraCorners[firstIndex],
      cameraCorners[secondIndex],
    );
    return clipped?.map((point) => cameraPointToPixel(point, intrinsics)) ?? null;
  };
  const polygon = clipCameraPolygonToNear(cameraCorners)
    .map((point) => cameraPointToPixel(point, intrinsics));
  return {
    polygon,
    // Local +Y is the far edge; local +X is the right edge.
    top: projectSegment(3, 2),
    side: projectSegment(2, 1),
  };
}

function transformDirection(ray, transform) {
  return [0, 1, 2].map((row) => (
    Number(transform[row][0]) * ray[0]
    + Number(transform[row][1]) * ray[1]
    + Number(transform[row][2]) * ray[2]
  ));
}

function transformPoint(point, transform) {
  return [0, 1, 2].map((row) => (
    Number(transform[row][0]) * point[0]
    + Number(transform[row][1]) * point[1]
    + Number(transform[row][2]) * point[2]
    + Number(transform[row][3])
  ));
}

function depthObservation(sample, fallbackIntrinsics, transform, expectedTopZ) {
  if (sample?.ok !== true) return null;
  const depthM = Number(sample.depth_m);
  const validCount = Number(sample.valid_count);
  if (!Number.isFinite(depthM) || depthM <= 0.08 || depthM > 4
      || !Number.isFinite(validCount) || validCount < 3) return null;
  const pixel = [Number(sample.x), Number(sample.y)];
  if (!pixel.every(Number.isFinite)) return null;
  const ray = pixelToCameraRay(pixel, sample.intrinsics ?? fallbackIntrinsics);
  const camera = ray.map((value) => value * depthM);
  const base = transformPoint(camera, transform);
  const residualZ = base[2] - expectedTopZ;
  return {
    depth_m: depthM,
    valid_count: validCount,
    camera,
    base,
    residual_z_m: residualZ,
    // A tabletop sample is about 30 mm below the expected cube top.  Reject
    // it, as well as depth spikes on the black surface, before fusion.
    valid: Math.abs(residualZ) <= 0.02,
  };
}

function intersectPixelWithPlane(pixel, intrinsics, transform, planeZ) {
  const rayCamera = pixelToCameraRay(pixel, intrinsics);
  const rayBase = transformDirection(rayCamera, transform);
  const origin = [0, 1, 2].map((row) => Number(transform[row][3]));
  if (Math.abs(rayBase[2]) < 1e-6) throw new Error("相机射线与桌面近平行");
  const scale = (planeZ - origin[2]) / rayBase[2];
  if (!Number.isFinite(scale) || scale <= 0) throw new Error("桌面位于相机视线后方");
  return {
    base: origin.map((value, index) => value + rayBase[index] * scale),
    camera: rayCamera.map((value) => value * scale),
  };
}

export function projectImagePolygonToBasePlane(
  polygon,
  intrinsics,
  TBaseCamera,
  planeZ,
) {
  if (!finiteMatrix4(TBaseCamera)) throw new Error("固定相机标定尚未启用");
  if (!Array.isArray(polygon) || polygon.length < 3) throw new Error("工作区多边形无效");
  const z = Number(planeZ);
  if (!Number.isFinite(z)) throw new Error("桌面高度无效");
  return polygon.map((pixel) => intersectPixelWithPlane(
    pixel,
    intrinsics,
    TBaseCamera,
    z,
  ).base);
}

function limitedVector(vector, maximumLength) {
  const length = Math.hypot(vector[0], vector[1]);
  if (!Number.isFinite(length) || length < 1e-6) return [0, 0];
  const scale = Math.min(1, maximumLength / length);
  return [vector[0] * scale, vector[1] * scale];
}

function correctedTopCenterPixel(detection, intrinsics, transform, tableTopZ, cubeSizeM) {
  const rawPixel = detection.center_px.map(Number);
  const topZ = Number(tableTopZ) + cubeSizeM;
  const top = intersectPixelWithPlane(rawPixel, intrinsics, transform, topZ).base;
  const contactPixel = basePointToPixel(
    [top[0], top[1], Number(tableTopZ)],
    transform,
    intrinsics,
  );
  const sideProjection = [
    contactPixel[0] - rawPixel[0],
    contactPixel[1] - rawPixel[1],
  ];
  const projectionLength = Math.hypot(sideProjection[0], sideProjection[1]);
  if (!Number.isFinite(projectionLength) || projectionLength < 3) {
    return { pixel: rawPixel, correction_px: [0, 0], applied: false };
  }
  const apparentSize = Number(detection.size_px ?? 30);
  const maximumCorrection = Math.max(4, Math.min(36, apparentSize * 0.48));
  const correction = limitedVector(
    [sideProjection[0] * 0.58, sideProjection[1] * 0.58],
    maximumCorrection,
  );
  return {
    pixel: [rawPixel[0] - correction[0], rawPixel[1] - correction[1]],
    correction_px: [-correction[0], -correction[1]],
    applied: true,
  };
}

function distance2d(first, second) {
  return Math.hypot(first[0] - second[0], first[1] - second[1]);
}

function normalizeYaw(value) {
  return Math.atan2(Math.sin(value), Math.cos(value));
}

export class CubePoseFilter {
  constructor() {
    this.position = null;
    this.yaw = null;
    this.depthSamples = [];
    this.depthAnchor = null;
  }

  reset() {
    this.position = null;
    this.yaw = null;
    this.depthSamples = [];
    this.depthAnchor = null;
  }

  updateDepthAverage(pose, rgbPosition, status) {
    if (status === "occluded") return null;
    const depthPosition = pose?.depth_top_base_xyz_m?.map(Number);
    if (!pose?.depth_valid || !Array.isArray(depthPosition)
        || depthPosition.length !== 3 || !depthPosition.every(Number.isFinite)) {
      return null;
    }
    const depthDelta = Math.hypot(
      depthPosition[0] - rgbPosition[0],
      depthPosition[1] - rgbPosition[1],
    );
    if (depthDelta > 0.04) return null;
    if (this.depthAnchor !== null
        && Math.hypot(
          rgbPosition[0] - this.depthAnchor[0],
          rgbPosition[1] - this.depthAnchor[1],
        ) > 0.025) {
      this.depthSamples = [];
    }
    this.depthAnchor = rgbPosition.slice(0, 2);
    this.depthSamples.push(depthPosition.slice(0, 2));
    if (this.depthSamples.length > 15) this.depthSamples.shift();
    if (this.depthSamples.length < 5) return null;
    const values = this.depthSamples;
    const median = [0, 1].map((index) => {
      const sorted = values.map((sample) => sample[index]).sort((a, b) => a - b);
      return sorted[Math.floor(sorted.length / 2)];
    });
    const distances = values.map((sample) => Math.hypot(
      sample[0] - median[0],
      sample[1] - median[1],
    ));
    const inlierLimit = Math.max(
      0.004,
      2.5 * (distances.slice().sort((a, b) => a - b)[Math.floor(distances.length / 2)] || 0),
    );
    const inliers = values.filter((_sample, index) => distances[index] <= inlierLimit);
    if (inliers.length < 3) return null;
    return [
      inliers.reduce((sum, sample) => sum + sample[0], 0) / inliers.length,
      inliers.reduce((sum, sample) => sum + sample[1], 0) / inliers.length,
    ];
  }

  update(pose, status = "tracking") {
    const rawPosition = pose?.base_xyz_m?.map(Number);
    const rawYaw = Number(pose?.yaw_rad);
    if (!Array.isArray(rawPosition) || rawPosition.length !== 3
        || !rawPosition.every(Number.isFinite) || !Number.isFinite(rawYaw)) {
      throw new Error("方块基座位姿无效");
    }
    const rgbPosition = rawPosition.slice();
    const depthAverage = this.updateDepthAverage(pose, rgbPosition, status);
    const depthAverageUsed = depthAverage !== null;
    if (depthAverageUsed) {
      rawPosition[0] = rgbPosition[0] * 0.4 + depthAverage[0] * 0.6;
      rawPosition[1] = rgbPosition[1] * 0.4 + depthAverage[1] * 0.6;
    }
    if (this.position === null || this.yaw === null) {
      this.position = rawPosition.slice();
      this.yaw = rawYaw;
    } else if (status !== "occluded") {
      const innovation = Math.hypot(
        rawPosition[0] - this.position[0],
        rawPosition[1] - this.position[1],
      );
      let alpha;
      if (innovation <= 0.001) alpha = 0;
      else if (innovation <= 0.004) alpha = 0.1;
      else if (innovation <= 0.015) alpha = 0.36;
      else alpha = 0.72;
      this.position[0] += (rawPosition[0] - this.position[0]) * alpha;
      this.position[1] += (rawPosition[1] - this.position[1]) * alpha;
      this.position[2] = rawPosition[2];
      const yawDelta = normalizeYaw(rawYaw - this.yaw);
      this.yaw = normalizeYaw(this.yaw + yawDelta * Math.min(alpha, 0.2));
    }
    return {
      ...pose,
      raw_base_xyz_m: rgbPosition,
      base_xyz_m: this.position.slice(),
      yaw_rad: this.yaw,
      yaw_deg: this.yaw * 180 / Math.PI,
      filter: "adaptive_low_pass",
      depth_average_samples: this.depthSamples.length,
      position_source: depthAverageUsed
        ? "rgb_plane+depth_average"
        : (pose.position_source ?? "rgb_plane"),
    };
  }
}

export function estimateCubePose(detection, TBaseCamera, tableTopZM) {
  if (!finiteMatrix4(TBaseCamera)) throw new Error("固定相机标定尚未启用");
  if (!Array.isArray(detection?.center_px) || !Array.isArray(detection?.axis_px)) {
    throw new Error("方块像素检测无效");
  }
  const cubeSizeM = Number(detection.cube_size_m) || 0.03;
  const planeZ = Number(tableTopZM) + cubeSizeM;
  if (!Number.isFinite(planeZ)) throw new Error("桌面高度无效");
  const intrinsics = detection.intrinsics;
  const correctedCenter = correctedTopCenterPixel(
    detection,
    intrinsics,
    TBaseCamera,
    Number(tableTopZM),
    cubeSizeM,
  );
  const rawCenter = intersectPixelWithPlane(
    detection.center_px,
    intrinsics,
    TBaseCamera,
    planeZ,
  );
  const center = intersectPixelWithPlane(
    correctedCenter.pixel,
    intrinsics,
    TBaseCamera,
    planeZ,
  );
  const depth = depthObservation(
    detection.depth_sample,
    intrinsics,
    TBaseCamera,
    planeZ,
  );
  const axisLength = Math.max(8, Number(detection.size_px ?? 20) * 0.4);
  const axis = detection.axis_px.map(Number);
  const beforePixel = [
    correctedCenter.pixel[0] - axis[0] * axisLength,
    correctedCenter.pixel[1] - axis[1] * axisLength,
  ];
  const afterPixel = [
    correctedCenter.pixel[0] + axis[0] * axisLength,
    correctedCenter.pixel[1] + axis[1] * axisLength,
  ];
  const before = intersectPixelWithPlane(beforePixel, intrinsics, TBaseCamera, planeZ).base;
  const after = intersectPixelWithPlane(afterPixel, intrinsics, TBaseCamera, planeZ).base;
  const yaw = normalizeYaw(Math.atan2(after[1] - before[1], after[0] - before[0]));

  // The object size is a fixed experiment constant. Silhouette corners can
  // mix the cube's top and contact edges under perspective, so projecting all
  // four of them onto one top plane produces an inflated (for example 65 mm)
  // estimate and incorrectly rejects the real 30 mm cube.
  let sizeEstimateM = Number(detection.cube_size_m);
  if (!Number.isFinite(sizeEstimateM) || sizeEstimateM <= 0) sizeEstimateM = 0.03;
  let projectedSizeM = null;
  if (Array.isArray(detection.corners_px) && detection.corners_px.length === 4) {
    const corners = detection.corners_px.map((pixel) => (
      intersectPixelWithPlane(pixel, intrinsics, TBaseCamera, planeZ).base
    ));
    projectedSizeM = corners.reduce((sum, corner, index) => (
      sum + distance2d(corner, corners[(index + 1) % corners.length])
    ), 0) / corners.length;
  }
  // The D455 stereo depth is unstable on this small, black, close-range
  // surface. Use the calibrated RGB ray/table-plane intersection for control;
  // retain depth only as a diagnostic residual.
  const positionBase = center.base;
  const positionCamera = center.camera;
  return {
    base_xyz_m: [positionBase[0], positionBase[1], Number(tableTopZM) + 0.015],
    camera_xyz_m: positionCamera,
    plane_base_xyz_m: center.base,
    raw_plane_base_xyz_m: rawCenter.base,
    raw_center_px: detection.center_px,
    corrected_center_px: correctedCenter.pixel,
    center_correction_px: correctedCenter.correction_px,
    center_correction_m: distance2d(center.base, rawCenter.base),
    silhouette_center_corrected: correctedCenter.applied,
    yaw_rad: yaw,
    yaw_deg: yaw * 180 / Math.PI,
    size_estimate_m: sizeEstimateM,
    projected_size_m: projectedSizeM,
    plane_z_m: planeZ,
    position_source: "rgb_plane",
    depth_valid: depth?.valid === true,
    depth_m: depth?.depth_m ?? null,
    depth_valid_count: depth?.valid_count ?? 0,
    depth_top_base_xyz_m: depth?.base ?? null,
    depth_residual_z_m: depth?.residual_z_m ?? null,
    depth_xy_delta_m: depth
      ? distance2d(depth.base, center.base)
      : null,
  };
}
