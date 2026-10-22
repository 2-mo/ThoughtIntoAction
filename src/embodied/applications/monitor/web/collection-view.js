// Pure display projection: unknown counts stay unknown and every named camera is kept.
export function collectionMetrics(data) {
  const recording = data.recording;
  const counts = recording?.counts || data.episode_metadata?.counts || {};
  const cameraCounts = recording?.camera_counts || data.episode_metadata?.camera_counts || {};
  const cameras = Object.entries(cameraCounts);
  const count = value => Number.isInteger(value) && value >= 0 ? String(value) : '—';
  const drops = recording?.dropped;
  const dropped = drops && Object.values(drops).every(v => Number.isInteger(v) && v >= 0)
    ? Object.values(drops).reduce((a, b) => a + b, 0) : null;
  return {
    frames: cameras.length ? cameras.map(([, value]) => count(value)).join(' / ') : count(counts.frame),
    frameTitle: cameras.length ? cameras.map(([id, value]) => `${id}: ${count(value)}`).join(' · ') : '相机计数未按名称提供',
    samples: count(counts.state),
    progress: recording ? `写盘队列 ${count(recording.queue_size)} / ${count(recording.queue_capacity)} · 丢弃 ${count(dropped)}${recording.live ? '' : ' · 最后计数，非实时'}` : '',
  };
}

export function expireRecording(data, elapsedS) {
  const recording = data.recording;
  if (!recording?.live || !Number.isFinite(recording.heartbeat_age_s) ||
      recording.heartbeat_age_s + elapsedS < recording.heartbeat_ttl_s) return data;
  return {...data, collection: {...data.collection, state: 'STALE'},
    recording: {...recording, live: false}, collection_hint: '录制状态刷新已过期；以下为最后计数。'};
}
