import { withToken } from "./control-protocol";

export interface EpisodeSummary {
  name: string;
  task: string | null;
  lifecycle: string | null;
  ticks: number | null;
  frames: number | null;
  cameras: string[];
  written_at: string;
}

export interface EpisodeListing {
  directory: string;
  episodes: EpisodeSummary[];
  total: number;
}

export async function deleteEpisode(directory: string, name: string): Promise<void> {
  const query = new URLSearchParams({ directory });
  const response = await fetch(
    withToken(`/api/v1/collection/episodes/${encodeURIComponent(name)}?${query}`),
    { method: "DELETE" },
  );
  if (response.status === 401) throw new Error("配对令牌无效");
  if (response.ok) return;
  const text = await response.text();
  let data: any = null;
  if (text) {
    try {
      data = JSON.parse(text);
    } catch {
      data = null;
    }
  }
  throw new Error(data?.detail || `HTTP ${response.status}`);
}

export async function listEpisodes(directory: string, limit = 6): Promise<EpisodeListing> {
  const query = new URLSearchParams({ directory, limit: String(limit) });
  const response = await fetch(withToken(`/api/v1/collection/episodes?${query}`), { cache: "no-store" });
  if (response.status === 401) throw new Error("配对令牌无效");
  const text = await response.text();
  let data: any = null;
  if (text) {
    try {
      data = JSON.parse(text);
    } catch {
      throw new Error("后端未运行（静态预览）");
    }
  }
  if (!response.ok) throw new Error(data?.detail || `HTTP ${response.status}`);
  return data as EpisodeListing;
}
