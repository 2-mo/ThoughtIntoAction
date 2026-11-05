import { withToken } from "./control-protocol";

export interface QuestSnapshot {
  receiver: boolean;
  usb: string | null;
  serial: string | null;
  reverse: boolean;
  client: boolean;
  peers: string[];
  connected: boolean;
  error: string | null;
}

async function request(path: string, init?: RequestInit): Promise<any> {
  const response = await fetch(withToken(path), { cache: "no-store", ...init });
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
  return data;
}

export async function questStatus(): Promise<QuestSnapshot> {
  return request("/api/v1/system/quest");
}

export async function runAction(name: string): Promise<{ ok: boolean; output: string }> {
  return request(`/api/v1/system/actions/${name}`, { method: "POST" });
}
