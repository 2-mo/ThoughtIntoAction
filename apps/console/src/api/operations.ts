import { withToken } from "./control-protocol";

export interface OperationSnapshot {
  kind: string;
  state: string;
  pid: number | null;
  started_at: number;
  exit_code: number | null;
  run_id: string | null;
  log: string;
  argv: string[];
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

export async function listOperations(): Promise<OperationSnapshot[]> {
  const data = await request("/api/v1/operations");
  return (data?.operations ?? []) as OperationSnapshot[];
}

export async function startOperation(kind: string, payload: object = {}): Promise<OperationSnapshot> {
  return request(`/api/v1/operations/${kind}/start`, {
    method: "POST",
    headers: { "content-type": "application/json" },
    body: JSON.stringify(payload),
  });
}

export async function stopOperation(kind: string): Promise<OperationSnapshot> {
  return request(`/api/v1/operations/${kind}/stop`, { method: "POST" });
}

export async function operationLog(kind: string, lines = 60): Promise<string[]> {
  const data = await request(`/api/v1/operations/${kind}/log?lines=${lines}`);
  return (data?.lines ?? []) as string[];
}
