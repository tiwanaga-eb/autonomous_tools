// busy/error の共通ハンドリング。各パネルで散在していた
//   setBusy(true); try { ... } catch(e){ setStatus(String(e)) } finally { setBusy(false) }
// を1か所へ集約し、二重起動防止・エラーメッセージ整形（ApiError.detail）も統一する。

import { ApiError } from "@/api/client";
import { useStore } from "@/store/useStore";

/** unknown なエラーから人間可読メッセージを取り出す（ApiError は整形済み message を持つ）。 */
export function errMessage(e: unknown): string {
  if (e instanceof ApiError) return e.message;
  if (e instanceof Error) return e.message;
  return String(e);
}

interface RunBusyOptions {
  /** 成功時に表示するステータス（省略時は変更しない）。 */
  successMessage?: string;
  /** エラー時メッセージの接頭辞（例: "経路生成に失敗"）。 */
  failPrefix?: string;
}

/**
 * 非同期処理を busy 表示・例外捕捉つきで実行する。
 * - 実行中は busy=true（既に busy なら二重起動を防ぎ undefined を返す）
 * - 例外は status(error) に整形表示し、握り潰す（呼び出し側 UI を壊さない）
 * - finally で必ず busy=false
 */
export async function runBusy<T>(
  label: string,
  fn: () => Promise<T>,
  opts: RunBusyOptions = {},
): Promise<T | undefined> {
  const { busy, setBusy, setStatus } = useStore.getState();
  if (busy) return undefined; // 二重起動ガード
  setBusy(true, label);
  try {
    const result = await fn();
    if (opts.successMessage) setStatus(opts.successMessage, "success");
    return result;
  } catch (e) {
    const prefix = opts.failPrefix ? `${opts.failPrefix}: ` : "";
    setStatus(`${prefix}${errMessage(e)}`, "error");
    return undefined;
  } finally {
    setBusy(false);
  }
}
