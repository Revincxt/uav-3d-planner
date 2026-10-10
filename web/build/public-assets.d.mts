export const PUBLIC_SITE_ASSETS: readonly string[];
export function publicAssetsPlugin(): {
  name: string;
  configResolved(config: { publicDir: string }): void;
  generateBundle(this: { emitFile(asset: { type: 'asset'; fileName: string; source: Uint8Array }): unknown }): Promise<void>;
};
