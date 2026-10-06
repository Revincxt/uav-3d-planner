import { createHash, randomUUID } from 'node:crypto';
import { link, mkdir, readFile, stat, unlink, writeFile } from 'node:fs/promises';
import { resolve } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { gunzipSync, gzipSync } from 'node:zlib';

export const STUDY_FILES = ['demo-data.json', 'dynamic-data.json', 'predictive-data.json'];
const defaults = {
  publicDir: fileURLToPath(new URL('../public/', import.meta.url)),
  archiveDir: fileURLToPath(new URL('../../data/studies/', import.meta.url)),
};
const digest = bytes => `sha256:${createHash('sha256').update(bytes).digest('hex')}`;

export async function packStudyData(options = {}) {
  const { publicDir, archiveDir } = { ...defaults, ...options };
  await mkdir(archiveDir, { recursive: true });
  const files = [];
  for (const name of STUDY_FILES) {
    const bytes = await readFile(resolve(publicDir, name));
    const archive = `${name}.gz`, compressed = gzipSync(bytes, { level: 9 });
    await writeFile(resolve(archiveDir, archive), compressed);
    files.push({ name, archive, bytes: bytes.length, archiveBytes: compressed.length, sha256: digest(bytes) });
  }
  await writeFile(resolve(archiveDir, 'manifest.json'), `${JSON.stringify({ version: 1, files }, null, 2)}\n`);
  return files;
}

export async function restoreStudyData(options = {}) {
  const { publicDir, archiveDir } = { ...defaults, ...options };
  const missing = [];
  for (const name of STUDY_FILES) {
    try {
      if (!(await stat(resolve(publicDir, name))).isFile()) throw new Error(`Not a study file: ${name}`);
    } catch (error) {
      if (error.code !== 'ENOENT') throw error;
      missing.push(name);
    }
  }
  // Local exports are never replaced. Repack them explicitly before committing.
  if (!missing.length) return [];
  const manifest = JSON.parse(await readFile(resolve(archiveDir, 'manifest.json'), 'utf8'));
  if (manifest.version !== 1 || !Array.isArray(manifest.files) || manifest.files.length !== STUDY_FILES.length) {
    throw new Error('Invalid study archive manifest');
  }
  for (const [index, name] of STUDY_FILES.entries()) {
    const entry = manifest.files[index];
    if (!entry || entry.name !== name || entry.archive !== `${name}.gz`
      || !Number.isSafeInteger(entry.bytes) || entry.bytes <= 0 || entry.bytes > 256 * 1024 * 1024
      || !Number.isSafeInteger(entry.archiveBytes) || entry.archiveBytes <= 0
      || !/^sha256:[a-f0-9]{64}$/.test(entry.sha256)) throw new Error('Invalid study archive entry');
  }
  await mkdir(publicDir, { recursive: true });
  const restored = [];
  for (const name of missing) {
    const entry = manifest.files[STUDY_FILES.indexOf(name)];
    const compressed = await readFile(resolve(archiveDir, entry.archive));
    if (compressed.length !== entry.archiveBytes) throw new Error(`Study archive size mismatch: ${name}`);
    const bytes = gunzipSync(compressed, { maxOutputLength: entry.bytes });
    if (bytes.length !== entry.bytes || digest(bytes) !== entry.sha256) throw new Error(`Study archive checksum mismatch: ${name}`);
    const temporary = resolve(publicDir, `.${name}.${randomUUID()}.restore`);
    await writeFile(temporary, bytes, { flag: 'wx' });
    try {
      // An atomic, exclusive hard link also preserves exports created concurrently.
      await link(temporary, resolve(publicDir, name));
      restored.push(name);
    } catch (error) {
      if (error.code !== 'EEXIST') throw error;
    } finally {
      await unlink(temporary);
    }
  }
  return restored;
}

if (process.argv[1] && import.meta.url === pathToFileURL(resolve(process.argv[1])).href) {
  if (process.argv[2] === '--pack') console.log(JSON.stringify(await packStudyData(), null, 2));
  else if (!process.argv[2]) {
    const restored = await restoreStudyData();
    if (restored.length) console.log(`Restored verified study data: ${restored.join(', ')}`);
  } else throw new Error('Usage: node scripts/study-data.mjs [--pack]');
}
