import { createHash, randomUUID } from 'node:crypto';
import { createReadStream, createWriteStream } from 'node:fs';
import { link, mkdir, open, readFile, rename, stat, unlink, writeFile } from 'node:fs/promises';
import { resolve } from 'node:path';
import { Readable, Transform } from 'node:stream';
import { pipeline } from 'node:stream/promises';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { createGunzip, createGzip } from 'node:zlib';

export const STUDY_FILES = ['demo-data.json', 'dynamic-data.json', 'predictive-data.json'];
// Below GitHub's browser upload limit too. Streaming preserves exact records
// without requiring a second huge RAM copy when the city and flights grow.
export const MAX_ARCHIVE_PART_BYTES = 24 * 1024 * 1024;
const MAX_NATIVE_BYTES = 1024 * 1024 * 1024;
const defaults = {
  publicDir: fileURLToPath(new URL('../public/', import.meta.url)),
  archiveDir: fileURLToPath(new URL('../../data/studies/', import.meta.url)),
};
const checksum = hash => `sha256:${hash.digest('hex')}`;
const validHash = hash => typeof hash === 'string' && /^sha256:[a-f0-9]{64}$/.test(hash);
const positiveInteger = number => Number.isSafeInteger(number) && number > 0;

export async function packStudyData(options = {}) {
  const { publicDir, archiveDir, partBytes = MAX_ARCHIVE_PART_BYTES } = { ...defaults, ...options };
  if (!positiveInteger(partBytes) || partBytes > MAX_ARCHIVE_PART_BYTES) throw new Error('Invalid archive part size');
  await mkdir(archiveDir, { recursive: true });
  const files = [];
  for (const name of STUDY_FILES) {
    const hash = createHash('sha256'), parts = [];
    let bytes = 0, part, handle, partHash;
    const meter = new Transform({ transform(chunk, encoding, callback) {
      bytes += chunk.length; hash.update(chunk);
      callback(bytes > MAX_NATIVE_BYTES ? new Error(`Study source too large: ${name}`) : null, chunk);
    } });
    const closePart = async () => {
      if (!handle) return;
      await handle.close(); handle = undefined;
      part.sha256 = checksum(partHash);
      const archive = `${name}.gz.${part.sha256.slice(7)}.part${String(parts.length + 1).padStart(3, '0')}`;
      // Content-addressed shards leave the previous manifest valid even if a
      // later export or compression fails before the atomic manifest swap.
      await rename(resolve(archiveDir, part.archive), resolve(archiveDir, archive));
      part.archive = archive; parts.push(part);
    };
    try {
      await pipeline(createReadStream(resolve(publicDir, name)), meter, createGzip({ level: 9 }), async source => {
        for await (const chunk of source) {
          let offset = 0;
          while (offset < chunk.length) {
            if (!handle) {
              part = { archive: `.${name}.${randomUUID()}.pack`, bytes: 0 };
              handle = await open(resolve(archiveDir, part.archive), 'wx');
              partHash = createHash('sha256');
            }
            const slice = chunk.subarray(offset, offset + Math.min(chunk.length - offset, partBytes - part.bytes));
            await handle.writeFile(slice); partHash.update(slice);
            part.bytes += slice.length; offset += slice.length;
            if (part.bytes === partBytes) await closePart();
          }
        }
        await closePart();
      });
    } finally {
      if (handle) {
        await handle.close();
        await unlink(resolve(archiveDir, part.archive));
      }
    }
    if (!bytes) throw new Error(`Empty study source: ${name}`);
    files.push({ name, bytes, archiveBytes: parts.reduce((sum, item) => sum + item.bytes, 0), sha256: checksum(hash), parts });
  }
  const temporary = resolve(archiveDir, `.manifest.${randomUUID()}.tmp`);
  await writeFile(temporary, `${JSON.stringify({ version: 2, files }, null, 2)}\n`, { flag: 'wx' });
  await rename(temporary, resolve(archiveDir, 'manifest.json'));
  return files;
}

function validateManifest(manifest) {
  if (manifest.version !== 2 || !Array.isArray(manifest.files) || manifest.files.length !== STUDY_FILES.length)
    throw new Error('Invalid study archive manifest');
  for (const [index, name] of STUDY_FILES.entries()) {
    const entry = manifest.files[index];
    if (!entry || entry.name !== name || !positiveInteger(entry.bytes) || entry.bytes > MAX_NATIVE_BYTES
      || !positiveInteger(entry.archiveBytes) || !validHash(entry.sha256)) throw new Error('Invalid study archive entry');
    if (!Array.isArray(entry.parts) || !entry.parts.length || entry.parts.length > 128
      || entry.parts.some((part, number) => !validHash(part.sha256)
        || part.archive !== `${name}.gz.${part.sha256.slice(7)}.part${String(number + 1).padStart(3, '0')}`
        || !positiveInteger(part.bytes) || part.bytes > MAX_ARCHIVE_PART_BYTES)
      || entry.parts.reduce((sum, part) => sum + part.bytes, 0) !== entry.archiveBytes)
      throw new Error('Invalid study archive parts');
  }
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
  validateManifest(manifest);
  await mkdir(publicDir, { recursive: true });
  const restored = [];
  for (const name of missing) {
    const entry = manifest.files[STUDY_FILES.indexOf(name)];
    const temporary = resolve(publicDir, `.${name}.${randomUUID()}.restore`);
    const hash = createHash('sha256'); let bytes = 0;
    const meter = new Transform({ transform(chunk, encoding, callback) {
      bytes += chunk.length; hash.update(chunk);
      callback(bytes > entry.bytes ? new Error(`Study archive size mismatch: ${name}`) : null, chunk);
    } });
    async function* compressed() {
      for (const part of entry.parts) {
        if ((await stat(resolve(archiveDir, part.archive))).size !== part.bytes)
          throw new Error(`Study archive size mismatch: ${name}`);
        const chunk = await readFile(resolve(archiveDir, part.archive));
        if (checksum(createHash('sha256').update(chunk)) !== part.sha256)
          throw new Error(`Study archive part checksum mismatch: ${name}`);
        yield chunk;
      }
    }
    try {
      await pipeline(Readable.from(compressed()), createGunzip(), meter, createWriteStream(temporary, { flags: 'wx' }));
      if (bytes !== entry.bytes || checksum(hash) !== entry.sha256) throw new Error(`Study archive checksum mismatch: ${name}`);
      // An atomic, exclusive hard link also preserves exports created concurrently.
      try { await link(temporary, resolve(publicDir, name)); restored.push(name); }
      catch (error) { if (error.code !== 'EEXIST') throw error; }
    } finally {
      await unlink(temporary).catch(error => { if (error.code !== 'ENOENT') throw error; });
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
