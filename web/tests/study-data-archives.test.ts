import { afterEach, beforeEach, describe, expect, it } from 'vitest';
import { packStudyData, restoreStudyData, STUDY_FILES, MAX_ARCHIVE_PART_BYTES } from '../scripts/study-data.mjs';
import type { StudyArchiveEntry } from '../scripts/study-data.mjs';

const filesystemModule = 'node:fs/promises', osModule = 'node:os', pathModule = 'node:path';
const { mkdtemp, mkdir, readFile, readdir, stat, rm, unlink, writeFile } = await import(filesystemModule);
const { tmpdir } = await import(osModule), { join } = await import(pathModule);

describe('lossless study source archives', () => {
  let root: string, options: { publicDir: string; archiveDir: string; partBytes: number };
  beforeEach(async () => {
    root = await mkdtemp(join(tmpdir(), 'uav-study-archives-'));
    options = { publicDir: join(root, 'public'), archiveDir: join(root, 'archives'), partBytes: 32 };
    await mkdir(options.publicDir);
    for (const name of STUDY_FILES) await writeFile(join(options.publicDir, name), `${JSON.stringify({ name, altitude: 33.000, route: [[1, 2, 3]] }, null, 2)}\n`);
    await packStudyData(options);
  });
  afterEach(async () => { await rm(root, { recursive: true, force: true }); });

  it('restores every byte, including formatting and the final newline', async () => {
    const originals = await Promise.all(STUDY_FILES.map((name: string) => readFile(join(options.publicDir, name))));
    await Promise.all(STUDY_FILES.map((name: string) => unlink(join(options.publicDir, name))));
    expect(await restoreStudyData(options)).toEqual(STUDY_FILES);
    for (const [index, name] of STUDY_FILES.entries()) expect(await readFile(join(options.publicDir, name))).toEqual(originals[index]);
  });

  it('preserves local exports instead of overwriting them', async () => {
    const file = join(options.publicDir, STUDY_FILES[0]);
    await writeFile(file, 'local export\n');
    expect(await restoreStudyData(options)).toEqual([]);
    expect(await readFile(file, 'utf8')).toBe('local export\n');
  });

  it('can repack new exports deterministically', async () => {
    const manifest = JSON.parse(await readFile(join(options.archiveDir, 'manifest.json'), 'utf8'));
    const first = await readFile(join(options.archiveDir, manifest.files[0].parts[0].archive));
    await packStudyData(options);
    expect(await readFile(join(options.archiveDir, manifest.files[0].parts[0].archive))).toEqual(first);
    const file = join(options.publicDir, STUDY_FILES[0]);
    await writeFile(file, 'updated export\n');
    await packStudyData(options); await unlink(file); await restoreStudyData(options);
    expect(await readFile(file, 'utf8')).toBe('updated export\n');
  });

  it('does not replace a target created by a concurrent restoration', async () => {
    await Promise.all(STUDY_FILES.map((name: string) => unlink(join(options.publicDir, name))));
    await Promise.all([restoreStudyData(options), restoreStudyData(options)]);
    expect(await restoreStudyData(options)).toEqual([]);
  });

  it('rejects corrupt compressed data before creating a destination', async () => {
    const name = STUDY_FILES[0], file = join(options.publicDir, name);
    const manifest = JSON.parse(await readFile(join(options.archiveDir, 'manifest.json'), 'utf8'));
    await unlink(file); await writeFile(join(options.archiveDir, manifest.files[0].parts[0].archive), 'broken archive');
    await expect(restoreStudyData(options)).rejects.toThrow('size mismatch');
    await expect(readFile(file)).rejects.toMatchObject({ code: 'ENOENT' });
  });

  it.each(['checksum', 'traversal', 'version', 'duplicate'])('rejects invalid %s metadata', async (kind) => {
    const file = join(options.archiveDir, 'manifest.json');
    const manifest = JSON.parse(await readFile(file, 'utf8'));
    if (kind === 'checksum') manifest.files[0].sha256 = `sha256:${'0'.repeat(64)}`;
    if (kind === 'traversal') manifest.files[0].parts[0].archive = '../../outside.gz';
    if (kind === 'version') manifest.version = 3;
    if (kind === 'duplicate') manifest.files[1] = manifest.files[0];
    await writeFile(file, JSON.stringify(manifest));
    await unlink(join(options.publicDir, STUDY_FILES[0]));
    await expect(restoreStudyData(options)).rejects.toThrow();
    await expect(readFile(join(options.publicDir, STUDY_FILES[0]))).rejects.toMatchObject({ code: 'ENOENT' });
  });

  it('caps every compressed shard and restores a gzip stream split across boundaries', async () => {
    const manifest = JSON.parse(await readFile(join(options.archiveDir, 'manifest.json'), 'utf8'));
    expect(manifest.version).toBe(2);
    expect(MAX_ARCHIVE_PART_BYTES).toBeLessThan(25 * 1024 * 1024);
    for (const entry of manifest.files) {
      expect(entry.parts.length).toBeGreaterThan(1);
      expect(entry.parts.every((part: { bytes: number }) => part.bytes <= options.partBytes)).toBe(true);
      expect(entry.archiveBytes).toBe(entry.parts.reduce((sum: number, part: { bytes: number }) => sum + part.bytes, 0));
    }
    await expect(packStudyData({ ...options, partBytes: MAX_ARCHIVE_PART_BYTES + 1 })).rejects.toThrow('part size');
  });

  it('rejects a same-size corrupt shard without publishing a partial native file', async () => {
    const manifest = JSON.parse(await readFile(join(options.archiveDir, 'manifest.json'), 'utf8'));
    const name = STUDY_FILES[0], part = join(options.archiveDir, manifest.files[0].parts[1].archive);
    const bytes = await readFile(part); bytes[0] ^= 1; await writeFile(part, bytes);
    await unlink(join(options.publicDir, name));
    await expect(restoreStudyData(options)).rejects.toThrow('part checksum mismatch');
    await expect(readFile(join(options.publicDir, name))).rejects.toMatchObject({ code: 'ENOENT' });
  });

  it('keeps the previously published shards restorable after a failed repack', async () => {
    const manifestPath = join(options.archiveDir, 'manifest.json'), manifest = await readFile(manifestPath);
    const original = await readFile(join(options.publicDir, STUDY_FILES[0]));
    await writeFile(join(options.publicDir, STUDY_FILES[0]), 'new export\n');
    await unlink(join(options.publicDir, STUDY_FILES[1]));
    await expect(packStudyData(options)).rejects.toThrow();
    expect(await readFile(manifestPath)).toEqual(manifest);
    await unlink(join(options.publicDir, STUDY_FILES[0])); await restoreStudyData(options);
    expect(await readFile(join(options.publicDir, STUDY_FILES[0]))).toEqual(original);
  });

  it('rejects the retired single-gzip format instead of silently bypassing shard checks', async () => {
    const path = join(options.archiveDir, 'manifest.json');
    const manifest = JSON.parse(await readFile(path, 'utf8')); manifest.version = 1;
    await writeFile(path, JSON.stringify(manifest));
    await unlink(join(options.publicDir, STUDY_FILES[0]));
    await expect(restoreStudyData(options)).rejects.toThrow('Invalid study archive manifest');
    await expect(readFile(join(options.publicDir, STUDY_FILES[0]))).rejects.toMatchObject({ code: 'ENOENT' });
  });

  it('rejects missing, reordered and oversized shards', async () => {
    const path = join(options.archiveDir, 'manifest.json'), original = await readFile(path, 'utf8');
    await unlink(join(options.publicDir, STUDY_FILES[0]));
    for (const kind of ['missing', 'order', 'oversize']) {
      const manifest = JSON.parse(original), entry = manifest.files[0];
      if (kind === 'missing') entry.parts.pop();
      if (kind === 'order') entry.parts.reverse();
      if (kind === 'oversize') entry.parts[0].bytes = MAX_ARCHIVE_PART_BYTES + 1;
      await writeFile(path, JSON.stringify(manifest));
      await expect(restoreStudyData(options)).rejects.toThrow('archive parts');
    }
  });
});

describe('committed study inventory', () => {
  it('contains only current manifest-referenced archives, within the upload limit', async () => {
    const directory = new URL('../../data/studies/', import.meta.url);
    const manifest = JSON.parse(await readFile(new URL('manifest.json', directory), 'utf8')) as {
      version: number; files: StudyArchiveEntry[];
    };
    const filenames: string[] = await readdir(directory);
    expect(manifest.version).toBe(2);
    expect(filenames.filter(name => name.includes('.gz')).sort()).toEqual(
      manifest.files.flatMap(entry => entry.parts.map(part => part.archive)).sort(),
    );
    for (const entry of manifest.files) for (const part of entry.parts) {
      expect((await stat(new URL(part.archive, directory))).size).toBe(part.bytes);
      expect(part.bytes).toBeLessThanOrEqual(MAX_ARCHIVE_PART_BYTES);
    }
  });
});
