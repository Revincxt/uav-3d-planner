import { afterEach, beforeEach, describe, expect, it } from 'vitest';
import { packStudyData, restoreStudyData, STUDY_FILES } from '../scripts/study-data.mjs';

const filesystemModule = 'node:fs/promises', osModule = 'node:os', pathModule = 'node:path';
const { mkdtemp, mkdir, readFile, rm, unlink, writeFile } = await import(filesystemModule);
const { tmpdir } = await import(osModule), { join } = await import(pathModule);

describe('lossless study source archives', () => {
  let root: string, options: { publicDir: string; archiveDir: string };
  beforeEach(async () => {
    root = await mkdtemp(join(tmpdir(), 'uav-study-archives-'));
    options = { publicDir: join(root, 'public'), archiveDir: join(root, 'archives') };
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
    const first = await readFile(join(options.archiveDir, `${STUDY_FILES[0]}.gz`));
    await packStudyData(options);
    expect(await readFile(join(options.archiveDir, `${STUDY_FILES[0]}.gz`))).toEqual(first);
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
    await unlink(file); await writeFile(join(options.archiveDir, `${name}.gz`), 'broken archive');
    await expect(restoreStudyData(options)).rejects.toThrow('size mismatch');
    await expect(readFile(file)).rejects.toMatchObject({ code: 'ENOENT' });
  });

  it.each(['checksum', 'traversal', 'version', 'duplicate'])('rejects invalid %s metadata', async (kind) => {
    const file = join(options.archiveDir, 'manifest.json');
    const manifest = JSON.parse(await readFile(file, 'utf8'));
    if (kind === 'checksum') manifest.files[0].sha256 = `sha256:${'0'.repeat(64)}`;
    if (kind === 'traversal') manifest.files[0].archive = '../../outside.gz';
    if (kind === 'version') manifest.version = 2;
    if (kind === 'duplicate') manifest.files[1] = manifest.files[0];
    await writeFile(file, JSON.stringify(manifest));
    await unlink(join(options.publicDir, STUDY_FILES[0]));
    await expect(restoreStudyData(options)).rejects.toThrow();
    await expect(readFile(join(options.publicDir, STUDY_FILES[0]))).rejects.toMatchObject({ code: 'ENOENT' });
  });
});
