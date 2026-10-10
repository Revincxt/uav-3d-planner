import { describe, expect, it } from 'vitest';
import { parseStudyChunks, readStudyDataWithHash } from '../scripts/study-reader.mjs';

describe('bounded native JSON framing', () => {
  const fixture = { label: '南部 \\ " { } [ ]', protocol: { value: 1e-13 },
    scenarios: [{ id: 'one', nested: [{ text: 'comma, \\" and }] chars', values: [true, null, -1.2e7] }] },
      { id: 'two', positions: [[1.23456789101, -2010.12345, 110]] }],
    optional: null, enabled: true, count: 8 };
  it('preserves every value across all character and escape boundaries', () => {
    const text = JSON.stringify(fixture);
    for (let size = 1; size < 38; size++) {
      const chunks = Array.from({ length: Math.ceil(text.length / size) }, (_, i) => text.slice(i * size, (i + 1) * size));
      expect(parseStudyChunks(chunks)).toEqual(JSON.parse(text));
    }
    expect(parseStudyChunks([JSON.stringify(fixture, null, 2)])).toEqual(fixture);
  });
  it('handles empty roots, empty scenarios and scalar properties in any order', () => {
    for (const text of ['{}', '{"scenarios":[]}', '{"x":-1.23e-8,"scenarios":[null,1,"a"],"z":false}'])
      expect(parseStudyChunks([...text])).toEqual(JSON.parse(text));
  });
  it.each(['', '[]', '{', '{"scenarios":{}}', '{"scenarios":[{},]}', '{"a":1,}',
    '{"a":1}garbage', '{"scenarios":[{"a":"unterminated}]}', '{"a":NaN}', '{"a":[1,]}',
    '{"a":1 "b":2}', '{"a":1,"a":2}', '{"scenarios":[]}{}'])('rejects malformed framing: %s', text => {
    expect(() => parseStudyChunks([...text])).toThrow();
  });
  it('keeps JSON prototype-like property names as own data, never prototype mutation', () => {
    const parsed = parseStudyChunks(['{"__proto__":{"polluted":true},"scenarios":[]}']);
    expect(Object.hasOwn(parsed, '__proto__')).toBe(true);
    expect(Object.getPrototypeOf(parsed)).toBe(Object.prototype);
    expect(parsed.polluted).toBeUndefined();
  });
  it('streams UTF-8 without changing values or the byte-exact source digest', async () => {
    const fsModule='node:fs/promises', osModule='node:os', cryptoModule='node:crypto';
    const fs=await import(fsModule), os=await import(osModule), {createHash}=await import(cryptoModule);
    const directory=await fs.mkdtemp(`${os.tmpdir()}/uav-study-reader-`), file=`${directory}/study.json`;
    const text=JSON.stringify({...fixture,padding:'南'.repeat(400000)})+'\n';
    try {
      await fs.writeFile(file,text);
      const parsed=await readStudyDataWithHash(file);
      expect(parsed.value).toEqual(JSON.parse(text));
      expect(parsed.sha256).toBe(createHash('sha256').update(text).digest('hex'));
    } finally { await fs.rm(directory,{recursive:true,force:true}); }
  });
});
