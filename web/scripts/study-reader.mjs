import { createReadStream } from 'node:fs';
import { createHash } from 'node:crypto';
import { StringDecoder } from 'node:string_decoder';

// Native histories can exceed V8's single-string limit. Frame one scenario at a
// time, then let JSON.parse decode every value; never round or prune records.
class StudyReader {
  state = 'root'; value = {}; key; frame; pieces = []; characters = 0;
  whitespace(char) { return char === ' ' || char === '\n' || char === '\r' || char === '\t'; }
  fail() { throw new SyntaxError('Invalid native study JSON'); }
  append(part) {
    this.characters += part.length;
    if (this.characters > 256 * 1024 * 1024) throw new Error('Native study scenario exceeds the parsing budget');
    this.pieces.push(part);
  }
  begin(char, target) {
    this.frame = { target, depth: char === '{' || char === '[' ? 1 : 0,
      string: char === '"', escaped: false, primitive: char !== '"' && char !== '{' && char !== '[' };
    this.pieces = []; this.characters = 0;
  }
  complete() {
    const decoded = JSON.parse(this.pieces.join('')), target = this.frame.target;
    this.frame = undefined; this.pieces = []; this.characters = 0;
    if (target === 'key') {
      if (typeof decoded !== 'string' || Object.hasOwn(this.value, decoded)) this.fail();
      this.key = decoded; this.state = 'colon';
    } else if (target === 'item') {
      this.value.scenarios.push(decoded); this.state = 'item-separator';
    } else {
      Object.defineProperty(this.value, this.key, { value: decoded, enumerable: true, writable: true, configurable: true });
      this.state = 'property-separator';
    }
  }
  write(chunk) {
    let start = this.frame ? 0 : -1;
    for (let index = 0; index < chunk.length; index++) {
      const char = chunk[index];
      if (this.frame) {
        const frame = this.frame;
        if (frame.primitive) {
          if (this.whitespace(char) || char === ',' || char === '}' || char === ']') {
            this.append(chunk.slice(start, index)); this.complete(); start = -1; index--; continue;
          }
        } else if (frame.string) {
          if (frame.escaped) frame.escaped = false;
          else if (char === '\\') frame.escaped = true;
          else if (char === '"') {
            frame.string = false;
            if (!frame.depth) {
              this.append(chunk.slice(start, index + 1)); this.complete(); start = -1;
            }
          }
        } else if (char === '"') frame.string = true;
        else if (char === '{' || char === '[') frame.depth++;
        else if (char === '}' || char === ']') {
          if (--frame.depth === 0) {
            this.append(chunk.slice(start, index + 1)); this.complete(); start = -1;
          }
        }
        continue;
      }
      if (this.whitespace(char)) continue;
      if (this.state === 'root') {
        if (char !== '{') this.fail(); this.state = 'key-or-end';
      } else if (this.state === 'key-or-end' || this.state === 'key') {
        if (char === '}' && this.state === 'key-or-end') this.state = 'done';
        else { if (char !== '"') this.fail(); this.begin(char, 'key'); start = index; }
      } else if (this.state === 'colon') {
        if (char !== ':') this.fail(); this.state = 'value';
      } else if (this.state === 'value') {
        if (this.key === 'scenarios') {
          if (char !== '[') this.fail(); this.value.scenarios = []; this.state = 'item-or-end';
        } else { this.begin(char, 'property'); start = index; }
      } else if (this.state === 'item-or-end' || this.state === 'item') {
        if (char === ']' && this.state === 'item-or-end') this.state = 'property-separator';
        else { if (char === ']' || char === ',') this.fail(); this.begin(char, 'item'); start = index; }
      } else if (this.state === 'item-separator') {
        if (char === ',') this.state = 'item';
        else if (char === ']') this.state = 'property-separator';
        else this.fail();
      } else if (this.state === 'property-separator') {
        if (char === ',') this.state = 'key';
        else if (char === '}') this.state = 'done';
        else this.fail();
      } else this.fail();
    }
    if (this.frame) this.append(chunk.slice(start));
  }
  finish() {
    if (this.frame || this.state !== 'done') this.fail();
    return this.value;
  }
}

export function parseStudyChunks(chunks) {
  const reader = new StudyReader();
  for (const chunk of chunks) reader.write(chunk);
  return reader.finish();
}

export async function readStudyDataWithHash(file) {
  const reader = new StudyReader(), decoder = new StringDecoder('utf8'), hash = createHash('sha256');
  for await (const chunk of createReadStream(file, { highWaterMark: 1024 * 1024 })) {
    hash.update(chunk); reader.write(decoder.write(chunk));
  }
  reader.write(decoder.end());
  return { value: reader.finish(), sha256: hash.digest('hex') };
}

export async function readStudyData(file) { return (await readStudyDataWithHash(file)).value; }
