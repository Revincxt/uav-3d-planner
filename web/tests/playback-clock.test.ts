import { describe, expect, it } from "vitest";
import { BASE_PLAYBACK_SPEED, DEFAULT_PLAYBACK_RATE, PLAYBACK_RATES, PlaybackClock } from "../src/playback-clock";

describe("Shared accelerated playback clock", () => {
  it("defaults to four trace seconds for one wall second", () => {
    const clock = new PlaybackClock(); clock.start(20,1000);
    expect(BASE_PLAYBACK_SPEED).toBe(4); expect(DEFAULT_PLAYBACK_RATE).toBe(1); expect(clock.rate).toBe(1); expect(PLAYBACK_RATES).toEqual([0.5,1,2]);
    expect(clock.sample(1500)).toBe(22); expect(clock.sample(2000)).toBe(24);
  });
  it("does not lose time when frames are delayed and does not depend on frame count", () => {
    const clock = new PlaybackClock(); clock.start(0,1000);
    for (let frame = 1000; frame < 2000; frame += 15) clock.sample(frame);
    expect(clock.sample(6000)).toBe(20);
    const sparse = new PlaybackClock(); sparse.start(0,1000);
    expect(sparse.sample(6000)).toBe(clock.sample(6000));
  });
  it("rebases speed changes at the current shared instant without jumping", () => {
    const clock = new PlaybackClock(); clock.start(100,1000);
    const current = clock.sample(2500);
    expect(clock.setRate(2,current,2500)).toBe(true);
    expect(clock.sample(2500)).toBe(current); expect(clock.sample(3500)).toBe(current + 8);
    expect(clock.setRate(1,current + 8,3500)).toBe(true); expect(clock.sample(4500)).toBe(current + 12);
  });
  it("restarts from sought or paused time, not from elapsed pause duration", () => {
    const clock = new PlaybackClock(); clock.start(0,1000);
    const paused = clock.sample(2000);
    clock.start(paused,12000); expect(clock.sample(12250)).toBe(paused + 1);
    clock.start(50,15000); expect(clock.sample(15000)).toBe(50); expect(clock.sample(15250)).toBe(51);
    expect(clock.sample(14000)).toBe(50);
  });
  it("rejects unsupported rates and non-finite clock values", () => {
    const clock = new PlaybackClock(); clock.start(0,1000);
    for (const rate of [0,4,-1,60,NaN,Infinity]) expect(clock.setRate(rate,2,1500)).toBe(false);
    expect(clock.rate).toBe(1); expect(clock.sample(2000)).toBe(4);
    expect(() => clock.start(NaN,1000)).toThrow(); expect(() => clock.start(-1,1000)).toThrow();
    expect(() => clock.sample(Infinity)).toThrow();
  });
  it("offers half speed relative to the normalized normal pace", () => {
    const clock = new PlaybackClock(); expect(clock.setRate(0.5,10,1000)).toBe(true);
    expect(clock.sample(2000)).toBe(12);
  });
});
