import { describe, expect, it } from 'vitest';

import { trafficLightModeToRust } from './commands';

describe('trafficLightModeToRust', () => {
  it('lowercases a single word', () => {
    expect(trafficLightModeToRust('Manual')).toBe('manual');
    expect(trafficLightModeToRust('Auto')).toBe('auto');
  });

  it('inserts underscores between lower and upper case segments', () => {
    expect(trafficLightModeToRust('SemiAuto')).toBe('semi_auto');
    expect(trafficLightModeToRust('Adaptive')).toBe('adaptive');
  });

  it('does not split consecutive leading capitals (only lower→upper pairs)', () => {
    expect(trafficLightModeToRust('XMLParser')).toBe('xmlparser');
  });
});
