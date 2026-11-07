// Preview only: intercept every fetch locally. Never send requests to a robot.
(() => {
  const started = Date.now();
  let left = 0, right = 0, lastDrive = 0, pageId = '', sequence = 0;
  const reply = (status, data) => ({
    ok: status >= 200 && status < 300,
    status,
    json: async () => data
  });

  function snapshot() {
    const uptime = Date.now() - started;
    const t = uptime / 1000;
    if (Date.now() - lastDrive > 500) left = right = 0;
    const leftCm = Math.round(135 + 5 * Math.sin(t));
    const ranges = Array.from({length: 90}, (_, i) => {
      const degrees = i * 4 + 2;
      const a = degrees * Math.PI / 180;
      // Simulate front, right, and left walls with an opening behind the rover.
      if (degrees > 150 && degrees < 225) return 0;
      const hits = [];
      if (Math.cos(a) > 0.05) hits.push(180 / Math.cos(a));
      if (Math.sin(a) > 0.05) hits.push(220 / Math.sin(a));
      if (Math.sin(a) < -0.05) hits.push(leftCm / -Math.sin(a));
      const distance = Math.min(...hits) + 1.5 * Math.sin(t + i);
      return Number.isFinite(distance) && distance <= 300 ? Math.round(distance) : 0;
    });
    const moving = left !== 0 || right !== 0;
    return {
      state: moving ? `Preview: driving (left ${left.toFixed(2)}, right ${right.toFixed(2)})`
        : 'Preview: stopped — simulated sensors running',
      values: [
        {name: 'Mode', value: 'Browser simulation'},
        {name: 'Left speed', value: left.toFixed(2)},
        {name: 'Right speed', value: right.toFixed(2)},
        {name: 'Uptime', value: `${Math.floor(t)} s`}
      ],
      cycle: {min: 9.8, avg: 10, max: 10.6},
      lidarAgeMs: 25,
      sensors: {
        us: [180, leftCm, 220, 300, 165], usMax: 300,
        lidar: {ok: true, d: ranges, pps: 340, crc: 0},
        imu: {
          ok: true, sample: true, ageMs: 25, timestampMs: uptime,
          heading: (90 + 2 * Math.sin(t / 3)).toFixed(1),
          roll: (0.8 * Math.sin(t)).toFixed(1),
          pitch: (0.5 * Math.cos(t)).toFixed(1), cal: [3, 3, 3, 3]
        },
        gps: {chars: 12000 + Math.floor(t) * 100, fix: false, sats: 4, hdop: 2.1}
      }
    };
  }

  window.fetch = async (input) => {
    const url = new URL(String(input), 'http://preview.local');
    if (url.pathname === '/values') return reply(200, snapshot());
    if (url.pathname === '/drive') {
      const id = url.searchParams.get('id');
      const n = Number(url.searchParams.get('n'));
      if (id === pageId && n <= sequence) return reply(200, {ok: false, stale: true});
      pageId = id;
      sequence = n;
      const l = Number(url.searchParams.get('l'));
      const r = Number(url.searchParams.get('r'));
      left = l;
      right = r;
      lastDrive = Date.now();
      return reply(200, {ok: true});
    }
    return reply(404, {ok: false, error: 'This route is not simulated'});
  };
})();
