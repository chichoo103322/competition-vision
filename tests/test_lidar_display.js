const assert = require('node:assert/strict');
const {mapToCanvas, canvasToMap, polarToMap, ScanDisplayBuffer} = require('../tools/lidar_display.js');

for (const rotated of [false, true]) {
  for (const [x, y] of [[0, 0], [2.4, 2.4], [.15, 2.25], [1.2, 1.2]]) {
    const point = mapToCanvas(x, y, rotated, 42, 900, 340);
    const back = canvasToMap(...point, rotated, 42, 900, 340);
    assert.ok(Math.abs(back[0] - x) < 1e-9);
    assert.ok(Math.abs(back[1] - y) < 1e-9);
  }
}
assert.deepEqual(polarToMap(0, 1, [2.25, 2.25, 180]).map(x => +x.toFixed(6)), [1.25, 2.25]);
assert.deepEqual(polarToMap(Math.PI / 2, 1, [2.25, 2.25, 180]).map(x => +x.toFixed(6)), [2.25, 1.25]);
for (const [local, cellY] of [[[.182652,.841832], 2.069], [[.5807575,.817315], 1.67]]) {
  const world=polarToMap(Math.atan2(local[1],local[0]),Math.hypot(...local),[2.15,2.2,-93.52],-1);
  assert.ok(Math.abs(world[0]-1.299)<.003);
  assert.ok(Math.abs(world[1]-cellY)<.003);
}

const buffer = new ScanDisplayBuffer();
buffer.accept({session_id: 'a', valid: true, age_ms: 20, frame: {seq: 1}}, 100, 5);
assert.equal(buffer.fresh(500), true);
buffer.disconnect();
assert.equal(buffer.fresh(500), false);
assert.equal(buffer.visible(500), true);
assert.equal(buffer.visible(5200), false);
buffer.accept({session_id: 'b', valid: true, age_ms: 0, frame: {seq: 1}}, 6000, 1);
assert.equal(buffer.visible(6001), true);
assert.equal(buffer.fresh(6001), true);

const {RoutePlayback}=require('../tools/lidar_display.js');
const replay=new RoutePlayback();
const plan={status:'ready',plan_id:'eight',segments:Array.from({length:8},(_,i)=>({nodes:i%2?[1,0]:[0,1],label:'segment '+(i+1)}))};
replay.load(plan,[{xy:[0,0]},{xy:[1,0]}]);
replay.play(0);
assert.equal(replay.tick(500).segment,1);
assert.ok(Math.abs(replay.position().xy[0]-.5)<1e-9);
replay.pause(500);
replay.tick(5000);
assert.ok(Math.abs(replay.position().xy[0]-.5)<1e-9);
replay.play(5000);
assert.equal(replay.tick(6200).segment,2);
for(let i=2;i<=8;i++){
 replay.elapsed=(i-1)*1.6+.2;
 assert.equal(replay.position().segment,i);
}
replay.elapsed=13;
assert.equal(replay.tick(6200).done,true);
assert.equal(replay.running,false);
assert.deepEqual(replay.position().xy,[0,0]);
replay.play(7000);
assert.equal(replay.position().segment,1);
assert.equal(replay.elapsed,0);
assert.throws(()=>replay.load({...plan,status:'no_path'},[]));
assert.equal(replay.planId,'eight');
