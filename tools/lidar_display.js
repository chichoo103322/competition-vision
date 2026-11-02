/* Human viewing only. Old scans never become fresh by being polled again. */
class ScanDisplayBuffer {
  constructor() { this.clear(); }
  clear() { this.cached = null; this.deadline = 0; this.origin = 0; this.latestValid = false; this.session = null; this.seq = null; }
  accept(snapshot, now, rtt) {
    if (snapshot.session_id !== this.session) {
      this.clear(); this.session = snapshot.session_id;
    }
    this.latestValid = false;
    if (!snapshot.valid || !snapshot.frame || !Number.isFinite(snapshot.age_ms)) return;
    const age = Math.max(0, snapshot.age_ms) + Math.max(0, rtt);
    const origin = now - age;
    if (snapshot.frame.seq !== this.seq) {
      this.seq = snapshot.frame.seq; this.origin = origin;
      this.cached = snapshot; this.deadline = origin + 5000;
    } else {
      // A repeated/queued response may only shorten a source's remaining lifetime.
      this.origin = Math.min(this.origin, origin);
      this.deadline = Math.min(this.deadline, this.origin + 5000);
    }
    this.latestValid = true;
  }
  disconnect() { this.latestValid = false; }
  age(now) { return this.cached ? Math.max(0, now - this.origin) : null; }
  fresh(now) { return !!(this.cached && this.latestValid && this.age(now) < 1000); }
  visible(now) { return !!(this.cached && now < this.deadline); }
}
function mapToCanvas(x, y, rotated, pad, size, scale) {
  return rotated ? [pad + (2.4 - x) * scale, pad + y * scale]
    : [pad + x * scale, size - pad - y * scale];
}
function canvasToMap(sx, sy, rotated, pad, size, scale) {
  const x = (sx - pad) / scale;
  const y = rotated ? (sy - pad) / scale : (size - pad - sy) / scale;
  return rotated ? [2.4 - x, y] : [x, y];
}
function polarToMap(angle, range, pose, angleSign = 1) {
  const a = angleSign * angle + pose[2] * Math.PI / 180;
  return [pose[0] + Math.cos(a) * range, pose[1] + Math.sin(a) * range];
}

/* A screen-only replay of the saved route geometry, with no hardware output. */
function canPlayMission(plan, live) {
  return !!(live && plan?.status==='ready' && plan.evidence_status==='snapshot_preview');
}
class RoutePlayback {
  constructor() { this.reset(); }
  reset() { this.stages=[]; this.planId=null; this.elapsed=0; this.running=false; this.last=null; this.rate=1; }
  load(plan, nodes) {
    if (!plan || plan.status!=='ready' || plan.segments.length!==8) throw Error('需要完整的8段路线');
    const stages=plan.segments.map(segment=>{
      const points=segment.nodes.map(id=>nodes[id].xy.slice());
      if(!points.length)throw Error('路线缺少节点');
      const lengths=points.slice(1).map((p,i)=>Math.hypot(p[0]-points[i][0],p[1]-points[i][1]));
      return {label:segment.label,points,lengths,length:lengths.reduce((a,b)=>a+b,0)};
    });
    this.reset(); this.stages=stages; this.planId=plan.plan_id;
  }
  play(now) { if(!this.stages.length)return; if(this.position().done)this.elapsed=0; this.running=true; this.last=now; }
  pause(now) { this.tick(now); this.running=false; this.last=null; }
  restart(now) { this.elapsed=0; this.running=true; this.last=now; }
  tick(now, allowed = true) {
    // Freeze before advancing when live evidence invalidates the saved route.
    if(!allowed){this.running=false;this.last=null;return this.position();}
    if(this.running&&this.last!==null) this.elapsed+=Math.max(0,now-this.last)/1000*this.rate;
    this.last=now;
    const result=this.position(); if(result?.done)this.running=false;
    return result;
  }
  position() {
    if(!this.stages.length)return null;
    let t=this.elapsed;
    for(let i=0;i<this.stages.length;i++){
      const stage=this.stages[i],duration=stage.length+.6;
      if(t<duration||i===this.stages.length-1){
        let distance=Math.min(t,stage.length),heading=0;
        for(let k=0;k<stage.lengths.length;k++){
          const length=stage.lengths[k],a=stage.points[k],b=stage.points[k+1];
          heading=Math.atan2(b[1]-a[1],b[0]-a[0]);
          if(length>0&&distance<length)return {xy:[a[0]+(b[0]-a[0])*distance/length,a[1]+(b[1]-a[1])*distance/length],heading,segment:i+1,label:stage.label,done:false,atStation:false};
          distance-=length;
        }
        return {xy:stage.points[stage.points.length-1].slice(),heading,segment:i+1,label:stage.label,
                done:i===this.stages.length-1&&t>=duration,atStation:true};
      }
      t-=duration;
    }
  }
}
if (typeof module !== 'undefined') module.exports = {ScanDisplayBuffer, mapToCanvas, canvasToMap, polarToMap, RoutePlayback, canPlayMission};
