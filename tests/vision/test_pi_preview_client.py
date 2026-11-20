"""Run the shipped JS with a deterministic clock/DOM/network, not a real browser."""

import json
import shutil
import subprocess
import unittest

from pi_vision.preview_page import FRAME_CLIENT_JS


HARNESS = r"""
const vm=require('node:vm'),assert=require('node:assert/strict');
const input=JSON.parse(require('node:fs').readFileSync(0,'utf8'));
let now=0,nextId=0,urlId=0;
const timers=new Map(),replies=[],events={},windowEvents={};
const img={src:'',removeAttribute(){this.src='';}},svg={style:{}},status={textContent:''};
const button={addEventListener(name,fn){this[name]=fn;}};
const document={hidden:false,getElementById(id){return id==='video-status'?status:button;},
  addEventListener(name,fn){events[name]=fn;}};
function timeout(fn,delay){const id=++nextId;timers.set(id,{fn,at:now+delay});return id;}
const context=vm.createContext({document,img,svg,sourceWidth:0,sourceHeight:0,applyViewBox(){},
  performance:{now:()=>now},Date,AbortController,
  setTimeout:timeout,clearTimeout:id=>timers.delete(id),
  URL:{createObjectURL:()=>`blob:${++urlId}`,revokeObjectURL(){}},
  Image:class{naturalWidth=640;naturalHeight=360;set src(value){queueMicrotask(()=>this.onload());}},
  window:{addEventListener(name,fn){windowEvents[name]=fn;}},
  fetch(url,{signal}){return new Promise((resolve,reject)=>{
    signal.addEventListener('abort',()=>reject(new Error('aborted')),{once:true});
    const reply=replies.shift();if(!reply)return;
    timeout(()=>{
      if(reply.error)return reject(new Error(reply.error));
      resolve({ok:true,headers:{get:name=>reply.headers[name]??null},blob:async()=>({size:100})});
    },reply.delay);
  });}
});
function reply(delay=0,age=100,display=3000){return {delay,headers:{'X-Capture-Age-Ms':String(age),
  'X-Frame-Max-Age-Ms':'1000',...(display===null?{}:{'X-Preview-Max-Age-Ms':String(display)})}};}
async function flush(){for(let n=0;n<12;n++)await Promise.resolve();}
async function advance(ms){const end=now+ms;while(true){
  let first;for(const pair of timers)if(pair[1].at<=end&&(!first||pair[1].at<first[1].at))first=pair;
  if(!first)break;now=first[1].at;timers.delete(first[0]);first[1].fn();await flush();
}now=end;await flush();}
async function start(first){replies.push(first);vm.runInContext(input.client,context);await advance(first.delay);}
(async()=>{
  if(input.scenario==='slow'){
    await start(reply(1300));assert.ok(img.src);assert.equal(svg.style.visibility,'hidden');
    assert.match(status.textContent,/延迟/);await advance(1599);assert.ok(img.src);
    await advance(1);assert.equal(img.src,'');
  }else if(input.scenario==='expiry'){
    await start(reply(100));assert.ok(img.src);assert.equal(svg.style.visibility,'');
    await advance(800);assert.ok(img.src);assert.equal(svg.style.visibility,'hidden');
    await advance(2000);assert.equal(img.src,'');
  }else if(input.scenario==='retry'){
    await start(reply(100));const previous=img.src;
    replies.push({delay:100,error:'network'});await advance(200);
    assert.equal(img.src,previous);assert.match(status.textContent,/上一帧/);
    await advance(2600);assert.equal(img.src,'');
  }else if(input.scenario==='recovery'){
    await start(reply(1300));assert.equal(svg.style.visibility,'hidden');
    replies.push(reply(50,20));await advance(150);
    assert.ok(img.src);assert.equal(svg.style.visibility,'');assert.match(status.textContent,/已连接/);
  }else if(input.scenario==='fallback'){
    await start(reply(1300,100,null));assert.equal(img.src,'');
  }else if(input.scenario==='metadata'){
    const bad=reply(100);delete bad.headers['X-Capture-Age-Ms'];await start(bad);
    assert.equal(img.src,'');
  }else if(input.scenario==='visibility'){
    await start(reply(100));assert.ok(img.src);
    document.hidden=true;events.visibilitychange();assert.equal(img.src,'');
    replies.push(reply(50));document.hidden=false;events.visibilitychange();await advance(50);
    assert.ok(img.src);windowEvents.pagehide();assert.equal(img.src,'');
  }else if(input.scenario==='generation'){
    replies.push(reply(1000));vm.runInContext(input.client,context);await advance(100);
    replies.push(reply(50));button.click();await advance(50);const current=img.src;
    assert.ok(current);await advance(850);assert.equal(img.src,current);
  }else if(input.scenario==='longgap'){
    await start(reply(2200,200,5000));assert.ok(img.src);
    replies.push(reply(2200,200,5000));await advance(2299);assert.ok(img.src);
    await advance(1);assert.ok(img.src);assert.equal(svg.style.visibility,'hidden');
  }else throw new Error('unknown scenario');
  process.stdout.write('ok');
})().catch(error=>{process.stderr.write(error.stack);process.exitCode=1;});
"""


@unittest.skipUnless(shutil.which('node'), 'Node.js is needed only for deterministic preview JS checks')
class PreviewClientTests(unittest.TestCase):
    def test_shipped_client_network_and_lifecycle(self):
        for scenario in ('slow', 'expiry', 'retry', 'recovery', 'fallback', 'metadata', 'visibility', 'generation', 'longgap'):
            with self.subTest(scenario=scenario):
                result = subprocess.run(
                    [shutil.which('node'), '-e', HARNESS],
                    input=json.dumps({'client': FRAME_CLIENT_JS, 'scenario': scenario}),
                    text=True, capture_output=True, timeout=10,
                )
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(result.stdout, 'ok')
