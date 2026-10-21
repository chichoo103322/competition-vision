"""完整预览的有限请求视频客户端：避免长连接失效后永久黑屏。"""

FRAME_CLIENT_JS = r"""
const videoStatus=document.getElementById('video-status');
const reconnectButton=document.getElementById('reconnect-video');
let frameTimer=null,frameController=null,frameGeneration=0,videoUrl=null,videoStopped=false;
function videoMessage(message){videoStatus.textContent=message;}
async function loadLatestFrame(){
  if(videoStopped||document.hidden)return;
  const generation=frameGeneration,controller=new AbortController();
  frameController=controller;
  const requestStarted=performance.now();
  const deadline=setTimeout(()=>controller.abort(),6000);
  let pendingUrl=null,nextDelay=200;
  try{
    const response=await fetch('/frame.jpg?t='+Date.now(),{cache:'no-store',signal:controller.signal});
    if(!response.ok)throw new Error('HTTP '+response.status);
    const blob=await response.blob();
    if(!blob.size)throw new Error('empty image');
    pendingUrl=URL.createObjectURL(blob);
    const decoded=new Image();
    await new Promise((resolve,reject)=>{decoded.onload=resolve;decoded.onerror=reject;decoded.src=pendingUrl;});
    if(generation!==frameGeneration||videoStopped||document.hidden)return;
    const previousUrl=videoUrl;
    img.src=pendingUrl;videoUrl=pendingUrl;pendingUrl=null;
    if(previousUrl)URL.revokeObjectURL(previousUrl);
    const age=Number(response.headers.get('X-Capture-Age-Ms')||0);
    sourceWidth=Number(response.headers.get('X-Source-Width'))||decoded.naturalWidth;
    sourceHeight=Number(response.headers.get('X-Source-Height'))||decoded.naturalHeight;
    videoMessage(age>2000?'相机画面超过 2 秒未更新，请检查相机':'画面已连接 · 预览 '+decoded.naturalWidth+'×'+decoded.naturalHeight);
    applyViewBox();
    nextDelay=Math.max(200,Math.min(1500,Math.round((performance.now()-requestStarted)*0.5)));
  }catch(error){
    if(generation===frameGeneration&&!videoStopped){
      videoMessage('预览传输超时或失败，正在重试…');nextDelay=1000;
    }
  }finally{
    clearTimeout(deadline);
    if(pendingUrl)URL.revokeObjectURL(pendingUrl);
    if(frameController===controller)frameController=null;
    if(generation===frameGeneration&&!videoStopped&&!document.hidden)
      frameTimer=setTimeout(loadLatestFrame,nextDelay);
  }
}
function reconnectVideo(){
  frameGeneration++;clearTimeout(frameTimer);
  if(frameController)frameController.abort();
  videoStopped=false;videoMessage('正在连接相机画面…');
  if(!document.hidden)loadLatestFrame();
}
reconnectButton.addEventListener('click',reconnectVideo);
document.addEventListener('visibilitychange',()=>{
  if(document.hidden){frameGeneration++;clearTimeout(frameTimer);if(frameController)frameController.abort();}
  else reconnectVideo();
});
window.addEventListener('pagehide',()=>{
  videoStopped=true;frameGeneration++;clearTimeout(frameTimer);
  if(frameController)frameController.abort();
  if(videoUrl){URL.revokeObjectURL(videoUrl);videoUrl=null;}
});
window.addEventListener('pageshow',()=>{if(videoStopped)reconnectVideo();});
reconnectVideo();
"""
