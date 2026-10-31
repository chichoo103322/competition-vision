"""完整预览的有限请求视频客户端：避免长连接失效后永久黑屏。"""

FRAME_CLIENT_JS = r"""
const videoStatus=document.getElementById('video-status');
const reconnectButton=document.getElementById('reconnect-video');
let frameTimer=null,frameExpiryTimer=null,frameFreshnessTimer=null,frameController=null,frameGeneration=0,videoUrl=null,videoStopped=false,displayDeadline=0;
function videoMessage(message){videoStatus.textContent=message;}
function clearPreview(){
  clearTimeout(frameExpiryTimer);clearTimeout(frameFreshnessTimer);
  displayDeadline=0;img.removeAttribute('src');svg.style.visibility='hidden';
  if(videoUrl){URL.revokeObjectURL(videoUrl);videoUrl=null;}
}
function delayedPreview(message){clearTimeout(frameFreshnessTimer);svg.style.visibility='hidden';videoMessage(message);}
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
    const ageHeader=response.headers.get('X-Capture-Age-Ms');
    const age=Number(ageHeader);
    const controlMaxAge=Number(response.headers.get('X-Frame-Max-Age-Ms'));
    const displayMaxAge=Number(response.headers.get('X-Preview-Max-Age-Ms')||controlMaxAge);
    const elapsed=performance.now()-requestStarted,estimatedAge=age+elapsed;
    const remain=displayMaxAge-estimatedAge,controlRemain=controlMaxAge-estimatedAge;
    if(ageHeader===null||!Number.isFinite(age)||age<0||!Number.isFinite(controlMaxAge)||controlMaxAge<=0||!Number.isFinite(remain)||remain<=0)
      throw new Error('expired or unknown image age');
    const previousUrl=videoUrl;
    img.src=pendingUrl;videoUrl=pendingUrl;pendingUrl=null;
    if(previousUrl)URL.revokeObjectURL(previousUrl);
    clearTimeout(frameExpiryTimer);clearTimeout(frameFreshnessTimer);
    displayDeadline=performance.now()+remain;
    frameExpiryTimer=setTimeout(()=>{clearPreview();videoMessage('预览已过期，等待新画面…');},remain);
    sourceWidth=Number(response.headers.get('X-Source-Width'))||decoded.naturalWidth;
    sourceHeight=Number(response.headers.get('X-Source-Height'))||decoded.naturalHeight;
    if(controlRemain>0){
      svg.style.visibility='';
      videoMessage('画面已连接 · 预览 '+decoded.naturalWidth+'×'+decoded.naturalHeight+' · 仅供查看');
      frameFreshnessTimer=setTimeout(()=>delayedPreview('预览有延迟，仅供查看；图上标记已隐藏'),controlRemain);
    }else delayedPreview('预览有延迟，仅供查看；图上标记已隐藏');
    applyViewBox();
    nextDelay=100;
  }catch(error){
    if(generation===frameGeneration&&!videoStopped){
      if(videoUrl&&performance.now()<displayDeadline)
        delayedPreview('预览连接波动，暂显示上一帧（仅供查看），正在重试…');
      else{clearPreview();videoMessage('预览传输超时或失败，正在重试…');}
      nextDelay=200;
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
  if(document.hidden){frameGeneration++;clearTimeout(frameTimer);clearPreview();if(frameController)frameController.abort();}
  else reconnectVideo();
});
window.addEventListener('pagehide',()=>{
  videoStopped=true;frameGeneration++;clearTimeout(frameTimer);
  clearPreview();
  if(frameController)frameController.abort();
});
window.addEventListener('pageshow',()=>{if(videoStopped)reconnectVideo();});
reconnectVideo();
"""
