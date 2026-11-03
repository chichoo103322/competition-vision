"""Round top-face image localization. No pixel-to-mechanical calibration.

Requires a sufficiently complete ellipse inside the color region. An uncertain
face never falls back to the whole-object centroid as a valid top-face point.
"""
from dataclasses import dataclass
import math
import cv2
import numpy as np


@dataclass(frozen=True)
class TopFace:
    center_x: float
    center_y: float
    axis_a: float
    axis_b: float
    angle_deg: float
    residual: float


def locate_top_face(frame, color_contour):
    x, y, w, h = cv2.boundingRect(color_contour)
    height, width = frame.shape[:2]
    if x <= 1 or y <= 1 or x+w >= width-1 or y+h >= height-1 or min(w, h) < 15:
        return None, 'color_region_clipped_or_small'
    # Small local crop bounds cost; ellipse and return values are full input pixels.
    x0, y0 = max(0, x-4), max(0, y-4)
    crop = frame[y0:min(height,y+h+4), x0:min(width,x+w+4)]
    mask = np.zeros(crop.shape[:2], np.uint8)
    local = color_contour-np.array([[[x0,y0]]])
    cv2.drawContours(mask, [local], -1, 255, -1)
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    contours = [local]
    edge_union = np.zeros(mask.shape, np.uint8)
    for channel in (gray, hsv[:,:,1], hsv[:,:,2]):
        blur = cv2.GaussianBlur(channel, (3,3), 0)
        for low, high in ((12,35),(30,80),(60,120)):
            edge = cv2.Canny(blur, low, high)
            edge_union |= edge
            contours.extend(cv2.findContours(edge, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)[0])
    values = hsv[:,:,2][mask>0]
    if len(values)>0 and float(np.std(values))>8:
        threshold,_ = cv2.threshold(values.reshape(-1,1),0,255,cv2.THRESH_BINARY|cv2.THRESH_OTSU)
        bright = np.where((hsv[:,:,2]>threshold)&(mask>0),255,0).astype(np.uint8)
        bright = cv2.morphologyEx(bright,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
        contours.extend(cv2.findContours(bright,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)[0])
    distance = cv2.distanceTransform(255-edge_union, cv2.DIST_L2, 3)
    area = cv2.contourArea(color_contour)
    candidates = []
    for contour in contours:
        if len(contour) < 12:
            continue
        a = cv2.contourArea(contour)
        if not .25*area <= a <= 1.08*area:
            continue
        try:
            (cx,cy),(aa,bb),angle = cv2.fitEllipse(contour)
        except cv2.error:
            continue
        if min(aa,bb) < 12 or max(aa,bb)/min(aa,bb) > 1.8:
            continue
        ellipse_area = math.pi*aa*bb/4
        if not .9 <= a/ellipse_area <= 1.08:
            continue
        theta = math.radians(angle)
        pts = contour.reshape(-1,2)-np.array([cx,cy])
        u = (pts[:,0]*math.cos(theta)+pts[:,1]*math.sin(theta))/(aa/2)
        v = (-pts[:,0]*math.sin(theta)+pts[:,1]*math.cos(theta))/(bb/2)
        residual = float(np.mean(np.abs(np.hypot(u,v)-1)))
        coverage = len(set(np.floor((np.arctan2(v,u)+math.pi)/(2*math.pi)*24).astype(int)))
        if residual > .035 or coverage < 21:
            continue
        # Most of the fitted face interior must be in the selected color region.
        region = np.zeros(mask.shape,np.uint8)
        cv2.ellipse(region, ((cx,cy),(aa*.94,bb*.94),angle), 255,-1)
        if np.count_nonzero((region>0)&(mask>0))/max(1,np.count_nonzero(region)) < .95:
            continue
        samples = np.linspace(0,2*math.pi,96,endpoint=False)
        ex=cx+aa/2*np.cos(samples)*math.cos(theta)-bb/2*np.sin(samples)*math.sin(theta)
        ey=cy+aa/2*np.cos(samples)*math.sin(theta)+bb/2*np.sin(samples)*math.cos(theta)
        ex=np.clip(np.rint(ex).astype(int),0,mask.shape[1]-1)
        ey=np.clip(np.rint(ey).astype(int),0,mask.shape[0]-1)
        if np.mean(distance[ey,ex]<=2.5)<.75:
            continue
        ratio=min(aa,bb)/max(aa,bb)
        # Prefer the bright round face over a larger silhouette enclosing a dark side.
        outer = np.zeros(mask.shape,np.uint8)
        cv2.ellipse(outer,((cx,cy),(aa*1.12,bb*1.12),angle),255,-1)
        exclusion = np.zeros(mask.shape,np.uint8)
        cv2.ellipse(exclusion,((cx,cy),(aa*1.02,bb*1.02),angle),255,-1)
        band=(outer>0)&(exclusion==0)&(mask>0)
        inner=(region>0)&(mask>0)
        contrast=0.
        if np.count_nonzero(band)>=ellipse_area*.025:
            contrast=float(np.median(hsv[:,:,2][inner]))-float(np.median(hsv[:,:,2][band]))
        quality=ratio-3*residual+min(.6,max(0.,contrast)/80.)
        candidates.append((quality,ellipse_area,TopFace(cx+x0,cy+y0,aa,bb,angle,residual)))
    if not candidates:
        return None, 'top_face_not_confirmed'
    candidates.sort(key=lambda c:(c[0],c[1]),reverse=True)
    best=candidates[0]
    # Reject distinct similarly strong ellipses; nested rims share one center.
    for quality,_,face in candidates[1:]:
        if (quality >= best[0]-.04 and math.hypot(face.center_x-best[2].center_x,
                face.center_y-best[2].center_y) > min(best[2].axis_a,best[2].axis_b)*.2):
            return None, 'top_face_ambiguous'
    return best[2], 'top_face_ellipse_confirmed'
