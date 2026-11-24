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


def axial_rim(frame, contour):
    """Verify the end rim on an elongated cylinder in the dedicated raw-camera view."""
    major, minor = sorted(cv2.minAreaRect(contour)[1], reverse=True)
    if minor < 40 or not 1.22 <= major / minor <= 1.85:
        return None
    height, width = frame.shape[:2]
    x, y, w, h = cv2.boundingRect(contour)
    if x < 3 or y < 3 or x+w > width-3 or y+h > height-3:
        return None
    pad = math.ceil(.12*minor)
    x0, y0 = max(x-pad, 0), max(y-pad, 0)
    crop = frame[y0:y+h+pad, x0:x+w+pad]
    mask = np.zeros(crop.shape[:2], np.uint8)
    cv2.drawContours(mask, [contour-np.int32([x0,y0])], -1, 255, -1)
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    gray = cv2.createCLAHE(2., (4,4)).apply(cv2.GaussianBlur(gray, (7,7), 0))
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.4, minor*.22,
        param1=40, param2=35, minRadius=int(minor*.41), maxRadius=int(minor*.55))
    if circles is None:
        return None
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    edges = np.zeros_like(gray)
    for channel in (gray, hsv[:,:,1], hsv[:,:,2]):
        edges |= cv2.Canny(cv2.GaussianBlur(channel, (5,5), 0), 10, 25)
    # Sample independent edge pixels around the proposed circle. Require
    # evidence on the internal seam as well as the visible outer silhouette.
    angles = np.linspace(0, 2*math.pi, 72, endpoint=False)
    ux, uy = np.cos(angles), np.sin(angles)
    moments = cv2.moments(contour)
    if not moments['m00']:
        return None
    body = np.array([moments['m10'], moments['m01']]) / moments['m00']
    radial = body - [width/2, height/2]
    candidates = []
    for cx, cy, radius in circles[0]:
        offset = np.array([cx+x0, cy+y0]) - body
        # For this overhead camera the projected raised end lies outward
        # from the optical centre. Do not assign an end if direction is weak.
        if (np.linalg.norm(offset) < .08*minor or np.linalg.norm(radial) < .3*minor
                or offset @ radial < .70*np.linalg.norm(offset)*np.linalg.norm(radial)):
            continue
        radii = radius + np.arange(-int(.075*radius), int(.075*radius)+1)
        px = np.rint(cx + ux[:,None]*radii).astype(int)
        py = np.rint(cy + uy[:,None]*radii).astype(int)
        if px.min()<0 or py.min()<0 or px.max()>=mask.shape[1] or py.max()>=mask.shape[0]:
            continue
        hits = edges[py,px] != 0
        found = hits.any(axis=1)
        inward = ux*offset[0] + uy*offset[1] < -.5*np.linalg.norm(offset)
        if found.mean() < .85 or found[inward].mean() < .80:
            continue
        nearest = np.where(hits, abs(radii-radius), np.inf).argmin(axis=1)
        points = np.column_stack((px[np.arange(72),nearest],py[np.arange(72),nearest]))[found]
        (ex,ey),(aa,bb),angle = cv2.fitEllipse(points.astype(np.float32))
        if min(aa,bb) < .83*minor or max(aa,bb) > 1.14*minor or math.hypot(ex-cx,ey-cy) > .055*minor:
            continue
        interior = np.zeros_like(mask)
        cv2.ellipse(interior, ((ex,ey),(aa*.92,bb*.92),angle), 255,-1)
        if np.mean(mask[interior>0]>0) < .97:
            continue
        theta = math.radians(angle)
        dx, dy = points[:,0]-ex, points[:,1]-ey
        residual = np.abs(np.hypot((dx*np.cos(theta)+dy*np.sin(theta))/(aa/2),
                                   (-dx*np.sin(theta)+dy*np.cos(theta))/(bb/2))-1).mean()
        if residual <= .035:
            candidates.append(TopFace(ex+x0, ey+y0, aa, bb, angle, float(residual)))
    return candidates[0] if len(candidates)==1 else None


def locate_top_face(frame, color_contour):
    x, y, w, h = cv2.boundingRect(color_contour)
    height, width = frame.shape[:2]
    body_clipped = x <= 1 or y <= 1 or x+w >= width-1 or y+h >= height-1
    if min(w, h) < 15:
        return None, 'color_region_clipped_or_small'
    # Small local crop bounds cost; ellipse and return values are full input pixels.
    x0, y0 = max(0, x-4), max(0, y-4)
    crop = frame[y0:min(height,y+h+4), x0:min(width,x+w+4)]
    mask = np.zeros(crop.shape[:2], np.uint8)
    local = color_contour-np.array([[[x0,y0]]])
    cv2.drawContours(mask, [local], -1, 255, -1)
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    contours = [] if body_clipped else [local]
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
    # Yellow cylinders at the edge of the disc show little brightness contrast
    # between their end and side. In that view the end forms a distinct, more
    # saturated circular patch. Validate its rim with the same ellipse checks.
    yellow = (hsv[:, :, 0] >= 18) & (hsv[:, :, 0] <= 40) & (mask > 0)
    saturation_contours = []
    if np.count_nonzero(yellow) > .85 * np.count_nonzero(mask):
        saturation = cv2.GaussianBlur(hsv[:, :, 1], (5, 5), 0)
        sat_values = saturation[mask > 0]
        low, high = np.percentile(sat_values, [15, 85])
        if high - low >= 18:
            cut, _ = cv2.threshold(sat_values.reshape(-1, 1), 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
            patch = np.uint8((saturation > cut + 4) & (mask > 0)) * 255
            patch = cv2.morphologyEx(patch, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
            saturation_contours = cv2.findContours(patch, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0]
    distance = cv2.distanceTransform(255-edge_union, cv2.DIST_L2, 3)
    area = cv2.contourArea(color_contour)
    candidates = []
    for idx, contour in enumerate([*contours, *saturation_contours]):
        sat_patch = idx >= len(contours)
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
        if sat_patch and (max(aa, bb) > 1.3 * min(aa, bb) or a > .88 * area):
            continue
        if body_clipped:
            # Require the entire fitted rim and its supporting contour inside the
            # original frame. Never treat the cut edge as part of a circular rim.
            theta = math.radians(angle)
            rx = math.hypot(aa * math.cos(theta), bb * math.sin(theta)) / 2
            ry = math.hypot(aa * math.sin(theta), bb * math.cos(theta)) / 2
            bx, by, bw, bh = cv2.boundingRect(contour)
            if min(cx+x0-rx, cy+y0-ry, width-1-cx-x0-rx, height-1-cy-y0-ry,
                   bx+x0, by+y0, width-bx-x0-bw, height-by-y0-bh) <= 2:
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
        edge_support = distance[ey,ex] <= 2.5
        if sat_patch:
            # A diffuse saturation rim can fall between Canny edges. Check the
            # actual colour change across its fitted normal on the source image.
            offset = max(2., min(aa, bb) * .045)
            dx, dy = ex - cx, ey - cy
            length = np.maximum(1., np.hypot(dx, dy))
            def rim_saturation(sign):
                ix = np.clip(np.rint(ex + sign * offset * dx / length).astype(int), 0, mask.shape[1]-1)
                iy = np.clip(np.rint(ey + sign * offset * dy / length).astype(int), 0, mask.shape[0]-1)
                return saturation[iy, ix].astype(float)
            edge_support |= rim_saturation(-1) - rim_saturation(1) >= 10
        if np.mean(edge_support) < .75:
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
        if sat_patch:
            if np.count_nonzero(band) < .025 * ellipse_area:
                continue
            sat_contrast = float(np.median(saturation[inner])) - float(np.median(saturation[band]))
            if sat_contrast < 18:
                continue
            quality += min(.6, sat_contrast / 80.)
        candidates.append((quality,ellipse_area,TopFace(cx+x0,cy+y0,aa,bb,angle,residual)))
    if not candidates:
        return None, 'clipped_region_no_complete_rim' if body_clipped else 'top_face_not_confirmed'
    candidates.sort(key=lambda c:(c[0],c[1]),reverse=True)
    best=candidates[0]
    # Reject distinct similarly strong ellipses; nested rims share one center.
    for quality,_,face in candidates[1:]:
        if (quality >= best[0]-.04 and math.hypot(face.center_x-best[2].center_x,
                face.center_y-best[2].center_y) > min(best[2].axis_a,best[2].axis_b)*.2):
            return None, 'top_face_ambiguous'
    return best[2], 'top_face_ellipse_confirmed'
