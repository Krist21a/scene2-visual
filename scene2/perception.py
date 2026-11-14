"""Colour/geometry observations from calibrated RGB-D, in robot base metres."""
from dataclasses import dataclass, asdict
import cv2
import numpy as np


@dataclass
class Part:
    kind: str
    center: list
    axis: list
    extent: list
    top: float
    pixels: list
    stamp: float
    confidence: float
    full_center: list = None
    full_extent: list = None
    footprint: list = None
    occupied_xy: list = None

    @property
    def handle_length(self):
        """Observed full handle support, independent of grasp-percentile size."""
        length=float(self.extent[0])
        if self.kind=='C' and self.occupied_xy:
            length=max(length,float(np.ptp(np.asarray(self.occupied_xy)@self.axis)))
        return length


@dataclass
class Bin:
    kind: str
    center: list
    lower: list
    upper: list
    floor: float
    rim: float
    stamp: float
    cavity: list = None

    def contains(self, xy, margin=0.):
        if self.cavity is not None:
            polygon=np.asarray(self.cavity,dtype=float)
            edges=np.roll(polygon,-1,axis=0)-polygon
            inward=np.column_stack((-edges[:,1],edges[:,0]))
            inward/=np.linalg.norm(inward,axis=1)[:,None]
            return bool(np.all(np.sum((np.asarray(xy)-polygon)*inward,axis=1)>=margin))
        return bool(np.all(np.asarray(xy) >= np.asarray(self.lower)+margin)
                    and np.all(np.asarray(xy) <= np.asarray(self.upper)-margin))


def point_image(snapshot):
    depth, k, transform = (np.asarray(snapshot[key]) for key in
                            ('depth_m', 'intrinsic', 'camera_to_base'))
    v, u = np.indices(depth.shape)
    camera = np.stack(((u-k[0,2])*depth/k[0,0],
                       (v-k[1,2])*depth/k[1,1], depth), axis=-1)
    points = camera @ transform[:3,:3].T + transform[:3,3]
    valid = np.isfinite(points).all(axis=-1) & (depth > .1) & (depth < 3.)
    valid &= (points[:,:,0] > .1) & (points[:,:,0] < 1.2) & (np.abs(points[:,:,1]) < .8)
    return points, valid


def components(mask, minimum=12):
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), 8)
    for i in range(1, count):
        if stats[i, cv2.CC_STAT_AREA] >= minimum:
            yield labels == i


def masks(rgb):
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    h,s,v = cv2.split(hsv)
    return {
        'A': (s < 65) & (v > 125),
        'B': (s < 75) & (v >= 22) & (v < 115),
        'C': ((h < 12) | (h > 170)) & (s > 125) & (v > 75),
    }, {
        'A': (h >= 98) & (h < 123) & (s > 120) & (v > 45),
        'B': (h >= 12) & (h < 32) & (s > 130) & (v > 45),
        'C': (h >= 125) & (h < 165) & (s > 100) & (v > 45),
    }


class Detector:
    def detect(self, snapshot, include_bins=False):
        rgb = snapshot['rgb']
        points, valid = point_image(snapshot)
        table = float(snapshot['table_z_base_m'])
        stamp = float(snapshot['stamp'])
        part_masks, bin_masks = masks(rgb)
        hsv=cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        value = hsv[:,:,2]
        bins = {}
        for kind, colour in bin_masks.items():
            # Translucent bin geoms are absent from the simulated depth image.
            # Use the bright floor polygon and ray-plane intersection at the
            # tabletop + public 10mm floor thickness, not the false depth wall.
            candidates = list(components(colour & (value > 200) & valid, 100))
            if not candidates:
                continue
            # A placed shaft can split one floor into disconnected visible
            # regions. Bin colours are unique; retain every supported region.
            region = np.logical_or.reduce(candidates)
            floor = table+.010
            camera = np.asarray(snapshot['camera_to_base'])[:3,3]
            rays = points[region]-camera
            scale = (floor-camera[2])/rays[:,2]
            bottom = camera+rays*scale[:,None]
            if len(bottom) < 80:
                continue
            lo, hi = np.percentile(bottom[:,:2], [1,99], axis=0)
            if np.any(hi-lo < .09) or np.any(hi-lo > .35):
                continue
            bins[kind] = Bin(kind, ((lo+hi)/2).tolist(), lo.tolist(), hi.tolist(),
                             floor, table+.160, stamp)
            if include_bins and kind=='C':
                from .bin_geometry import cavity_bounds
                bins[kind]=cavity_bounds(snapshot,colour,bins[kind])
        parts = []
        surface = valid & (points[:,:,2] > table+.004) & (points[:,:,2] < table+.20)
        bin_region=np.zeros(valid.shape,dtype=bool)
        if include_bins:
            for b in bins.values():
                region=valid&(points[:,:,2]>b.floor+.003)&(points[:,:,2]<b.rim+.02)
                for j in (0,1):
                    region&=(points[:,:,j]>b.lower[j]-.02)&(points[:,:,j]<b.upper[j]+.02)
                bin_region|=region
        for kind, colour in part_masks.items():
            if kind=='A' and include_bins:
                # Include neutral shaded faces for settled, possibly upright
                # brackets. Classification below still requires white support
                # and the complete 3-D shape family.
                colour=colour|(bin_region&(hsv[:,:,1]<75)&(value>40))
            mask = (colour & surface).astype(np.uint8)
            # Bridge mesh holes, but never join separate objects across centimetres.
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3,3),np.uint8))
            for region in components(mask, 15):
                region &= valid & surface
                p = points[region]
                if len(p) < 15:
                    continue
                center = np.median(p, axis=0)
                settled_bin=next((b for b in bins.values() if b.contains(center[:2],-.02)),None) if include_bins else None
                if not include_bins and any(b.contains(center[:2], -.025) for b in bins.values()):
                    continue
                _, eigenvectors = np.linalg.eigh(np.cov(p[:,:2].T))
                axis = eigenvectors[:,-1]
                basis = np.column_stack((axis, [-axis[1],axis[0]]))
                lo, hi = np.percentile(p[:,:2]@basis, [2,98], axis=0)
                extent = hi-lo
                center[:2] = ((lo+hi)/2)@basis.T
                settled_c=kind=='C' and settled_bin is not None
                if not settled_c and not (.018 <= extent[0] <= .14 and .008 <= extent[1] <= .085):
                    continue
                if kind == 'C' and not settled_c and extent[0]/extent[1] < 1.8:
                    continue
                height=top_height(p,table)
                # Table objects have known shape families; raised fingers are
                # not parts. Partial mesh fragments cannot supply a grasp pose.
                if kind=='A':
                    if settled_bin is None:
                        # Depth discontinuities can broaden a thin C shaft in
                        # metric space; its colour silhouette remains slender.
                        py,px=np.nonzero(region&part_masks['A'])
                        pixels=np.column_stack((px,py))
                        _,image_axes=np.linalg.eigh(np.cov(pixels.T))
                        image_extent=np.diff(np.percentile(pixels@image_axes,[2,98],axis=0),axis=0)[0]
                        if image_extent[1]>7*max(image_extent[0],1.):continue
                    if settled_bin is not None:
                        _,basis3=np.linalg.eigh(np.cov(p.T))
                        shape=np.diff(np.percentile(p@basis3,[1,99],axis=0),axis=0)[0]
                        if not (.055<shape[2]<.115 and .025<shape[1]<.065 and .005<shape[0]<.035
                                and np.count_nonzero(part_masks['A']&region)>=max(15,len(p)*.15)):
                            continue
                    elif not (.055<extent[0]<.115 and .025<extent[1]<.065 and .010<height<.045):
                        continue
                if kind=='B':
                    if settled_bin is not None:
                        _,basis3=np.linalg.eigh(np.cov(p.T))
                        shape=np.diff(np.percentile(p@basis3,[1,99],axis=0),axis=0)[0]
                        # A tilted B can expose mostly one face. Require a
                        # substantial raised, vertically extended dark surface
                        # for that case, rather than assuming hidden thickness.
                        side_face=(len(p)>=60 and .004<shape[0]<.025 and height>.050
                                   and np.ptp(np.percentile(p[:,2],[1,99]))>.025)
                        if not ((.025<shape[0]<.060 or side_face) and .025<shape[1]<.075 and
                                .040<shape[2]<.085 and .025<height<.080):continue
                    elif not (.035<extent[0]<.080 and .025<extent[1]<.075 and .025<height<.070):
                        continue
                if kind=='C':
                    if settled_c:
                        _,handle_basis=np.linalg.eigh(np.cov(p.T))
                        handle_shape=np.diff(np.percentile(p@handle_basis,[1,99],axis=0),axis=0)[0]
                        if not (.055<handle_shape[2]<.115 and .010<handle_shape[1]<.04 and .004<handle_shape[0]<.03):continue
                    elif not (.06<extent[0]<.115 and .010<height<.045):continue
                # The red handle is the grasp region. Metal shaft is excluded
                # from classification, but included conservatively in motion bounds.
                v,u = np.nonzero(region)
                if u.min() < 3 or v.min() < 3 or u.max() > rgb.shape[1]-4 or v.max() > rgb.shape[0]-4:
                    continue
                top = float(np.percentile(p[:,2],95))
                center[2] = table + (top-table)/2
                if settled_bin is not None:center[2]=(np.percentile(p[:,2],1)+top)/2
                full_center,full_extent = center.copy(),extent.copy()
                entire=p
                if kind=='C':
                    delta = points[:,:,:2]-center[:2]
                    along = delta@axis
                    across = np.abs(delta@np.array([-axis[1],axis[0]]))
                    if settled_c:
                        handle_center=np.median(p,axis=0);axis3=handle_basis[:,-1]
                        delta3=points-handle_center;along3=delta3@axis3
                        across3=np.linalg.norm(delta3-along3[:,:,None]*axis3,axis=-1)
                        shaft=(part_masks['A']&valid&(across3<.014)&(np.abs(along3)<.20)
                               &(np.abs(along3)>handle_shape[2]/2)
                               &(points[:,:,2]>settled_bin.floor+.002)&(points[:,:,2]<settled_bin.rim+.12))
                        if shaft.sum()<8:continue
                        sign=np.sign(np.median(along3[shaft]));shaft&=along3*sign>0
                    else:
                        shaft = part_masks['A'] & valid & (across<.014) & (np.abs(along)<.20)
                        shaft &= (points[:,:,2]>table+.003) & (points[:,:,2]<table+.04)
                        shaft &= np.abs(along)>extent[0]/2
                    if shaft.sum()<8:
                        continue
                    entire=np.vstack((p,points[shaft]))
                    if settled_c:
                        full_shape=np.diff(np.percentile(entire@handle_basis,[1,99],axis=0),axis=0)[0]
                        if not (.13<full_shape[2]<.28 and np.all(full_shape[:2]<.045)):continue
                    # The grasp handle and thin shaft have very different
                    # point densities. Trimming their combined distribution
                    # can discard a supported tip by centimetres. Motion
                    # bounds must enclose the same observed footprint used
                    # by final verification.
                    projected=entire[:,:2]@basis
                    full_lo,full_hi=projected.min(0),projected.max(0)
                    full_extent=full_hi-full_lo
                    full_center[:2]=((full_hi+full_lo)/2)@basis.T
                    if not ((.025 if settled_c else .13)<full_extent[0]<.28):
                        continue
                parts.append(Part(kind, center.tolist(), axis.tolist(), extent.tolist(), top,
                                  [int(u.min()),int(v.min()),int(u.max()),int(v.max())],
                                  stamp, min(1.,len(p)/60),full_center.tolist(),full_extent.tolist(),
                                  cv2.convexHull(entire[:,:2].astype(np.float32)).reshape(-1,2).tolist(),
                                  (np.unique(np.round(p[:,:2]/.002),axis=0)*.002).tolist() if kind in ('A','C') else None))
        if include_bins and 'C' in bins:
            from .settled_pair import screwdriver_pair
            pair=screwdriver_pair(snapshot,bins['C'])
            if pair:
                parts=[p for p in parts if not (p.kind=='C' and bins['C'].contains(p.center[:2],-.025))]+pair
        return parts, bins

    @staticmethod
    def annotate(snapshot, parts, bins):
        rgb = snapshot['rgb'].copy()
        for p in parts:
            x,y,x2,y2 = p.pixels
            cv2.rectangle(rgb,(x,y),(x2,y2),(255,255,0),1)
            cv2.putText(rgb,p.kind,(x,y-4),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,0),1)
        return rgb


def serialise(parts, bins):
    return {'parts':[asdict(p) for p in parts], 'bins':{k:asdict(v) for k,v in bins.items()}}


def top_height(points, table):
    return float(np.percentile(points[:,2],95)-table)
