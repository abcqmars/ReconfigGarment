import numpy as np
from patchRep import ApproxPolygon, Polygon


class Segment:
    def __init__(self):
        self.patches = []
        self._s2pvec = []

    def add_patch(self, patch):
        self.patches.append(patch)

    def distance(self, other):
        dist = 0
        for p1 in self.patches:
            for p2 in other.patches:
                dist += p1.distance(p2)
        return dist

    def get_poly(self):
        assert len(self.patches) == 1
        return np.array(self.patches[0].verts_at_level(100))

    @property
    def num_patch(self):
        return sum(self._s2pvec)

    @property
    def bbox(self):
        min_x = min(patch.bbox[0][0] for patch in self.patches)
        min_y = min(patch.bbox[0][1] for patch in self.patches)
        max_x = max(patch.bbox[1][0] for patch in self.patches)
        max_y = max(patch.bbox[1][1] for patch in self.patches)
        return [[min_x, min_y], [max_x, max_y]]


class Garment:
    def __init__(self):
        self._segments = []
        self._pataches = []
        self._p2smat = []

    def num_segments(self):
        return len(self._segments)

    def num_patches(self):
        return len(self._pataches)

    def get_p2smat(self):
        assert self._p2smat != []
        assert len(self._p2smat) == self.num_patches()
        assert all(len(row) == self.num_segments() for row in self._p2smat)
        return self._p2smat

    def get_segments(self):
        assert self._segments
        return self._segments

    def build_from_txt(self, txt_path, type=Polygon):
        if type is ApproxPolygon:
            self._p2smat = []
            with open(txt_path, "r") as f:
                num_patches = int(f.readline().strip())
                num_segments = int(f.readline().strip())

                for _ in range(num_segments):
                    seg = Segment()
                    patch = ApproxPolygon()
                    paids = [int(i) for i in f.readline().strip().split()]
                    n = int(f.readline().strip())
                    for _ in range(n):
                        x, y = map(float, f.readline().split())
                        patch.verts2D.append((x, y))

                    patch.init_metric()
                    patch.create_flippedCopy()
                    seg.patches.append(patch)
                    seg._s2pvec = [0] * num_patches
                    for i in paids:
                        seg._s2pvec[i] = 1

                    self._pataches = [None] * num_patches
                    self._segments.append(seg)
                    self._p2smat.append(seg._s2pvec)

                self._p2smat = [list(row) for row in zip(*self._p2smat)]
            return

        if type is Polygon:
            with open(txt_path, "r") as f:
                num_patches = int(f.readline().strip())
                num_segments = int(f.readline().strip())

                for _ in range(num_segments):
                    seg = Segment()
                    patch = Polygon()
                    paids = [int(i) for i in f.readline().strip().split()]
                    num_seams = int(f.readline().strip())
                    for _ in range(num_seams):
                        patch.seam_range.append([int(i) for i in f.readline().strip().split()])

                    num_verts = patch.seam_range[-1][1] + 1
                    all_verts = []
                    for _ in range(num_verts):
                        x, y = map(float, f.readline().split())
                        all_verts.append((x, y))

                    patch.verts2D = all_verts
                    patch.make_CCW()
                    patch.create_flippedCopy()
                    seg.patches.append(patch)
                    seg._s2pvec = [0] * num_patches
                    for i in paids:
                        seg._s2pvec[i] = 1

                    self._pataches = [None] * num_patches
                    self._segments.append(seg)
                    self._p2smat.append(seg._s2pvec)

                self._p2smat = [list(row) for row in zip(*self._p2smat)]
            return

        raise ValueError(f"Unsupported garment type: {type!r}")
