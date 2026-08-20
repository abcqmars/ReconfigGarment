from abc import abstractmethod
import math
import numpy as np
import turning_function

SEAM_AWARE_LAMBDA = 0.4


class PatchRepresentation:
    @abstractmethod
    def merge(self, patch):
        pass

    @abstractmethod
    def distance(self, patch):
        pass


class ApproxPolygon(PatchRepresentation):
    def __init__(self):
        self.verts2D = []

    def create_flippedCopy(self):
        self.flipped_copy = ApproxPolygon()
        center = self.center()
        new_verts = [[2 * center[0] - vert[0], vert[1]] for vert in self.verts2D]
        self.flipped_copy.verts2D = new_verts[::-1]
        self.flipped_copy.init_metric()
        self.flipped_copy.flipped_copy = self

    @property
    def is_CCW(self):
        assert len(self.verts2D) != 0
        return self.signed_area > 0

    def make_CCW(self):
        if not self.is_CCW:
            self.verts2D = self.verts2D[::-1]

    def init_metric(self):
        perim = 0
        ranges = [0]
        turning_angles = []
        self.make_CCW()
        edge_vec = np.array(self.verts2D[1]) - np.array(self.verts2D[0])
        init_angle = math.atan2(edge_vec[1], edge_vec[0])
        turning_angles.append(init_angle)

        for i in range(self.num_edge):
            vec0 = np.array(self.verts2D[i % self.num_edge]) - np.array(self.verts2D[(i - 1) % self.num_edge])
            vec1 = np.array(self.verts2D[(i + 1) % self.num_edge]) - np.array(self.verts2D[i % self.num_edge])
            turning_angle = math.atan2(np.cross(vec0, vec1), np.dot(vec0, vec1))
            perim += np.linalg.norm(vec1)
            ranges.append(perim)
            turning_angles.append(turning_angles[-1] + turning_angle)

        self.ranges = [v / perim for v in ranges]
        self.turning_angles = turning_angles

    def merge(self, patch):
        raise NotImplementedError

    @property
    def num_edge(self):
        return len(self.verts2D)

    def distance(self, patch):
        score = turning_function.distance(self.verts2D, patch.verts2D, brute_force_updates=False)[0]
        flipped_score = turning_function.distance(
            self.flipped_copy.verts2D, patch.verts2D, brute_force_updates=False
        )[0]
        scale_penalty = abs(np.log(self.area) - np.log(patch.area))
        return min(score, flipped_score) + scale_penalty

    def seamAwareDistance(self, patch, lambda_=SEAM_AWARE_LAMBDA):
        return self.distance(patch)

    def center(self):
        c = np.array(self.verts2D).mean(axis=0)
        return [c[0], c[1]]

    @property
    def area(self):
        return abs(self.signed_area)

    @property
    def signed_area(self):
        area = 0
        for i in range(self.num_edge):
            x1, y1 = self.verts2D[i]
            x2, y2 = self.verts2D[(i + 1) % self.num_edge]
            area += x1 * y2 - x2 * y1
        return area * 0.5

    @property
    def bbox(self):
        min_x = min(vert[0] for vert in self.verts2D)
        min_y = min(vert[1] for vert in self.verts2D)
        max_x = max(vert[0] for vert in self.verts2D)
        max_y = max(vert[1] for vert in self.verts2D)
        return [[min_x, min_y], [max_x, max_y]]


class Polygon(PatchRepresentation):
    def __init__(self):
        self.verts2D = []
        self.seam_range = []

    def seam_length(self, start_idx, end_idx):
        return abs(end_idx - start_idx) + 1

    @staticmethod
    def num_vertices_at_level(n, level):
        return max(2, math.ceil(n / (2 ** level)))

    @property
    def max_downsample_level(self):
        if not self.seam_range:
            n = len(self.verts2D)
            return max(0, math.ceil(math.log2(n)) - 1) if n > 2 else 0
        return max(
            max(0, math.ceil(math.log2(self.seam_length(s, e))) - 1)
            for s, e in self.seam_range
        )

    @staticmethod
    def _uniform_subsample_indices(start_idx, end_idx, num_keep):
        if start_idx <= end_idx:
            indices = list(range(start_idx, end_idx + 1))
        else:
            indices = list(range(start_idx, end_idx - 1, -1))

        n = len(indices)
        num_keep = max(2, min(num_keep, n))
        if num_keep >= n:
            return indices
        if num_keep == 2:
            return [indices[0], indices[-1]]

        positions = np.linspace(0, n - 1, num_keep)
        picked = []
        for p in positions:
            vid = indices[int(round(p))]
            if not picked or picked[-1] != vid:
                picked.append(vid)
        if picked[0] != indices[0]:
            picked[0] = indices[0]
        if picked[-1] != indices[-1]:
            picked[-1] = indices[-1]
        return picked

    def seam_vertex_indices_at_level(self, start_idx, end_idx, level):
        n = self.seam_length(start_idx, end_idx)
        num_keep = self.num_vertices_at_level(n, level)
        return self._uniform_subsample_indices(start_idx, end_idx, num_keep)

    def vertex_indices_at_level(self, level):
        level = max(0, min(level, self.max_downsample_level))
        if not self.seam_range:
            n = len(self.verts2D)
            num_keep = self.num_vertices_at_level(n, level)
            return self._uniform_subsample_indices(0, n - 1, num_keep)

        boundary = []
        for start_idx, end_idx in self.seam_range:
            seam_indices = self.seam_vertex_indices_at_level(start_idx, end_idx, level)
            if boundary and seam_indices[0] == boundary[-1]:
                seam_indices = seam_indices[1:]
            boundary.extend(seam_indices)
        return boundary

    def verts_at_level(self, level):
        return [self.verts2D[i] for i in self.vertex_indices_at_level(level)]

    def flipped_verts_at_level(self, level):
        center = self.center()
        return [[2 * center[0] - vert[0], vert[1]] for vert in reversed(self.verts_at_level(level))]

    def create_flippedCopy(self):
        self.flipped_copy = ApproxPolygon()
        center = self.center()
        new_verts = [[2 * center[0] - vert[0], vert[1]] for vert in self.verts2D]
        self.flipped_copy.verts2D = new_verts[::-1]
        self.flipped_copy.init_metric()
        self.flipped_copy.flipped_copy = self

    @property
    def is_CCW(self):
        assert len(self.verts2D) != 0
        return self.signed_area > 0

    def make_CCW(self):
        if self.is_CCW:
            return
        n = len(self.verts2D)
        self.verts2D = self.verts2D[::-1]
        if self.seam_range:
            self.seam_range = [
                [n - 1 - end_idx, n - 1 - start_idx]
                for start_idx, end_idx in reversed(self.seam_range)
            ]

    def merge(self, patch):
        raise NotImplementedError

    @property
    def num_edge(self):
        return len(self.verts2D)

    def _turning_distance_at_level(self, patch, level):
        src_verts = self.verts_at_level(level)
        src_verts_flipped = self.flipped_verts_at_level(level)
        tgt_verts = patch.verts_at_level(level)
        score = turning_function.distance(src_verts, tgt_verts, brute_force_updates=False)[0]
        flipped_score = turning_function.distance(
            src_verts_flipped, tgt_verts, brute_force_updates=False
        )[0]
        scale_penalty = abs(np.log(self.area) - np.log(patch.area))
        return min(score, flipped_score) + scale_penalty

    def distance(self, patch):
        return self._turning_distance_at_level(patch, 100)

    def seamAwareDistance(self, patch, lambda_=SEAM_AWARE_LAMBDA):
        max_level = 0
        min_level = self.max_downsample_level
        d_max = self._turning_distance_at_level(patch, max_level)
        d_min = self._turning_distance_at_level(patch, min_level)
        return lambda_ * d_max + (1.0 - lambda_) * d_min

    def center(self):
        c = np.array(self.verts2D).mean(axis=0)
        return [c[0], c[1]]

    @property
    def area(self):
        return abs(self.signed_area)

    @property
    def signed_area(self):
        area = 0
        for i in range(self.num_edge):
            x1, y1 = self.verts2D[i]
            x2, y2 = self.verts2D[(i + 1) % self.num_edge]
            area += x1 * y2 - x2 * y1
        return area * 0.5

    @property
    def bbox(self):
        min_x = min(vert[0] for vert in self.verts2D)
        min_y = min(vert[1] for vert in self.verts2D)
        max_x = max(vert[0] for vert in self.verts2D)
        max_y = max(vert[1] for vert in self.verts2D)
        return [[min_x, min_y], [max_x, max_y]]
