from ortools.linear_solver import pywraplp


class mipGarment_solver:
    def __init__(self, garment1, garment2, solvertype="SAT"):
        self.solver = pywraplp.Solver.CreateSolver(solvertype)
        assert self.solver, f"Initialize {solvertype} solver failed."
        self.garment1 = garment1
        self.garment2 = garment2
        self.xs1 = None
        self.xs2 = None
        self.map = None
        self.distMat = None
        self.status = None

    def precompute_distMat(self):
        self.distMat = [
            [0 for _ in range(self.garment2.num_segments())]
            for _ in range(self.garment1.num_segments())
        ]
        for i, s1 in enumerate(self.garment1.get_segments()):
            for j, s2 in enumerate(self.garment2.get_segments()):
                self.distMat[i][j] = s1.distance(s2)
        return True

    def create_variables(self):
        self.xs1 = [None] * self.garment1.num_segments()
        self.xs2 = [None] * self.garment2.num_segments()
        self.map = [
            [None for _ in range(self.garment2.num_segments())]
            for _ in range(self.garment1.num_segments())
        ]

        for i in range(self.garment1.num_segments()):
            self.xs1[i] = self.solver.BoolVar("")
        for i in range(self.garment2.num_segments()):
            self.xs2[i] = self.solver.BoolVar("")
        for i in range(self.garment1.num_segments()):
            for j in range(self.garment2.num_segments()):
                self.map[i][j] = self.solver.BoolVar("")
        return True

    def create_constraints(self):
        for ps in self.garment1.get_p2smat():
            self.solver.Add(
                self.solver.Sum(ps[i] * self.xs1[i] for i in range(self.garment1.num_segments())) == 1
            )
        for ps in self.garment2.get_p2smat():
            self.solver.Add(
                self.solver.Sum(ps[i] * self.xs2[i] for i in range(self.garment2.num_segments())) == 1
            )

        for i, row in enumerate(self.map):
            self.solver.Add(self.solver.Sum(row) == self.xs1[i])
        for i, column in enumerate(zip(*self.map)):
            self.solver.Add(self.solver.Sum(column) == self.xs2[i])
        return True

    def create_objectives(self):
        objectives = []
        for i in range(self.garment1.num_segments()):
            for j in range(self.garment2.num_segments()):
                objectives.append(self.distMat[i][j] * self.map[i][j])
        self.solver.Minimize(self.solver.Sum(objectives))
        return True

    def solve(self):
        assert self.precompute_distMat()
        assert self.create_variables()
        assert self.create_constraints()
        assert self.create_objectives()

        status = self.solver.Solve()
        self.status = status
        return status
