import math
import time

import gurobipy as grb
import pybullet as p
import pybullet_data
from action import Action
from constants import ObjStage, SimOptions
from wrapt_timeout_decorator import timeout


class Solver:
	def __init__(self, num, start, goal, goal_start_constraint, scale, shape, obj_to_start_position, obj_to_goal_position, output="action_sequence.txt", sim_type=SimOptions.GUI, verbose=False):
		self.num = num
		self.start, self.goal = start, goal
		self.goal_start_constraint = goal_start_constraint  # only for bottom objects in start
		self.scale = scale
		self.shape = shape
		self.output = output
		self.action_list = []
		self.env = grb.Env(empty=True)
		self.sim_type = sim_type
		self.verbose = verbose
		self.env.setParam("OutputFlag", 0)
		self.env.start()
		self.obj_to_start_position = obj_to_start_position
		self.square = math.ceil(math.sqrt(self.num))
		self.curr_buff_count = 0
		self.ilp_buff_count = -1
		self.greedy_buff_count = -1
		self.obj_to_goal_position = obj_to_goal_position
		self.obj_to_sim_id = [None] * self.num
		self.alr_solved = set()
		self.obj_to_colors = None

	def init_alr_solved(self, solved):
		self.alr_solved.update(solved)

	def init_colors(self, colors):
		self.obj_to_colors = colors

	def propagate_goal_start_constraint(self, topo_order):
		"""proprogate goal_start_constraint to upper goal layers as the goal start constraint is initialized only for bottom layer"""
		for u in topo_order:
			self.goal_start_constraint[u] = [*set(self.goal_start_constraint[u])]
			for v in self.goal.up[u]:
				for val in self.goal_start_constraint[u]:
					self.goal_start_constraint[v].append(val)

	def get_dependencies(self):
		# number of lower obj in goal
		dep = [0] * self.num
		for i in range(self.num):
			for j in self.goal.up[i]:
				dep[j] += 1

		# stage in the manipulation
		stage = [ObjStage.START] * self.num
		for v in range(self.num):
			if v in self.alr_solved:
				stage[v] = ObjStage.GOAL
				for obj in self.goal.up[v]:
					dep[obj] -= 1
		return dep, stage

	def init_start_pose(self):
		"""initializes start pose"""
		if self.sim_type == SimOptions.NONE:
			return
		self.obj_to_expected_position = self.obj_to_start_position.copy()
		self.curr_buff_count = 0
		if self.sim_type >= SimOptions.GUI: p.connect(p.GUI)
		elif self.sim_type == SimOptions.DIRECT: p.connect(p.DIRECT)
		p.setGravity(0, 0, -9.8)
		p.setTimeStep(1./240.)
		# p.setRealTimeSimulation(True)
		if self.sim_type >= SimOptions.GUI:
			p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
			# p.resetDebugVisualizerCamera(cameraTargetPosition=(0, 0, 0), cameraDistance=20*self.scale, cameraPitch=-35, cameraYaw=-130)
			p.resetDebugVisualizerCamera(cameraTargetPosition=(0, 0, 0), cameraDistance=20*self.scale, cameraPitch=-35, cameraYaw=-100)
			# p.resetDebugVisualizerCamera(cameraTargetPosition=(0, 0, 0), cameraDistance=15*self.scale, cameraPitch=-35, cameraYaw=50)
		p.setAdditionalSearchPath(pybullet_data.getDataPath())
		p.loadURDF("plane.urdf")
		if self.shape == p.GEOM_BOX:
			size = [self.scale, self.scale, self.scale]
			self.visual = p.createVisualShape(p.GEOM_BOX, halfExtents=size)
			self.coll_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=size)
		elif self.shape == p.GEOM_CYLINDER:
			self.visual = p.createVisualShape(p.GEOM_CYLINDER, radius=1.25*self.scale, length=2*self.scale)
			self.coll_shape = p.createCollisionShape(p.GEOM_CYLINDER, radius=1.25*self.scale, height=2*self.scale)

		for obj, pos in enumerate(self.obj_to_start_position):
			sim_id = p.createMultiBody(baseMass=10, baseCollisionShapeIndex=self.coll_shape, baseVisualShapeIndex=self.visual, basePosition=pos)
			if self.sim_type >= SimOptions.GUI:
				if self.obj_to_colors:
					p.changeVisualShape(sim_id, -1, rgbaColor=self.obj_to_colors[obj])
			self.obj_to_sim_id[obj] = sim_id
		self.start_id = p.saveState()
		if self.sim_type >= SimOptions.GUI:
			for _ in range(100):
				p.stepSimulation()
				time.sleep(1./240.)
			if self.sim_type == SimOptions.GUI_INTERACTIVE:
				input("Press Enter to start the simulation...")

	def write_action(self, res=None):
		if self.sim_type == SimOptions.NONE:
			return
		if not res: res = self.output
		with open(res, "w") as f:
			for action in self.action_list:
				f.write("Obj " + str(action.obj_id) + " -> " + ("buffer" if action.target == ObjStage.BUFF else "goal") + "\n")
		self.action_list = []

	def check_move_stability(self, obj, action, stage):
		if self.sim_type == SimOptions.NONE:
			stage[obj] = action
			self.action_list.append(Action(obj, action))
			return True
		if self.verbose: print("Checking", obj, "to", ("buffer" if action == ObjStage.BUFF else "goal"))
		if self.sim_type >= SimOptions.GUI:
			p.changeVisualShape(self.obj_to_sim_id[obj], -1, rgbaColor=(0, 0, 0, 1))
			time.sleep(0.1)
			color = self.obj_to_colors[obj] if self.obj_to_colors else (1, 1, 1, 1)
			p.changeVisualShape(self.obj_to_sim_id[obj], -1, rgbaColor=color)
			time.sleep(0.1)
		prev = stage[obj]
		if action == ObjStage.BUFF:
			side_length = 2*self.scale
			x = round(1.5*side_length*(1 + (self.curr_buff_count % self.square)), 2)
			y = round(1.5*side_length*(1 + self.curr_buff_count // self.square), 2)
			buff_pos = (-x, -y, self.scale)
			curr_orn = p.getBasePositionAndOrientation(self.obj_to_sim_id[obj])[1]
			p.resetBasePositionAndOrientation(self.obj_to_sim_id[obj], buff_pos, curr_orn)
			stage[obj] = ObjStage.BUFF
			self.curr_buff_count += 1
		elif action == ObjStage.GOAL:
			goal_pos = self.obj_to_goal_position[obj]
			p.resetBasePositionAndOrientation(self.obj_to_sim_id[obj], goal_pos, (0, 0, 0, 1))
			self.obj_to_expected_position[obj] = goal_pos
			stage[obj] = ObjStage.GOAL
		for _ in range(50):
			p.stepSimulation()
			if self.sim_type >= SimOptions.GUI: time.sleep(1./240.)
		# calculates distance from expected for each obj
		# unstable if greater than threshold
		for k, position in enumerate(self.obj_to_expected_position):
			if stage[k] == ObjStage.BUFF:
				continue
			curr_pos = p.getBasePositionAndOrientation(self.obj_to_sim_id[k])[0]
			dist_from_expected = math.dist(position, curr_pos)
			if dist_from_expected > 0.1:
				if self.sim_type >= SimOptions.GUI: print("Moving obj", obj, "to", ("buffer" if action == ObjStage.BUFF else "goal"), "is unstable!")
				stage[obj] = prev
				return False
		self.action_list.append(Action(obj, action))
		return True

	def check_buffer_to_goal(self, u, stage, curr_dep):
		if stage[u] != ObjStage.BUFF:
			return False
		if curr_dep[u] == 0:
			for v in self.goal_start_constraint[u]:
				if stage[v] == ObjStage.START:
					return False
			if self.sim_type != SimOptions.NONE:
				prev = p.saveState()
				if not self.check_move_stability(u, ObjStage.GOAL, stage):
					p.restoreState(prev)
					p.removeState(prev)
					if self.verbose: print(stage)
					return False
				p.removeState(prev)
			else:
				self.check_move_stability(u, ObjStage.GOAL, stage)
			for v in self.goal.up[u]:
				curr_dep[v] -= 1
			return True

	def add_stability_constraint(self, obj_id, permu, seq, model):
		unstable_move = round(permu[obj_id].X)
		seq_vars = [None] * unstable_move
		if self.verbose:
			print("unstable_obj:", obj_id)
			print("unstable_move:", unstable_move)
			print("seq:", seq)
		for i in range(unstable_move):
			name = "seq_obj_" + str(seq[i]) + "_lt_" + str(unstable_move)
			existing_var = model.getVarByName(name)
			if self.verbose: print(existing_var)
			seq_vars[i] = existing_var if existing_var else model.addVar(vtype=grb.GRB.BINARY, name=name)
			# (permu[seq[i]] < unstable_move) -> seq_vars[i] == 1
			model.addConstr((seq_vars[i] == 0) >> (permu[seq[i]] >= unstable_move), name)
		and_constr_var = model.addVar(vtype=grb.GRB.BINARY)
		model.addConstr(and_constr_var == grb.and_(seq_vars))
		model.addConstr((and_constr_var == 1) >> (permu[obj_id] >= unstable_move + 1), "obj_" + str(obj_id) + "_unstable_move_" + str(unstable_move))
		model.optimize()
		if self.verbose:
			print("seq_vars:", seq_vars)
			print(permu[obj_id], unstable_move)

	def generate_action_sequence(self, permu, buff, topo_order, model):
		self.init_start_pose()

		stable = False
		while not stable:
			curr_dep, stage = self.get_dependencies()
			seq = [None] * self.num
			for obj in range(self.num):
				seq_num = round(permu[obj].X)
				seq[seq_num] = obj
			for obj_id in seq:
				if stage[obj_id] == ObjStage.GOAL:
					stable = True
					if self.verbose: print(obj_id, "(already solved):", buff[obj_id].X)
					continue
				if buff[obj_id].X == 1.0:
					stable = self.check_move_stability(obj_id, ObjStage.BUFF, stage)
				else:
					stable = self.check_move_stability(obj_id, ObjStage.GOAL, stage)
					if stable:
						for v in self.goal.up[obj_id]:
							curr_dep[v] -= 1
				if not stable and self.sim_type != SimOptions.NONE:
					self.obj_to_expected_position = self.obj_to_start_position.copy()
					self.curr_buff_count = 0
					self.action_list = []
					self.add_stability_constraint(obj_id, permu, seq, model)
					if model.Status == grb.GRB.INFEASIBLE:
						print("Infeasible!")
						p.disconnect()
						return
					p.restoreState(self.start_id)
					break
				# greedily empty buffer if it can be put in goal
				for i, u in enumerate(topo_order):
					success = self.check_buffer_to_goal(u, stage, curr_dep)
					while success:
						success = False
						# check earlier nodes in topo_order
						for j in range(i):
							if self.check_buffer_to_goal(topo_order[j], stage, curr_dep):
								success = True
		if self.sim_type != SimOptions.NONE: p.removeState(self.start_id)
		self.write_action()
		if self.sim_type == SimOptions.GUI_INTERACTIVE: input("Press Enter to close the simulation...")
		if self.sim_type != SimOptions.NONE: p.disconnect()

	@timeout(300)
	def ilp_permutation_solver(self):
		# build ilp model
		model = grb.Model("mip1", env=self.env)

		# obj i leaves its start at permu[i]
		permu = [None] * self.num
		buff = [None] * self.num

		for i in range(self.num):
			permu[i] = model.addVar(0.0, self.num, 0.0, grb.GRB.INTEGER, "permu_" + str(i))
			buff[i] = model.addVar(0.0, 1.0, 0.0, grb.GRB.BINARY, "permu_" + str(i))

			model.addConstr(permu[i] >= 0, "permu_" + str(i) + "_lower")
			model.addConstr(permu[i] <= self.num - 1, "permu_" + str(i) + "_upper")

		for i in range(self.num):
			for j in range(i + 1, self.num):
				yij = model.addVar(0.0, self.num, 0.0, grb.GRB.BINARY, "neq_" + str(i) + "_" + str(j))
				yji = model.addVar(0.0, self.num, 0.0, grb.GRB.BINARY, "neq_" + str(j) + "_" + str(i))

				# pi != pj
				model.addConstr(permu[i] >= permu[j] + 1 - self.num * (1 - yij), "permu_" + str(i) + "_" + str(j))
				model.addConstr(permu[j] >= permu[i] + 1 - self.num * (1 - yji), "permu_" + str(j) + "_" + str(i))
				model.addConstr(yij + yji >= 1)

		for i in range(self.num):
			if i in self.alr_solved:
				continue
			# j directly above i in start dag
			# pj < pi
			for j in self.start.up[i]:
				model.addConstr(permu[i] >= permu[j] + 1, "start_layer_" + str(i) + "_" + str(j))
				if self.verbose:
					print("pi < pj: Obj" + str(i) + " > Obj" + str(j))

			# j above i in goal dag
			# (pj < pi) -> buff[j]
			queue = [i]
			pushed = set([i])
			while queue:
				j = queue[0]
				queue.pop(0)
				for nj in self.goal.up[j]:
					if nj not in pushed:
						queue.append(nj)
						pushed.add(nj)
				if j == i: continue
				model.addConstr(buff[j] * self.num >= permu[i] - permu[j], "goal_layer_" + str(i) + "_" + str(j))
				if self.verbose:
					print("(pj < pi)2 -> buff[j]: p" + str(j) + " < p" + str(i) + " -> buff[" + str(j) + "]")

		topo_order = self.goal.get_topo_order()
		self.propagate_goal_start_constraint(topo_order)

		for j in range(self.num):
			for i in self.goal_start_constraint[j]:
				# j goal pose is in collision with i start pose
				# (pj < pi) -> buff[j]
				model.addConstr(buff[j] * self.num >= (permu[i] - permu[j] + 1), "goal_start_" + str(i) + "_" + str(j))
				if self.verbose:
					print("(pj < pi) -> buff[j]: p" + str(j) + " < p" + str(i) + " -> buff[" + str(j) + "]")

		buff_sum = grb.quicksum(buff)
		model.setObjective(buff_sum, grb.GRB.MINIMIZE)
		model.optimize()

		buff_count = int(model.getObjective().getValue())
		if self.verbose: print("Buffer num:", buff_count)

		self.generate_action_sequence(permu, buff, topo_order, model)
		self.ilp_buff_count = buff_count
		return buff_count

	def bfs(self, graph, source):
		seen = []
		q = [source]
		while q:
			u = q.pop(0)
			if u not in seen: seen.append(u)
			for v in graph[u]:
				q.append(v)
		return seen

	def check_goal_available(self, u, stage, dep):
		if dep[u] == 0:
			for v in self.goal_start_constraint[u]:
				if stage[v] == ObjStage.START:
					return False
			for v in self.goal.up[u]:
				dep[v] -= 1
			return True
		return False

	def greedy_solver(self):
		def quit_fail():
			p.disconnect()
			self.action_list = []

		buff_count = 0
		topo_order = self.goal.get_topo_order()
		self.propagate_goal_start_constraint(topo_order)
		self.init_start_pose()
		dep, stage = self.get_dependencies()
		for v in topo_order:
			if stage[v] == ObjStage.GOAL:
				continue
			if stage[v] == ObjStage.START:
				# free obj v's start pose
				dependencies = self.bfs(self.start.up, v)[:0:-1]
				for u in dependencies:
					if stage[u] == ObjStage.START:
						# check if obj can be moved to goal
						if self.check_goal_available(u, stage, dep):
							stable = self.check_move_stability(u, ObjStage.GOAL, stage)
						else:
							stable = self.check_move_stability(u, ObjStage.BUFF, stage)
							buff_count += 1
						if not stable:
							quit_fail()
							return -1
			# free obj v's goal pose
			for u in self.goal_start_constraint[v]:
				dependencies = self.bfs(self.start.up, u)[::-1]
				for j in dependencies:
					if stage[j] == ObjStage.START:
						if self.check_goal_available(j, stage, dep):
							stable = self.check_move_stability(j, ObjStage.GOAL, stage)
						else:
							stable = self.check_move_stability(j, ObjStage.BUFF, stage)
							buff_count += 1
						if not stable:
							quit_fail()
							return -1
			stable = self.check_move_stability(v, ObjStage.GOAL, stage)
			if not stable:
				quit_fail()
				return -1
			for v in self.goal.up[v]:
				dep[v] -= 1
			# greedily empty buffer if it can be put in goal
			for i, u in enumerate(topo_order):
				success = self.check_buffer_to_goal(u, stage, dep)
				while success:
					success = False
					# check previous nodes in topo_order
					for j in range(i):
						if self.check_buffer_to_goal(topo_order[j], stage, dep):
							success = True
		self.write_action('action_sequence_greedy.txt')
		if self.sim_type != SimOptions.NONE: p.disconnect()
		if self.verbose: print("Buffer num:", buff_count)
		self.greedy_buff_count = buff_count
		return buff_count
