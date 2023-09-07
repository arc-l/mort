class DAG:
	def __init__(self, num, bottom, up):
		self.num = num
		self.bot = bottom  # list of bottom obj
		self.up = up  # list of lists

	def get_topo_order(self):
		"""get topological order of the DAG, from bottom to top"""
		chn = [0] * self.num
		topo_order = []
		for i in range(self.num):
			for j in self.up[i]:
				chn[j] += 1
		q = []
		for v in self.bot:
			q.append(v)
		while q:
			u = q.pop(0)
			topo_order.append(u)
			for v in self.up[u]:
				chn[v] -= 1
				if chn[v] == 0:
					q.append(v)
		return topo_order
