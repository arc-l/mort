class Action:
	def __init__(self, obj_id, target):
		self.obj_id = obj_id
		self.target = target  # 1 for goto buffer, 2 for goto goal, otherwise invalid
