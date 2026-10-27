"""A fixed target training tree of only valid-normal points, with original IDs."""
import numpy as np
from scipy.spatial import cKDTree
class GoodNormalTree:
 def __init__(self,points,good):
  self.original_indices=np.flatnonzero(np.asarray(good,dtype=bool))
  if not len(self.original_indices):raise ValueError('no valid-normal target training points')
  self.tree=cKDTree(np.asarray(points)[self.original_indices])
 def query(self,points,**kwargs):
  distance,subindex=self.tree.query(points,**kwargs)
  return distance,self.original_indices[subindex]
