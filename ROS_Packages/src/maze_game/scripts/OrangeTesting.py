import Orange
import random

data = Orange.data.Table("voting")
test = Orange.data.Table(random.sample(data, 5))
train = Orange.data.Table([d for d in data if d not in test])

tree = Orange.regression.tree.TreeLearner(train, some_majority_pruning=1, m_pruning=2)
tree.name = "tree"
knn = Orange.classification.knn.kNNLearner(train, k=21)
knn.name = "k-NN"
lr = Orange.classification.logreg.LogRegLearner(train)
lr.name = "lr"

classifiers = [tree, knn, lr]

tree.dot(file_name="/home/matthew/0.dot", node_shape="ellipse", leaf_shape="box")