import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt
from tensorflow.examples.tutorials.mnist import input_data
mnist = input_data.read_data_sets("/tmp/data/", one_hot=True)
learning_rate=0.001
epochs=10
batch_size=100
n_samples=mnist.train.num_examples
print(n_samples)
n_classes=10
n_input=784
n_hidden_1=600
n_hidden_2=500
x=tf.placeholder('float',[None,n_input])
y=tf.placeholder('float',[None,n_classes])
def multilayer_perceptron(x,weights,biases):
	#First Hidden Layer using relu
	layer_1=tf.add(tf.matmul(x,weights['h1']),biases['b1'])
	layer_1=tf.nn.relu(layer_1)
	layer_2=tf.add(tf.matmul(layer_1,weights['h2']),biases['b2'])
	layer_2=tf.nn.relu(layer_2)
	out_layer=tf.matmul(layer_2,weights['out'])+biases['out']
	return out_layer

weights = {
    'h1': tf.Variable(tf.random_normal([n_input, n_hidden_1])),
    'h2': tf.Variable(tf.random_normal([n_hidden_1, n_hidden_2])),
    'out': tf.Variable(tf.random_normal([n_hidden_2, n_classes]))
}
biases = {
    'b1': tf.Variable(tf.random_normal([n_hidden_1])),
    'b2': tf.Variable(tf.random_normal([n_hidden_2])),
    'out': tf.Variable(tf.random_normal([n_classes]))
}
pred=multilayer_perceptron(x,weights,biases)
cost =tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits( logits=pred, labels=y))
optimizer = tf.train.AdamOptimizer(learning_rate=learning_rate).minimize(cost)
init = tf.global_variables_initializer()
with tf.Session() as sess:
	sess.run(init)
	costs=[]
	for epoch in range(epochs):
		total_batch = int(n_samples/batch_size)
		avg_cost=0.0
		for batch in range(total_batch):
			x_batch,y_batch=mnist.train.next_batch(batch_size)
			c,_=sess.run([cost,optimizer],feed_dict={x:x_batch,y:y_batch})
			avg_cost+=c/batch_size;
		costs.append(avg_cost)
	print(costs)
	plt.plot(range(epochs),costs)
	plt.show()	
	correct_predictions = tf.equal(tf.argmax(pred, 1), tf.argmax(y, 1))
	print(pred)
	correct_predictions = tf.cast(correct_predictions, "float")
	accuracy = tf.reduce_mean(correct_predictions)
	print("Accuracy:", accuracy.eval({x: mnist.test.images, y: mnist.test.labels}))


