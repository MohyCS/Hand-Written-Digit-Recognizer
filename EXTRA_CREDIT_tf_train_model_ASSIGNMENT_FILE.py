import tensorflow as tf
import numpy as np
import os
from tensorflow.examples.tutorials.mnist import input_data

class build_train:
    def __init__(self):
        self.rootPath = os.path.abspath(os.path.join(os.getcwd(), os.pardir))   # DO NOT EDIT
        self.save_dir = self.rootPath + '/tf_model'                             # DO NOT EDIT

    def build_train_network(self, network):

        ############### MNIST DATA #########################################
        mnist = input_data.read_data_sets("MNIST_data/", one_hot=True)      # DO NOT EDIT
        ############### END OF MNIST DATA ##################################

        ############### CONSTRUCT NEURAL NETWORK MODEL HERE ################

        # MODEL
        # INPUT MUST BE 784 array in order be able to train on MNIST
        # INPUT PLACEHOLDERS MUST BE NAME AS name='ph_x' AND name='ph_y_'
        '''
        Follow following format for defining placeholders:
        x = tf.placeholder(data_type, array_shape, name='ph_x')
        y_ = tf.placeholder(data_type, array_shape, name='ph_y_')
        '''
        # OUTPUT VECTOR y MUST BE LENGTH 10, EACH OUTPUT NEURON CORRESPONDS TO A DIGIT 0-9
       
	x = tf.placeholder(tf.float32, shape=[None, 784])
	y_ = tf.placeholder(tf.float32, shape=[None, 10])

	W = tf.Variable(tf.zeros([784,10]))
	b = tf.Variable(tf.zeros([10]))

	y = tf.matmul(x,W) + b

	cross_entropy = tf.reduce_mean(
		tf.nn.softmax_cross_entropy_with_logits(labels=y_, logits=y))

 ################### YOUR MODEL GOES HERE ######################################################################
        ################### YOUR MODEL GOES HERE ######################################################################
        ################### YOUR MODEL GOES HERE ######################################################################
        ################### YOUR MODEL GOES HERE ######################################################################
        ################### YOUR MODEL GOES HERE ######################################################################
        ################### YOUR MODEL GOES HERE ######################################################################
        ################### YOUR MODEL GOES HERE ######################################################################


        # LOSS FUNCTION, PREDICTION FUNCTION, ACCURACY FUNCTIONS
        # MAKE SURE ACCURCY FUNCTION IS NAMED ---name='op_accuracy'----
        '''
        EXAMPLE OF NAMING ACCURACY FUNCTION:
        accuracy = tf.reduce_mean(tf.cast(prediction, tf.float32), name='op_accuracy')
        '''
        ############## YOUR LOSS AND ACCURACY FUNCTIONS GO HERE #######################################################
	correct_prediction = tf.equal(tf.argmax(y,1), tf.argmax(y_,1))
	accuracy = tf.reduce_mean(tf.cast(correct_prediction, tf.float32))
	print(accuracy.eval(feed_dict={x: mnist.test.images, y_: mnist.test.labels}))

        ############# END OF NEURAL NETWORK MODEL ##########################


	train_step = tf.train.GradientDescentOptimizer(0.5).minimize(cross_entropy)



        ############# CONSTRUCT TRAINING FUNCTION ##########################

        # TRAINING FUNCTION SHOULD USE YOUR LOSS FUNCTION TO OPTIMIZE THE MODEL PARAMETERS
        ############## YOUR TRAINING FUNCTION GOES HERE ###############################################################
        ############## YOUR TRAINING FUNCTION GOES HERE ###############################################################
        ############## YOUR TRAINING FUNCTION GOES HERE ###############################################################
        ############## YOUR TRAINING FUNCTION GOES HERE ###############################################################

        ############# END OF TRAINING FUNCTION #############################


        ############# CONSTRUCT TRAINING SESSION ###########################
        saver = tf.train.Saver()                                            # DO NOT EDIT
        sess = tf.InteractiveSession()                                      # DO NOT EDIT
        sess.run(tf.global_variables_initializer())                         # DO NOT EDIT

        ############## YOUR TRAINING LOOP CODE GOES HERE #############################################################
	for _ in range(1000):
		batch = mnist.train.next_batch(100)
		train_step.run(feed_dict={x: batch[0], y_: batch[1]})
	


        ############# SAVE MODEL ###########################################

        saver.save(sess, save_path=self.save_dir, global_step=network)      # DO NOT EDIT
        print('Model Saved')                                                # DO NOT EDIT
        sess.close()                                                        # DO NOT EDIT
        ############# END OF SAVE MODEL ####################################

        ############# OUTPUT ACCURACY PLOT ################################

        ############## YOUR MODEL ACCURCY PLOT CODE GOES HERE ########################################################
        ############## YOUR MODEL ACCURCY PLOT CODE GOES HERE ########################################################
        ############## YOUR MODEL ACCURCY PLOT CODE GOES HERE ########################################################
        ############## YOUR MODEL ACCURCY PLOT CODE GOES HERE ########################################################

        ############# END OF ACCURACY PLOT ################################


